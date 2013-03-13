// Copyright 2008 Isis Innovation Limited
#include "OpenGL.h"
#include "Tracker.h"
#include "MEstimator.h"
#include "ShiTomasi.h"
#include "SmallMatrixOpts.h"
#include "PatchFinder.h"
#include "TrackerData.h"

#include <cvd/utility.h>
#include <cvd/gl_helpers.h>
#include <cvd/fast_corner.h>
#include <cvd/vision.h>
#include <TooN/wls.h>
#include <gvars3/instances.h>
#include <gvars3/GStringUtil.h>

#include <fstream>
#include <fcntl.h>
#include <iomanip>
#include <math.h>
#include <assert.h>
#include <opencv/cv.h>

// <custom> custom includes
#include "CustomSettings.h"
#include "shm_socket/shm_socket.h"
#include <sys/time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <math.h>
#include <cvd/image_io.h>

namespace PTAMM {

    using namespace CVD;
    using namespace std;
    using namespace GVars3;

    /**
     * The constructor mostly sets up interal reference variables to the other classes..
     * @param irVideoSize video image size
     * @param c camera model
     * @param maps list of maps
     * @param m current map
     * @param mm map maker
     */
    Tracker::Tracker(ImageRef irVideoSize, const ATANCamera &c, std::vector<Map*> &maps, Map *m, MapMaker &mm, CVD::Image<CVD::Rgb<CVD::byte> > *colorFrame, int cameraNumber) :
    mCurrentKF(c), mvpMaps(maps), mpMap(m), mMapMaker(mm), mCamera(c), mRelocaliser(maps, mCamera), mirSize(irVideoSize), mFirstKF(mCamera), mPreviousFrameKF(mCamera) {
        this->cameraNumber = cameraNumber;
        this->colorFrame = colorFrame;
        mCurrentKF.bFixed = false;
        GUI.RegisterCommand("Reset", GUICommandCallBack, this);
        //  GUI.RegisterCommand("KeyPress", GUICommandCallBack, this);
        GUI.RegisterCommand("PokeTracker", GUICommandCallBack, this);

        mpSBILastFrame = NULL;
        mpSBIThisFrame = NULL;

        // Most of the initialisation is done in Reset()
        Reset();
    }

    /**
     * Common reset code for Reset() and ResetAll()
     */
    void Tracker::ResetCommon() {
        mbDidCoarse = false;
        mbUserPressedSpacebar = false;
        mTrackingQuality = GOOD;
        mnLostFrames = 0;
        mdMSDScaledVelocityMagnitude = 0;
        mCurrentKF.dSceneDepthMean = 1.0;
        mCurrentKF.dSceneDepthSigma = 1.0;
        mnInitialStage = TRAIL_TRACKING_NOT_STARTED;
        mlTrails.clear();
        mCamera.SetImageSize(mirSize);
        mCurrentKF.mMeasurements.clear();
        mnLastKeyFrameDropped = -20;
        mnFrame = 0;
        mv6CameraVelocity = Zeros;
        mbJustRecoveredSoUseCoarse = false;

    }

    /**
     * Resets the tracker, wipes the map.
     * This is the main Reset-handler-entry-point of the program!
     * Other classes' resets propagate from here.
     * It's always called in the Tracker's thread, often as a GUI command.
     */
    void Tracker::Reset() {
        if (mpMap->bEditLocked) {
            cerr << "MAP LOCKED: Cannot reset map " << mpMap->MapID() << "." << endl;
            return;
        }

        ResetCommon();

        // Tell the MapMaker to reset itself.. 
        // this may take some time, since the mapmaker thread may have to wait
        // for an abort-check during calculation, so sleep while waiting.
        // MapMaker will also clear the map.
        mMapMaker.RequestReset();
        while (!mMapMaker.ResetDone())
#ifndef WIN32
            usleep(10);
#else
            Sleep(1);
#endif
    }

    // TrackFrame is called by System.cc with each incoming video frame.
    // It figures out what state the tracker is in, and calls appropriate internal tracking
    // functions. bDraw tells the tracker wether it should output any GL graphics
    // or not (it should not draw, for example, when AR stuff is being shown.)

    void Tracker::TrackFrame(Image<CVD::byte> &imFrame, bool bDraw) {
        mbDraw = bDraw;
        mMessageForUser.str(""); // Wipe the user message clean

        // Take the input video image, and convert it into the tracker's keyframe struct
        // This does things like generate the image pyramid and find FAST corners
        mCurrentKF.mMeasurements.clear();
        mCurrentKF.MakeKeyFrame_Lite(imFrame);

        // Update the small images for the rotation estimator
        static gvar3<double> gvdSBIBlur("Tracker.RotationEstimatorBlur", 0.75, SILENT);
        static gvar3<int> gvnUseSBI("Tracker.UseRotationEstimator", 1, SILENT);
        mbUseSBIInit = *gvnUseSBI;
        if (!mpSBIThisFrame) {
            mpSBIThisFrame = new SmallBlurryImage(mCurrentKF, *gvdSBIBlur);
            mpSBILastFrame = new SmallBlurryImage(mCurrentKF, *gvdSBIBlur);
        } else {
            delete mpSBILastFrame;
            mpSBILastFrame = mpSBIThisFrame;
            mpSBIThisFrame = new SmallBlurryImage(mCurrentKF, *gvdSBIBlur);
        }

        // From now on we only use the keyframe struct!
        mnFrame++;

        if (mbDraw) {
            glDrawPixels(mCurrentKF.aLevels[0].im);
            if (GV2.GetInt("Tracker.DrawFASTCorners", 0, SILENT)) {
                glColor3f(1, 0, 1);
                glPointSize(1);
                glBegin(GL_POINTS);
                for (unsigned int i = 0; i < mCurrentKF.aLevels[0].vCorners.size(); i++)
                    glVertex(mCurrentKF.aLevels[0].vCorners[i]);
                glEnd();
            }
        }

        // Decide what to do - if there is a map, try to track the map ...
        if (mpMap->IsGood()) {
            if (mnLostFrames < NUM_LOST_FRAMES) // .. but only if we're not lost!
            {
                if (mbUseSBIInit)
                    CalcSBIRotation();
                ApplyMotionModel(); // 
                TrackMap(); //  These three lines do the main tracking work.
                // <custom>
                PublishTrackedPosAndMap();
                // </custom>
                UpdateMotionModel();

                AssessTrackingQuality(); //  Check if we're lost or if tracking is poor.

                { // Provide some feedback for the user:
                    mMessageForUser << "Map quality: ";
                    if (mTrackingQuality == GOOD) mMessageForUser << "good.";
                    if (mTrackingQuality == DODGY) mMessageForUser << "poor.";
                    if (mTrackingQuality == BAD) mMessageForUser << "bad.";
                    mMessageForUser << " Conf: " << (int) (100 * getConfidence());
                    mMessageForUser << "% = " << getNumFoundPoints();
                    mMessageForUser << "/" << getNumAttemptedPoints();
                    mMessageForUser << " Map " << mpMap->MapID() << ": "
                            << mpMap->vpPoints.size() << "P, " << mpMap->vpKeyFrames.size() << "KF";
                }

                // Heuristics to check if a key-frame should be added to the map:
                bool needNewKeyFrame = false;
                if (!FAST_MAP_EXPANSION) {
                    needNewKeyFrame = mMapMaker.NeedNewKeyFrame(mCurrentKF);
                } else { //<ptamv>
                    double currentDistance = 0, requiredDistance = 0, currentAngle = 0, requiredAngle = 0;
                    needNewKeyFrame = mMapMaker.NeedNewKeyFrameExtMode(mCurrentKF, currentDistance, requiredDistance, currentAngle, requiredAngle);
                }
                if (mTrackingQuality == GOOD &&
                        needNewKeyFrame &&
                        mnFrame - mnLastKeyFrameDropped > 20 &&
                        mpMap->QueueSize() < 3) {
                    mMessageForUser << " Adding key-frame.";
                    AddNewKeyFrame();
                };
            } else // what if there is a map, but tracking has been lost?
            {
                mMessageForUser << "** Attempting recovery **.";
                if (AttemptRecovery()) {
                    TrackMap();
                    AssessTrackingQuality();
                }
            }
            if (mbDraw)
                RenderGrid();
        } else {
            // <custom>
            // If there is no map, try to make one.
            if (!SURF_INITIALIZATION)
                TrackForInitialMap();
            else {
                IplImage* imCVFrame = cvCreateImage(cvSize(mirSize.x, mirSize.y), IPL_DEPTH_8U, 1);
                memcpy(imCVFrame->imageData, imFrame.data(), mirSize.x * mirSize.y);
                TrackForInitialMapUsingSURF(*imCVFrame); // use original opencv frame -> need to store it beforehand
                cvReleaseImage(&imCVFrame);
            }
        }

        // GUI interface
        while (!mvQueuedCommands.empty()) {
            GUICommandHandler(mvQueuedCommands.begin()->sCommand, mvQueuedCommands.begin()->sParams);
            mvQueuedCommands.erase(mvQueuedCommands.begin());
        }
    };

    /**
     * Try to relocalise in case tracking was lost.
     * Returns success or failure as a bool.
     * Actually, the SBI relocaliser will almost always return true, even if
     * it has no idea where it is, so graphics will go a bit
     * crazy when lost. Could use a tighter SSD threshold and return more false,
     * but the way it is now gives a snappier response and I prefer it.
     * @return success
     */
    bool Tracker::AttemptRecovery() {
        bool bRelocGood = mRelocaliser.AttemptRecovery(*mpMap, mCurrentKF);
        if (!bRelocGood)
            return false;

        SE3<> se3Best = mRelocaliser.BestPose();
        mse3CamFromWorld = mse3StartPos = se3Best;
        mv6CameraVelocity = Zeros;
        mbJustRecoveredSoUseCoarse = true;
        return true;
    }

    /**
     * Draw the reference grid to give the user an idea of wether tracking is OK or not.
     */
    void Tracker::RenderGrid() {
        // The colour of the ref grid shows if the coarse stage of tracking was used
        // (it's turned off when the camera is sitting still to reduce jitter.)
        if (mbDidCoarse)
            glColor4f(0.0f, 0.5f, 0.0f, 0.6f);
        else
            glColor4f(0.0f, 0.0f, 0.0f, 0.6f);

        // The grid is projected manually, i.e. GL receives projected 2D coords to draw.
        int nHalfCells = 8;
        int nTot = nHalfCells * 2 + 1;
        Image<Vector < 2 > > imVertices(ImageRef(nTot, nTot));
        for (int i = 0; i < nTot; i++)
            for (int j = 0; j < nTot; j++) {
                Vector < 3 > v3;
                v3[0] = (i - nHalfCells) * 0.1;
                v3[1] = (j - nHalfCells) * 0.1;
                v3[2] = 0.0;
                Vector < 3 > v3Cam = mse3CamFromWorld * v3;
                if (v3Cam[2] < 0.001)
                    v3Cam[2] = 0.001;
                imVertices[i][j] = mCamera.Project(project(v3Cam));
            }
        glEnable(GL_LINE_SMOOTH);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glLineWidth(2);
        for (int i = 0; i < nTot; i++) {
            glBegin(GL_LINE_STRIP);
            for (int j = 0; j < nTot; j++)
                glVertex(imVertices[i][j]);
            glEnd();

            glBegin(GL_LINE_STRIP);
            for (int j = 0; j < nTot; j++)
                glVertex(imVertices[j][i]);
            glEnd();
        };

        glLineWidth(1);
        glColor3f(1, 0, 0);
    }

    /**
     * GUI interface. Stuff commands onto the back of a queue so the tracker handles
     * them in its own thread at the end of each frame. Note the charming lack of
     * any thread safety (no lock on mvQueuedCommands).
     * @param ptr object pointer
     * @param sCommand command string
     * @param sParams parameter string
     */
    void Tracker::GUICommandCallBack(void* ptr, string sCommand, string sParams) {
        Command c;
        c.sCommand = sCommand;
        c.sParams = sParams;
        ((Tracker*) ptr)->mvQueuedCommands.push_back(c);
    }

    /**
     * This is called in the tracker's own thread.
     * @param sCommand command string
     * @param sParams  parameter string
     */
    void Tracker::GUICommandHandler(string sCommand, string sParams) // Called by the callback func..
    {
        if (sCommand == "Reset") {
            Reset();
            return;
        } else if ((sCommand == "PokeTracker")) {
            mbUserPressedSpacebar = true;
            return;
        }

        cout << "! Tracker::GUICommandHandler: unhandled command " << sCommand << endl;
        exit(1);
    }

    /**
     * Handle a key press command
     * This is a change from PTAM to enable games to use keys
     * @param sKey the key pressed
     * @return true if the key was used.
     */
    bool Tracker::HandleKeyPress(string sKey) {
        // KeyPress commands are issued by GLWindow, and passed to Tracker via System
        if (sKey == "Space") {
            mbUserPressedSpacebar = true;
            return true;
        } else if (sKey == "1") { // <custom><secondcamera>
            if (cameraNumber == 1)
                mbUserPressedSpacebar = true;
        } else if (sKey == "2") { // <custom><secondcamera>
            if (cameraNumber == 2)
                mbUserPressedSpacebar = true;
        } else if (sKey == "r") {
            Reset();
            return true;
        } else if (sKey == "s") { // <custom><wallvizmap> save current frame
            saveWallvizFrame();
        } else if (sKey == "h") { // <custom> reset scale based on height measurement
            setSLAMScaleToMetricScale();
        } else if (sKey == "w") { // <custom> notify when wallviz can start building the map
            shm::shared_data* shdata = shm::shared_data::get_or_create();
            shdata->sig_wallviz_ready.set(mpMap->MapID());
        } else if (sKey == "d") { // <custom> notify when passing door
            shm::shared_data* shdata = shm::shared_data::get_or_create();
            shdata->sig_door.set(mpMap->MapID());
        } else if (sKey == "m") { // <custom> force map export
            PublishMapPoints();
        }

        return false;
    }

    /**
     * Routine for establishing the initial map. This requires two spacebar presses from the user
     * to define the first two key-frames. Salient points are tracked between the two keyframes
     * using cheap frame-to-frame tracking (which is very brittle - quick camera motion will
     * break it.) The salient points are stored in a list of `Trail' data structures.
     * What action TrackForInitialMap() takes depends on the mnInitialStage enum variable..
     */
    void Tracker::TrackForInitialMap() {
        // MiniPatch tracking threshhold.
        static gvar3<int> gvnMaxSSD("Tracker.MiniPatchMaxSSD", 100000, SILENT);
        MiniPatch::mnMaxSSD = *gvnMaxSSD;

        // <custom> spacebar pressed externally
        if (!mbUserPressedSpacebar) {
            shm::shared_data* shdata = shm::shared_data::get_or_create();
            mbUserPressedSpacebar = shdata->nav.get_spacebar_pressed();
        }

        // What stage of initial tracking are we at?
        if (mnInitialStage == TRAIL_TRACKING_NOT_STARTED) {
            if (mbUserPressedSpacebar) // First spacebar = this is the first keyframe
            {
                mbUserPressedSpacebar = false;
                TrailTracking_Start();
                mnInitialStage = TRAIL_TRACKING_STARTED;
            } else
                mMessageForUser << "Point camera at planar scene and press spacebar to start tracking for initial map.";
            return;
        };

        if (mnInitialStage == TRAIL_TRACKING_STARTED) {
            int nGoodTrails = TrailTracking_Advance(); // This call actually tracks the trails
            if (nGoodTrails < 10) // if most trails have been wiped out, no point continuing.
            {
                cout << "Number of matches too small, try again!!" << endl;
                Reset();
                return;
            }

            // If the user pressed spacebar here, use trails to run stereo and make the intial map..
            if (mbUserPressedSpacebar) {
                mbUserPressedSpacebar = false;
                vector<pair<ImageRef, ImageRef> > vMatches; // This is the format the mapmaker wants for the stereo pairs
                for (list<Trail>::iterator i = mlTrails.begin(); i != mlTrails.end(); i++)
                    vMatches.push_back(pair<ImageRef, ImageRef > (i->irInitialPos,
                        i->irCurrentPos));
                mMapMaker.InitFromStereo(mFirstKF, mCurrentKF, vMatches, mse3CamFromWorld); // This will take some time!
                mnInitialStage = TRAIL_TRACKING_COMPLETE;
            } else
                mMessageForUser << "Translate the camera slowly sideways, and press spacebar again to perform stereo init.";
        }
    }

    /**
     * The current frame is to be the first keyframe!
     */
    void Tracker::TrailTracking_Start() {
        mCurrentKF.MakeKeyFrame_Rest(); // This populates the Candidates list, which is Shi-Tomasi thresholded.
        mFirstKF = mCurrentKF;
        vector<pair<double, ImageRef> > vCornersAndSTScores;
        for (unsigned int i = 0; i < mCurrentKF.aLevels[0].vCandidates.size(); i++) // Copy candidates into a trivially sortable vector
        { // so that we can choose the image corners with max ST score
            Candidate &c = mCurrentKF.aLevels[0].vCandidates[i];
            if (!mCurrentKF.aLevels[0].im.in_image_with_border(c.irLevelPos, MiniPatch::mnHalfPatchSize))
                continue;
            vCornersAndSTScores.push_back(pair<double, ImageRef > (-1.0 * c.dSTScore, c.irLevelPos)); // negative so highest score first in sorted list
        };
        sort(vCornersAndSTScores.begin(), vCornersAndSTScores.end()); // Sort according to Shi-Tomasi score
        int nToAdd = GV2.GetInt("MaxInitialTrails", 1000, SILENT);
        for (unsigned int i = 0; i < vCornersAndSTScores.size() && nToAdd > 0; i++) {
            if (!mCurrentKF.aLevels[0].im.in_image_with_border(vCornersAndSTScores[i].second, MiniPatch::mnHalfPatchSize))
                continue;
            Trail t;
            t.mPatch.SampleFromImage(vCornersAndSTScores[i].second, mCurrentKF.aLevels[0].im);
            t.irInitialPos = vCornersAndSTScores[i].second;
            t.irCurrentPos = t.irInitialPos;
            mlTrails.push_back(t);
            nToAdd--;
        }
        mPreviousFrameKF = mFirstKF; // Always store the previous frame so married-matching can work.
    }

    /**
     * Steady-state trail tracking: Advance from the previous frame, remove duds.
     * @return number of good trails
     */
    int Tracker::TrailTracking_Advance() {
        int nGoodTrails = 0;
        if (mbDraw) {
            glPointSize(5);
            glLineWidth(2);
            glEnable(GL_POINT_SMOOTH);
            glEnable(GL_LINE_SMOOTH);
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
            glEnable(GL_BLEND);
            glBegin(GL_LINES);
        }

        MiniPatch BackwardsPatch;
        Level &lCurrentFrame = mCurrentKF.aLevels[0];
        Level &lPreviousFrame = mPreviousFrameKF.aLevels[0];

        for (list<Trail>::iterator i = mlTrails.begin(); i != mlTrails.end();) {
            list<Trail>::iterator next = i;
            next++;

            Trail &trail = *i;
            ImageRef irStart = trail.irCurrentPos;
            ImageRef irEnd = irStart;
            bool bFound = trail.mPatch.FindPatch(irEnd, lCurrentFrame.im, 10, lCurrentFrame.vCorners);
            if (bFound) {
                // Also find backwards in a married-matches check
                BackwardsPatch.SampleFromImage(irEnd, lCurrentFrame.im);
                ImageRef irBackWardsFound = irEnd;
                bFound = BackwardsPatch.FindPatch(irBackWardsFound, lPreviousFrame.im, 10, lPreviousFrame.vCorners);
                if ((irBackWardsFound - irStart).mag_squared() > 2)
                    bFound = false;

                trail.irCurrentPos = irEnd;
                nGoodTrails++;
            }
            if (mbDraw) {
                if (!bFound)
                    glColor3f(0, 1, 1); // Failed trails flash purple before dying.
                else
                    glColor3f(1, 1, 0);
                glVertex(trail.irInitialPos);
                if (bFound) glColor3f(1, 0, 0);
                glVertex(trail.irCurrentPos);
            }
            if (!bFound) // Erase from list of trails if not found this frame.
            {
                mlTrails.erase(i);
            }
            i = next;
        }
        if (mbDraw)
            glEnd();

        mPreviousFrameKF = mCurrentKF;
        return nGoodTrails;
    }

    /**
     * TrackMap is the main purpose of the Tracker.
     * It first projects all map points into the image to find a potentially-visible-set (PVS);
     * Then it tries to find some points of the PVS in the image;
     * Then it updates camera pose according to any points found.
     * Above may happen twice if a coarse tracking stage is performed.
     * Finally it updates the tracker's current-frame-KeyFrame struct with any
     * measurements made.
     * A lot of low-level functionality is split into helper classes:
     * class TrackerData handles the projection of a MapPoint and stores intermediate results;
     * class PatchFinder finds a projected MapPoint in the current-frame-KeyFrame.
     */
    void Tracker::TrackMap() {
        // Some accounting which will be used for tracking quality assessment:
        for (int i = 0; i < LEVELS; i++)
            manMeasAttempted[i] = manMeasFound[i] = 0;

        // The Potentially-Visible-Set (PVS) is split into pyramid levels.
        vector<TrackerData*> avPVS[LEVELS];
        for (int i = 0; i < LEVELS; i++)
            avPVS[i].reserve(500);

        // For all points in the map..
        for (unsigned int i = 0; i < mpMap->vpPoints.size(); i++) {
            MapPoint &p = *(mpMap->vpPoints[i]);
            // Ensure that this map point has an associated TrackerData struct.
            if (!p.pTData) p.pTData = new TrackerData(&p, mirSize);
            TrackerData &TData = *p.pTData;

            // Project according to current view, and if it's not in the image, skip.
            TData.Project(mse3CamFromWorld, mCamera);
            if (!TData.bInImage)
                continue;

            // Calculate camera projection derivatives of this point.
            TData.GetDerivsUnsafe(mCamera);

            // And check what the PatchFinder (included in TrackerData) makes of the mappoint in this view..
            TData.nSearchLevel = TData.Finder.CalcSearchLevelAndWarpMatrix(TData.Point, mse3CamFromWorld, TData.m2CamDerivs);
            if (TData.nSearchLevel == -1)
                continue; // a negative search pyramid level indicates an inappropriate warp for this view, so skip.

            // Otherwise, this point is suitable to be searched in the current image! Add to the PVS.
            TData.bSearched = false;
            TData.bFound = false;
            avPVS[TData.nSearchLevel].push_back(&TData);
        };

        // Next: A large degree of faffing about and deciding which points are going to be measured!
        // First, randomly shuffle the individual levels of the PVS.
        for (int i = 0; i < LEVELS; i++)
            random_shuffle(avPVS[i].begin(), avPVS[i].end());

        // The next two data structs contain the list of points which will next 
        // be searched for in the image, and then used in pose update.
        vector<TrackerData*> vNextToSearch;
        vector<TrackerData*> vIterationSet;

        // Tunable parameters to do with the coarse tracking stage:
        static gvar3<unsigned int> gvnCoarseMin("Tracker.CoarseMin", 20, SILENT); // Min number of large-scale features for coarse stage
        static gvar3<unsigned int> gvnCoarseMax("Tracker.CoarseMax", 60, SILENT); // Max number of large-scale features for coarse stage
        static gvar3<unsigned int> gvnCoarseRange("Tracker.CoarseRange", 30, SILENT); // Pixel search radius for coarse features
        static gvar3<int> gvnCoarseSubPixIts("Tracker.CoarseSubPixIts", 8, SILENT); // Max sub-pixel iterations for coarse features
        static gvar3<int> gvnCoarseDisabled("Tracker.DisableCoarse", 0, SILENT); // Set this to 1 to disable coarse stage (except after recovery)
        static gvar3<double> gvdCoarseMinVel("Tracker.CoarseMinVelocity", 0.006, SILENT); // Speed above which coarse stage is used.

        unsigned int nCoarseMax = *gvnCoarseMax;
        unsigned int nCoarseRange = *gvnCoarseRange;

        mbDidCoarse = false;

        // Set of heuristics to check if we should do a coarse tracking stage.
        bool bTryCoarse = true;
        if (*gvnCoarseDisabled || mdMSDScaledVelocityMagnitude < *gvdCoarseMinVel || nCoarseMax == 0)
            bTryCoarse = false;
        if (mbJustRecoveredSoUseCoarse) {
            bTryCoarse = true;
            nCoarseMax *= 2;
            nCoarseRange *= 2;
            mbJustRecoveredSoUseCoarse = false;
        };

        // If we do want to do a coarse stage, also check that there's enough high-level 
        // PV map points. We use the lowest-res two pyramid levels (LEVELS-1 and LEVELS-2),
        // with preference to LEVELS-1.
        if (bTryCoarse && avPVS[LEVELS - 1].size() + avPVS[LEVELS - 2].size() > *gvnCoarseMin) {
            // Now, fill the vNextToSearch struct with an appropriate number of 
            // TrackerDatas corresponding to coarse map points! This depends on how many
            // there are in different pyramid levels compared to CoarseMin and CoarseMax.

            if (avPVS[LEVELS - 1].size() <= nCoarseMax) { // Fewer than CoarseMax in LEVELS-1? then take all of them, and remove them from the PVS list.
                vNextToSearch = avPVS[LEVELS - 1];
                avPVS[LEVELS - 1].clear();
            } else { // ..otherwise choose nCoarseMax at random, again removing from the PVS list.
                for (unsigned int i = 0; i < nCoarseMax; i++)
                    vNextToSearch.push_back(avPVS[LEVELS - 1][i]);
                avPVS[LEVELS - 1].erase(avPVS[LEVELS - 1].begin(), avPVS[LEVELS - 1].begin() + nCoarseMax);
            }

            // If didn't source enough from LEVELS-1, get some from LEVELS-2... same as above.
            if (vNextToSearch.size() < nCoarseMax) {
                unsigned int nMoreCoarseNeeded = nCoarseMax - vNextToSearch.size();
                if (avPVS[LEVELS - 2].size() <= nMoreCoarseNeeded) {
                    vNextToSearch = avPVS[LEVELS - 2];
                    avPVS[LEVELS - 2].clear();
                } else {
                    for (unsigned int i = 0; i < nMoreCoarseNeeded; i++)
                        vNextToSearch.push_back(avPVS[LEVELS - 2][i]);
                    avPVS[LEVELS - 2].erase(avPVS[LEVELS - 2].begin(), avPVS[LEVELS - 2].begin() + nMoreCoarseNeeded);
                }
            }
            // Now go and attempt to find these points in the image!
            unsigned int nFound = SearchForPoints(vNextToSearch, nCoarseRange, *gvnCoarseSubPixIts);
            vIterationSet = vNextToSearch; // Copy over into the to-be-optimised list.
            if (nFound >= *gvnCoarseMin) // Were enough found to do any meaningful optimisation?
            {
                mbDidCoarse = true;
                for (int iter = 0; iter < 10; iter++) // If so: do ten Gauss-Newton pose updates iterations.
                {
                    if (iter != 0) { // Re-project the points on all but the first iteration.
                        for (unsigned int i = 0; i < vIterationSet.size(); i++)
                            if (vIterationSet[i]->bFound)
                                vIterationSet[i]->ProjectAndDerivs(mse3CamFromWorld, mCamera);
                    }
                    for (unsigned int i = 0; i < vIterationSet.size(); i++)
                        if (vIterationSet[i]->bFound)
                            vIterationSet[i]->CalcJacobian();
                    double dOverrideSigma = 0.0;
                    // Hack: force the MEstimator to be pretty brutal 
                    // with outliers beyond the fifth iteration.
                    if (iter > 5)
                        dOverrideSigma = 1.0;

                    // Calculate and apply the pose update...
                    Vector < 6 > v6Update = CalcPoseUpdate(vIterationSet, dOverrideSigma);
                    mse3CamFromWorld = SE3<>::exp(v6Update) * mse3CamFromWorld;
                };
            }
        };

        // So, at this stage, we may or may not have done a coarse tracking stage.
        // Now do the fine tracking stage. This needs many more points!

        int nFineRange = 10; // Pixel search range for the fine stage. 
        if (mbDidCoarse) // Can use a tighter search if the coarse stage was already done.
            nFineRange = 5;

        // What patches shall we use this time? The high-level ones are quite important,
        // so do all of these, with sub-pixel refinement.
        {
            int l = LEVELS - 1;
            for (unsigned int i = 0; i < avPVS[l].size(); i++)
                avPVS[l][i]->ProjectAndDerivs(mse3CamFromWorld, mCamera);
            SearchForPoints(avPVS[l], nFineRange, 8);
            for (unsigned int i = 0; i < avPVS[l].size(); i++)
                vIterationSet.push_back(avPVS[l][i]); // Again, plonk all searched points onto the (maybe already populate) vIterationSet.
        };

        // All the others levels: Initially, put all remaining potentially visible patches onto vNextToSearch.
        vNextToSearch.clear();
        for (int l = LEVELS - 2; l >= 0; l--)
            for (unsigned int i = 0; i < avPVS[l].size(); i++)
                vNextToSearch.push_back(avPVS[l][i]);

        // But we haven't got CPU to track _all_ patches in the map - arbitrarily limit 
        // ourselves to 1000, and choose these randomly.
        static gvar3<int> gvnMaxPatchesPerFrame("Tracker.MaxPatchesPerFrame", 1000, SILENT);
        int nFinePatchesToUse = *gvnMaxPatchesPerFrame - static_cast<int> (vIterationSet.size());
        if ((int) vNextToSearch.size() > nFinePatchesToUse) {
            random_shuffle(vNextToSearch.begin(), vNextToSearch.end());
            vNextToSearch.resize(nFinePatchesToUse); // Chop!
        };

        // If we did a coarse tracking stage: re-project and find derivs of fine points
        if (mbDidCoarse)
            for (unsigned int i = 0; i < vNextToSearch.size(); i++)
                vNextToSearch[i]->ProjectAndDerivs(mse3CamFromWorld, mCamera);

        // Find fine points in image:
        SearchForPoints(vNextToSearch, nFineRange, 0);
        // And attach them all to the end of the optimisation-set.
        for (unsigned int i = 0; i < vNextToSearch.size(); i++)
            vIterationSet.push_back(vNextToSearch[i]);

        // Again, ten gauss-newton pose update iterations.
        Vector < 6 > v6LastUpdate;
        v6LastUpdate = Zeros;
        for (int iter = 0; iter < 10; iter++) {
            bool bNonLinearIteration; // For a bit of time-saving: don't do full nonlinear
            // reprojection at every iteration - it really isn't necessary!
            if (iter == 0 || iter == 4 || iter == 9)
                bNonLinearIteration = true; // Even this is probably overkill, the reason we do many
            else // iterations is for M-Estimator convergence rather than 
                bNonLinearIteration = false; // linearisation effects.

            if (iter != 0) // Either way: first iteration doesn't need projection update.
            {
                if (bNonLinearIteration) {
                    for (unsigned int i = 0; i < vIterationSet.size(); i++)
                        if (vIterationSet[i]->bFound)
                            vIterationSet[i]->ProjectAndDerivs(mse3CamFromWorld, mCamera);
                } else {
                    for (unsigned int i = 0; i < vIterationSet.size(); i++)
                        if (vIterationSet[i]->bFound)
                            vIterationSet[i]->LinearUpdate(v6LastUpdate);
                };
            }

            if (bNonLinearIteration)
                for (unsigned int i = 0; i < vIterationSet.size(); i++)
                    if (vIterationSet[i]->bFound)
                        vIterationSet[i]->CalcJacobian();

            // Again, an M-Estimator hack beyond the fifth iteration.
            double dOverrideSigma = 0.0;
            if (iter > 5)
                dOverrideSigma = 16.0;

            // Calculate and update pose; also store update vector for linear iteration updates.
            Vector < 6 > v6Update =
                    CalcPoseUpdate(vIterationSet, dOverrideSigma, iter == 9);
            mse3CamFromWorld = SE3<>::exp(v6Update) * mse3CamFromWorld;
            v6LastUpdate = v6Update;
        };

        if (mbDraw) {
            glPointSize(6);
            glEnable(GL_BLEND);
            glEnable(GL_POINT_SMOOTH);
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
            glBegin(GL_POINTS);
            for (vector<TrackerData*>::reverse_iterator it = vIterationSet.rbegin();
                    it != vIterationSet.rend();
                    it++) {
                if (!(*it)->bFound)
                    continue;
                glColor(gavLevelColors[(*it)->nSearchLevel]);
                glVertex((*it)->v2Image);
            }
            glEnd();
            glDisable(GL_BLEND);
        }

        // Update the current keyframe with info on what was found in the frame.
        // Strictly speaking this is unnecessary to do every frame, it'll only be
        // needed if the KF gets added to MapMaker. Do it anyway.
        // Export pose to current keyframe:
        mCurrentKF.se3CfromW = mse3CamFromWorld;

        // Record successful measurements. Use the KeyFrame-Measurement struct for this.
        mCurrentKF.mMeasurements.clear();
        for (vector<TrackerData*>::iterator it = vIterationSet.begin();
                it != vIterationSet.end();
                it++) {
            if (!(*it)->bFound)
                continue;
            Measurement m;
            m.v2RootPos = (*it)->v2Found;
            m.v2ImplanePos = mCamera.UnProject(m.v2RootPos);
            m.m2CamDerivs = mCamera.GetProjectionDerivs();
            m.nLevel = (*it)->nSearchLevel;
            m.bSubPix = (*it)->bDidSubPix;
            mCurrentKF.mMeasurements[& ((*it)->Point)] = m;
        }

        // Finally, find the mean scene depth from tracked features
        {
            double dSum = 0;
            double dSumSq = 0;
            int nNum = 0;
            for (vector<TrackerData*>::iterator it = vIterationSet.begin();
                    it != vIterationSet.end();
                    it++)
                if ((*it)->bFound) {
                    double z = (*it)->v3Cam[2];
                    dSum += z;
                    dSumSq += z*z;
                    nNum++;
                };
            if (nNum > 20) {
                mCurrentKF.dSceneDepthMean = dSum / nNum;
                mCurrentKF.dSceneDepthSigma = sqrt((dSumSq / nNum) - (mCurrentKF.dSceneDepthMean) * (mCurrentKF.dSceneDepthMean));
            }
        }
    }

    /**
     * Find points in the image. Uses the PatchFiner struct stored in TrackerData
     * @param vTD tracker data
     * @param nRange search range
     * @param nSubPixIts number of sub-pixel iterations required
     * @return number of points found
     */
    int Tracker::SearchForPoints(vector<TrackerData*> &vTD, int nRange, int nSubPixIts) {
        int nFound = 0;
        for (unsigned int i = 0; i < vTD.size(); i++) // for each point..
        {
            // First, attempt a search at pixel locations which are FAST corners.
            // (PatchFinder::FindPatchCoarse)
            TrackerData &TD = *vTD[i];
            PatchFinder &Finder = TD.Finder;
            Finder.MakeTemplateCoarseCont(TD.Point);
            if (Finder.TemplateBad()) {
                TD.bInImage = TD.bPotentiallyVisible = TD.bFound = false;
                continue;
            }
            manMeasAttempted[Finder.GetLevel()]++; // Stats for tracking quality assessmenta

            bool bFound =
                    Finder.FindPatchCoarse(ir(TD.v2Image), mCurrentKF, nRange);
            TD.bSearched = true;
            if (!bFound) {
                TD.bFound = false;
                continue;
            }

            TD.bFound = true;
            TD.dSqrtInvNoise = (1.0 / Finder.GetLevelScale());

            nFound++;
            manMeasFound[Finder.GetLevel()]++;

            // Found the patch in coarse search - are Sub-pixel iterations wanted too?
            if (nSubPixIts > 0) {
                TD.bDidSubPix = true;
                Finder.MakeSubPixTemplate();
                bool bSubPixConverges = Finder.IterateSubPixToConvergence(mCurrentKF, nSubPixIts);
                if (!bSubPixConverges) { // If subpix doesn't converge, the patch location is probably very dubious!
                    TD.bFound = false;
                    nFound--;
                    manMeasFound[Finder.GetLevel()]--;
                    continue;
                }
                TD.v2Found = Finder.GetSubPixPos();
            } else {
                TD.v2Found = Finder.GetCoarsePosAsVector();
                TD.bDidSubPix = false;
            }
        }
        return nFound;
    }

    /**
     * Calculate a pose update 6-vector from a bunch of image measurements.
     * User-selectable M-Estimator.
     * Normally this robustly estimates a sigma-squared for all the measurements
     * to reduce outlier influence, but this can be overridden if
     * dOverrideSigma is positive. Also, bMarkOutliers set to true
     * records any instances of a point being marked an outlier measurement
     * by the Tukey MEstimator.
     * @param vTD tracker data
     * @param dOverrideSigma 
     * @param bMarkOutliers 
     * @return 
     */
    Vector < 6 > Tracker::CalcPoseUpdate(vector<TrackerData*> vTD, double dOverrideSigma, bool bMarkOutliers) {
        // Which M-estimator are we using?
        int nEstimator = 0;
        static gvar3<string> gvsEstimator("TrackerMEstimator", "Tukey", SILENT);
        if (*gvsEstimator == "Tukey")
            nEstimator = 0;
        else if (*gvsEstimator == "Cauchy")
            nEstimator = 1;
        else if (*gvsEstimator == "Huber")
            nEstimator = 2;
        else {
            cout << "Tracker: Invalid TrackerMEstimator, choices are Tukey, Cauchy, Huber" << endl;
            nEstimator = 0;
            *gvsEstimator = "Tukey";
        };

        // Find the covariance-scaled reprojection error for each measurement.
        // Also, store the square of these quantities for M-Estimator sigma squared estimation.
        vector<double> vdErrorSquared;
        for (unsigned int f = 0; f < vTD.size(); f++) {
            TrackerData &TD = *vTD[f];
            if (!TD.bFound)
                continue;
            TD.v2Error_CovScaled = TD.dSqrtInvNoise * (TD.v2Found - TD.v2Image);
            vdErrorSquared.push_back(TD.v2Error_CovScaled * TD.v2Error_CovScaled);
        };

        // No valid measurements? Return null update.
        if (vdErrorSquared.size() == 0)
            return makeVector(0, 0, 0, 0, 0, 0);

        // What is the distribution of errors?
        double dSigmaSquared;
        if (dOverrideSigma > 0)
            dSigmaSquared = dOverrideSigma; // Bit of a waste having stored the vector of square errors in this case!
        else {
            if (nEstimator == 0)
                dSigmaSquared = Tukey::FindSigmaSquared(vdErrorSquared);
            else if (nEstimator == 1)
                dSigmaSquared = Cauchy::FindSigmaSquared(vdErrorSquared);
            else
                dSigmaSquared = Huber::FindSigmaSquared(vdErrorSquared);
        }

        // The TooN WLSCholesky class handles reweighted least squares.
        // It just needs errors and jacobians.
        WLS < 6 > wls;
        wls.add_prior(100.0); // Stabilising prior
        for (unsigned int f = 0; f < vTD.size(); f++) {
            TrackerData &TD = *vTD[f];
            if (!TD.bFound)
                continue;
            Vector < 2 > &v2 = TD.v2Error_CovScaled;
            double dErrorSq = v2 * v2;
            double dWeight;

            if (nEstimator == 0)
                dWeight = Tukey::Weight(dErrorSq, dSigmaSquared);
            else if (nEstimator == 1)
                dWeight = Cauchy::Weight(dErrorSq, dSigmaSquared);
            else
                dWeight = Huber::Weight(dErrorSq, dSigmaSquared);

            // Inlier/outlier accounting, only really works for cut-off estimators such as Tukey.
            if (dWeight == 0.0) {
                if (bMarkOutliers)
                    TD.Point.nMEstimatorOutlierCount++;
                continue;
            } else if (bMarkOutliers)
                TD.Point.nMEstimatorInlierCount++;

            Matrix < 2, 6 > &m26Jac = TD.m26Jacobian;
            wls.add_mJ(v2[0], TD.dSqrtInvNoise * m26Jac[0], dWeight); // These two lines are currently
            wls.add_mJ(v2[1], TD.dSqrtInvNoise * m26Jac[1], dWeight); // the slowest bit of poseits
        }

        wls.compute();
        return wls.get_mu();
    }

    /**
     * Time to add a new keyframe? The MapMaker handles most of this.
     */
    void Tracker::AddNewKeyFrame() {
        mMapMaker.AddKeyFrame(mCurrentKF);
        mnLastKeyFrameDropped = mnFrame;
    }

    /**
     * Some heuristics to decide if tracking is any good, for this frame.
     * This influences decisions to add key-frames, and eventually
     * causes the tracker to attempt relocalisation.
     */
    void Tracker::AssessTrackingQuality() {
        int nTotalAttempted = 0;
        int nTotalFound = 0;
        int nLargeAttempted = 0;
        int nLargeFound = 0;

        for (int i = 0; i < LEVELS; i++) {
            nTotalAttempted += manMeasAttempted[i];
            nTotalFound += manMeasFound[i];
            if (i >= 2) nLargeAttempted += manMeasAttempted[i];
            if (i >= 2) nLargeFound += manMeasFound[i];
        }

        if (nTotalFound == 0 || nTotalAttempted == 0)
            mTrackingQuality = BAD;
        else {
            double dTotalFracFound = (double) nTotalFound / nTotalAttempted;
            double dLargeFracFound;
            if (nLargeAttempted > 10)
                dLargeFracFound = (double) nLargeFound / nLargeAttempted;
            else
                dLargeFracFound = dTotalFracFound;

            static gvar3<double> gvdQualityGood("Tracker.TrackingQualityGood", 0.3, SILENT);
            static gvar3<double> gvdQualityLost("Tracker.TrackingQualityLost", 0.13, SILENT);


            if (dTotalFracFound > *gvdQualityGood)
                mTrackingQuality = GOOD;
            else if (dLargeFracFound < *gvdQualityLost)
                mTrackingQuality = BAD;
            else
                mTrackingQuality = DODGY;
        }

        if (mTrackingQuality == DODGY) {
            // Further heuristics to see if it's actually bad, not just dodgy...
            // If the camera pose estimate has run miles away, it's probably bad.
            if (mMapMaker.IsDistanceToNearestKeyFrameExcessive(mCurrentKF))
                mTrackingQuality = BAD;
        }

        if (mTrackingQuality == BAD)
            mnLostFrames++;
        else
            mnLostFrames = 0;
    }

    /**
     * Return the user infor message
     * @return message string
     */
    string Tracker::GetMessageForUser() {
        return mMessageForUser.str();
    }

    /**
     * Calculate the rotation of the small blurry image descriptor
     */
    void Tracker::CalcSBIRotation() {
        mpSBILastFrame->MakeJacs();
        pair<SE2<>, double> result_pair;
        result_pair = mpSBIThisFrame->IteratePosRelToTarget(*mpSBILastFrame, 6);
        SE3<> se3Adjust = SmallBlurryImage::SE3fromSE2(mCurrentKF.aLevels[3].im.size(), result_pair.first, mCamera);
        mv6SBIRot = se3Adjust.ln();
    }

    /**
     * Switch the map to the specified map.
     * @param map the map to switch to
     */
    bool Tracker::SwitchMap(Map *map) {
        if (map == NULL) {
            return false;
        }

        if (mpMap == map) {
            return true;
        }

        //set variables
        SE3<> se3Best = mRelocaliser.BestPose();
        mse3CamFromWorld = mse3StartPos = se3Best;
        mv6CameraVelocity = Zeros;
        mbJustRecoveredSoUseCoarse = true;

        //set new map
        mpMap = map;

        return true;
    }

    /**
     * Re initialize the map.
     */
    void Tracker::SetNewMap(Map * map) {
        if (mpMap == map) {
            cerr << "*** WARNING Tracker::SetNewMap() map is the same. Aborting ***" << endl;
            return;
        }

        ResetCommon();

        mpMap = map;
    }

    /**
     * Just add the current velocity to the current pose.
     * N.b. this doesn't actually use time in any way, i.e. it assumes
     * a one-frame-per-second camera. Skipped frames etc
     * are not handled properly here.
     */
    void Tracker::ApplyMotionModel() {
        mse3StartPos = mse3CamFromWorld;
        Vector < 6 > v6Velocity = mv6CameraVelocity;
        if (mbUseSBIInit) {
            v6Velocity.slice < 3, 3 > () = mv6SBIRot.slice < 3, 3 > ();
            v6Velocity[0] = 0.0;
            v6Velocity[1] = 0.0; // v6Velocity[2] = ???
        }
        mse3CamFromWorld = SE3<>::exp(v6Velocity) * mse3StartPos;
    };

    /**
     * The motion model is entirely the tracker's, and is kept as a decaying
     * constant velocity model.
     */
    void Tracker::UpdateMotionModel() {
        SE3<> se3NewFromOld = mse3CamFromWorld * mse3StartPos.inverse();
        Vector < 6 > v6Motion = SE3<>::ln(se3NewFromOld);
        Vector < 6 > v6OldVel = mv6CameraVelocity;

        mv6CameraVelocity = 0.9 * (0.5 * v6Motion + 0.5 * v6OldVel);
        mdVelocityMagnitude = sqrt(mv6CameraVelocity * mv6CameraVelocity);

        // Also make an estimate of this which has been scaled by the mean scene depth.
        // This is used to decide if we should use a coarse tracking stage.
        // We can tolerate more translational vel when far away from scene!
        Vector < 6 > v6 = mv6CameraVelocity;
        v6.slice < 0, 3 > () *= 1.0 / mCurrentKF.dSceneDepthMean;
        mdMSDScaledVelocityMagnitude = sqrt(v6 * v6);
    }

    /***
     * <custom><ptamv>
     * 
     * Remove the columns in m where the corresponding column in keep is zero
     */
    CV_IMPL void cvRemoveMatrixCols(CvMat **m, CvMat *keep) {
        CV_FUNCNAME("cvCleanMatrixCols");

        int m_channels, m_type, keep_type, cols;
        CvMat *M;

        __CV_BEGIN__;

        // Both m and keep should be matrices, and have the same number of columns
        CV_ASSERT(CV_IS_MAT(*m) && CV_IS_MAT(keep) && ((*m)->cols == keep->cols));

        // keep must have only one channel
        CV_ASSERT((((keep->type & CV_MAT_TYPE_MASK) >> 3)) == 0);

        m_channels = (((*m)->type & CV_MAT_TYPE_MASK) >> 3) + 1;
        m_type = (((*m)->type & CV_MAT_TYPE_MASK) & 0b111);
        keep_type = ((keep->type & CV_MAT_TYPE_MASK) & 0b111);

        // Only use valid types (i.e. not user-defined types)
        CV_ASSERT(keep_type < 7 && m_type < 7);

        M = *m;
        cols = 0;
        switch (keep_type) {
            case 6:
                for (int i = 0; i < keep->cols; ++i)
                    if (keep->data.db[i])
                        ++cols;
                break;
            case 5:
                for (int i = 0; i < keep->cols; ++i)
                    if (keep->data.fl[i])
                        ++cols;
                break;
            case 4:
                for (int i = 0; i < keep->cols; ++i)
                    if (keep->data.i[i])
                        ++cols;
                break;
            case 3:
            case 2:
                for (int i = 0; i < keep->cols; ++i)
                    if (keep->data.s[i])
                        ++cols;
                break;
            case 1:
            case 0:
                for (int i = 0; i < keep->cols; ++i)
                    if (keep->data.ptr[i])
                        ++cols;
                break;
        }
        if (cols == 0) { // no columns left
            cvReleaseMat(&M);
            *m = NULL;
        } else {
            *m = cvCreateMat((*m)->rows, cols, (*m)->type);
            int new_col = 0;
            int keepIt = 0;
            for (int row = 0; row < (*m)->rows; ++row) {
                for (int old_col = 0; old_col < keep->cols; ++old_col) {
                    keepIt = 0;
                    switch (keep_type) {
                        case 6:
                            if (keep->data.db[old_col])
                                keepIt = 1;
                            break;
                        case 5:
                            if (keep->data.fl[old_col])
                                keepIt = 1;
                            break;
                        case 4:
                            if (keep->data.i[old_col])
                                keepIt = 1;
                            break;
                        case 3:
                        case 2:
                            if (keep->data.s[old_col])
                                keepIt = 1;
                            break;
                        case 1:
                        case 0:
                            if (keep->data.ptr[old_col])
                                keepIt = 1;
                            break;
                    }
                    if (keepIt) {
                        for (int c = 0; c < m_channels; ++c) {
                            switch (m_type) {
                                case 6:
                                    (*m)->data.db[(new_col + row * cols) * m_channels + c] = M->data.db[(old_col + row * M->cols)
                                            * m_channels + c];
                                    break;
                                case 5:
                                    (*m)->data.fl[(new_col + row * cols) * m_channels + c] = M->data.fl[(old_col + row * M->cols)
                                            * m_channels + c];
                                    break;
                                case 4:
                                    (*m)->data.i[(new_col + row * cols) * m_channels + c] = M->data.i[(old_col + row * M->cols)
                                            * m_channels + c];
                                    break;
                                case 3:
                                    (*m)->data.s[(new_col + row * cols) * m_channels + c] = M->data.s[(old_col + row * M->cols)
                                            * m_channels + c];
                                    break;
                                case 2:
                                    (*m)->data.s[(new_col + row * cols) * m_channels + c] = M->data.s[(old_col + row * M->cols)
                                            * m_channels + c];
                                    break;
                                case 1:
                                    (*m)->data.ptr[(new_col + row * cols) * m_channels + c] = M->data.ptr[(old_col + row * M->cols)
                                            * m_channels + c];
                                    break;
                                case 0:
                                    (*m)->data.ptr[(new_col + row * cols) * m_channels + c] = M->data.ptr[(old_col + row * M->cols)
                                            * m_channels + c];
                                    break;
                            }
                        }
                        ++new_col;
                    }
                }
            }
            cvReleaseMat(&M);
        }
        __CV_END__;
    }

    /***
     * <custom><ptamv>
     */
    void matchKeypoints(IplImage &img1, IplImage& img2, CvMat** points1Return, CvMat** points2Return) {
        CvMat *status = NULL;
        CvMat *F = NULL;
        CvMat *points1 = NULL, *points2 = NULL;
        CvSeq *kp1 = NULL, *kp2 = NULL;
        CvSeq *desc1 = NULL, *desc2 = NULL;
        CvMemStorage *storage = cvCreateMemStorage(0);

        CvSURFParams surfParams = cvSURFParams(600, 1);
        int descriptorSize = surfParams.extended ? 128 : 64;

        cout << "  Extracting features from img1 using SURF...";
        // <custom> Note: using opencv 2.1 (and not 2.3)
        cvExtractSURF(&img1, NULL, &kp1, &desc1, storage, surfParams);
        cout << " found " << kp1->total << " keypoints\n";

        cout << "  Extracting features from img2 using SURF...";
        // <custom> Note: using opencv 2.1 (and not 2.3)
        cvExtractSURF(&img2, NULL, &kp2, &desc2, storage, surfParams);
        cout << " found " << kp2->total << " keypoints\n";

        cout << "  Matching keypoints...";
        CvMat* desc1mat = cvCreateMat(kp1->total, descriptorSize, CV_32FC1);
        CvMat* desc2mat = cvCreateMat(kp2->total, descriptorSize, CV_32FC1);
        float *seq;
        for (int k = 0; k < kp1->total; k++) {
            seq = (float*) cvGetSeqElem(desc1, k);
            for (int d = 0; d < descriptorSize; d++)
                CV_MAT_ELEM(*desc1mat, float, k, d) = seq[d];
        }
        for (int k = 0; k < kp2->total; k++) {
            seq = (float*) cvGetSeqElem(desc2, k);
            for (int d = 0; d < descriptorSize; d++)
                CV_MAT_ELEM(*desc2mat, float, k, d) = seq[d];
        }
        //CvFeatureTree *ft = cvCreateFeatureTree(desc1mat);
        CvFeatureTree *ft = cvCreateKDTree(desc1mat);
        CvMat *matches = cvCreateMat(kp2->total, 1, CV_32SC1);
        CvMat *distances = cvCreateMat(kp2->total, 1, CV_64FC1);
        cvFindFeatures(ft, desc2mat, matches, distances, 1, 250);
        cvReleaseFeatureTree(ft);
        int *reverseLookup = (int*) malloc(sizeof (int) * kp1->total);
        double *reverseLookupDist = (double*) malloc(sizeof (double) * kp1->total);
        for (int i = 0; i < kp1->total; ++i) {
            reverseLookup[i] = -1;
            reverseLookupDist[i] = DBL_MAX;
        }
        int matchCount = 0;
        for (int j = 0; j < kp2->total; ++j) {
            int i = (int) cvGetReal2D(matches, j, 0);
            double d = (double) cvGetReal2D(distances, j, 0);
            if (d < reverseLookupDist[i]) {
                if (reverseLookupDist[i] == DBL_MAX)
                    matchCount++;
                reverseLookup[i] = j;
                reverseLookupDist[i] = d;
            }
        }
        printf(" found %d putative correspondences\n", matchCount);
        points1 = cvCreateMat(1, matchCount, CV_32FC2);
        points2 = cvCreateMat(1, matchCount, CV_32FC2);
        CvPoint2D32f *p1, *p2;
        int m = 0;
        for (int j = 0; j < kp2->total; j++) {
            int i = (int) cvGetReal2D(matches, j, 0);
            if (j == reverseLookup[i]) {
                p1 = &((CvSURFPoint*) cvGetSeqElem(kp1, i))->pt;
                p2 = &((CvSURFPoint*) cvGetSeqElem(kp2, j))->pt;
                points1->data.fl[m * 2] = p1->x;
                points1->data.fl[m * 2 + 1] = p1->y;
                points2->data.fl[m * 2] = p2->x;
                points2->data.fl[m * 2 + 1] = p2->y;
                m++;
            }
        }
        free(reverseLookup);
        free(reverseLookupDist);

        status = cvCreateMat(1, points1->cols, CV_8UC1);
        F = cvCreateMat(3, 3, CV_32FC1);
        cvSetZero(F);
        cout << "  Calculating fundamental matrix and removing outliers...";
        int fm_count = cvFindFundamentalMat(points1, points2, F, CV_FM_LMEDS, 1.0, 0.99, status);
        cvRemoveMatrixCols(&points1, status); // remove outliers from points1
        cvRemoveMatrixCols(&points2, status); // remove outliers from points2
        cout << " " << (points1 ? points1->cols : 0) << " correspondences remaining\n";
        if (fm_count <= 0) {
            printf("No fundamental matrix found.\n");
            return;
        } else {
            //            printf("Found %d fundamental matri%s\n"
            //                "\t%4.16f\t%4.16f\t%4.16f\n"
            //                "\t%4.16f\t%4.16f\t%4.16f\n"
            //                "\t%4.16f\t%4.16f\t%4.16f\n", fm_count, fm_count > 1 ? "ces. The best one is:" : "x:", F->data.fl[0], F->data.fl[1],
            //                    F->data.fl[2], F->data.fl[3], F->data.fl[4], F->data.fl[5], F->data.fl[6], F->data.fl[7], F->data.fl[8]);
            //printf("Found %d fundamental matri%s\n", fm_count, fm_count > 1 ? "ces" : "x");
        }
        CvMat *points1_64 = cvCreateMat(1, points1->cols, CV_64FC2);
        CvMat *points2_64 = cvCreateMat(1, points1->cols, CV_64FC2);
        cvCorrectMatches(F, points1, points2, points1_64, points2_64);

        cvReleaseMat(&points1);
        cvReleaseMat(&points2);

        *points1Return = points1_64;
        *points2Return = points2_64;
    }

    /***
     * <custom><ptamv>
     * Like original version, but use SURF to find matches between images.
     * This helps outside, where to get stereo separation camera has to be moved
     * much further than indoors
     */
    void Tracker::TrackForInitialMapUsingSURF(IplImage & imCVFrame) {
        // <custom> spacebar pressed externally
        if (!mbUserPressedSpacebar) {
            shm::shared_data* shdata = shm::shared_data::get_or_create();
            mbUserPressedSpacebar = shdata->nav.get_spacebar_pressed();
        }
        // What stage of initial tracking are we at?
        if (mnInitialStage == TRAIL_TRACKING_NOT_STARTED) {
            if (mbUserPressedSpacebar) // First spacebar = this is the first keyframe
            {
                mbUserPressedSpacebar = false;
                SURFTracking_Start(imCVFrame); // TODO: Possibly run SURF on first frame already here, for now just save the frame
                mFirstCVFrame = cvCloneImage(&imCVFrame);

                mnInitialStage = TRAIL_TRACKING_STARTED;
            } else
                mMessageForUser << "Point camera at planar scene and press spacebar to start tracking for initial map.";
            return;
        };

        if (mnInitialStage == TRAIL_TRACKING_STARTED) {
            // If the user pressed spacebar here, use trails to run stereo and make the intial map..
            if (mbUserPressedSpacebar) {
                mbUserPressedSpacebar = false;

                CvMat* points1Return;
                CvMat* points2Return;
                matchKeypoints(*mFirstCVFrame, imCVFrame, &points1Return, &points2Return);

                if (points1Return->cols < 10) {
                    Reset();
                    return;
                }

                vector<pair<ImageRef, ImageRef> > vMatches; // This is the format the mapmaker wants for the stereo pairs

                for (int i = 0; i < points1Return->cols; i++) {
                    float x1 = points1Return->data.db[i * 2];
                    float y1 = points1Return->data.db[i * 2 + 1];
                    float x2 = points2Return->data.db[i * 2];
                    float y2 = points2Return->data.db[i * 2 + 1];

                    vMatches.push_back(pair<ImageRef, ImageRef > (ImageRef(x1, y1), ImageRef(x2, y2)));
                }

                mMapMaker.InitFromStereo(mFirstKF, mCurrentKF, vMatches, mse3CamFromWorld); // This will take some time!
                mnInitialStage = TRAIL_TRACKING_COMPLETE;
            } else
                mMessageForUser << "Translate the camera slowly sideways, and press spacebar again to perform stereo init.";
        }
    }

    /***
     * <custom><ptamv>
     * The current frame is to be the first keyframe!
     */
    void Tracker::SURFTracking_Start(IplImage & imCVFrame) {
        mCurrentKF.MakeKeyFrame_Rest(); // This populates the Candidates list, which is Shi-Tomasi thresholded.
        mFirstKF = mCurrentKF;
        mPreviousFrameKF = mFirstKF; // Always store the previous frame so married-matching can work.
    }

    /*** <custom> equivalent of time.time() in python ***/
    double time() {
        struct timeval tv; // see gettimeofday(2)
        gettimeofday(&tv, NULL);
        return (double) tv.tv_sec + (double) 1e-6 * tv.tv_usec;
    }

    /*** <custom> modulus of floats***/
    double mod(double a, double b) {
        while (a > b)
            a -= b;
        while (a < 0)
            a += b;
        return a;
    }

    /*** <custom> publish tracked pos and map to shdata ***/
    void Tracker::PublishTrackedPosAndMap() {
        // initialization
        shm::shared_data* shdata = shm::shared_data::get_or_create();

        /************ publish map points ************/
        if (cameraNumber == 1) {
            static unsigned int previousSize = 0;
            vector<MapPoint*> points = mpMap->vpPoints;
            // require that size has changed ([heuristic] no size change suggests no change)
            if (points.size() != previousSize)
                PublishMapPoints();
            previousSize = points.size();
        }

        /************ publish position ************/
        // get slamdata
        shm::slam_data *slamdata = NULL;
        if (cameraNumber == 1)
            slamdata = &shdata->slam;
        else if (cameraNumber == 2)
            slamdata = &shdata->slam2;
        // get conversion matrix (convert from SLAM xyz to navdata xyz)
        Matrix < 3, 3 > C(Zeros);
        C[0][1] = 1;
        C[1][0] = -1;
        C[2][2] = 1;
        Matrix < 3, 3 > Cinv = C.T();
        // convert position
        Vector < 3 > xyzpos = mse3CamFromWorld.inverse().get_translation();
        Matrix < 3, 3 > R = mse3CamFromWorld.inverse().get_rotation().get_matrix(); // rotation matrix
        R = C * R * Cinv;
        /*** 3D position ***/
        slamdata->x = xyzpos[1] / mpMap->mdSLAMScaleToMetricScale;
        slamdata->y = -xyzpos[0] / mpMap->mdSLAMScaleToMetricScale;
        slamdata->h = xyzpos[2] / mpMap->mdSLAMScaleToMetricScale;
        /*** Euler angles ***/
        double theta = asin(R[2][0]);
        double psi = atan2(R[2][1], R[2][2]);
        double phi = atan2(-R[1][0], -R[0][0]);
        // correct differences
        slamdata->theta = theta * 180 / M_PI;
        slamdata->psi = -(-phi * 180 / M_PI);
        slamdata->phi = -(180 - psi * 180 / M_PI);
        // move back to range ]-180, 180[
        slamdata->phi = mod(slamdata->phi + 180, 360) - 180;
        /*** extra data ***/
        slamdata->confidence = getConfidence();
        slamdata->num_found_points = getNumFoundPoints();
        slamdata->timestamp = time();
        /*** rotation matrix ***/
        ostringstream rotmx;
        rotmx << GetCurrentPose().inverse();
        strcpy(slamdata->rotmx, rotmx.str().c_str());
    }

    /*** <custom> publish map to shdata ***/
    void Tracker::PublishMapPoints() {
        // init vars
        shm::shared_data* shdata = shm::shared_data::get_or_create();
        vector<MapPoint*> points = mpMap->vpPoints;
        // check camera number
        if (cameraNumber != 1)
            return;
        // check size
        int size = (int) points.size();
        if (size > SHM_SOCKET_MAX_NUM_POINTS) {
            cout << "Warning (Tracker.cc): num points (" << size << ") too large for shared memory" << endl;
            size = SHM_SOCKET_MAX_NUM_POINTS;
        }
        for (int i = 0; i < size; i++) {
            shdata->points.p[i].x = points[i]->v3WorldPos[1] / mpMap->mdSLAMScaleToMetricScale;
            shdata->points.p[i].y = -points[i]->v3WorldPos[0] / mpMap->mdSLAMScaleToMetricScale;
            shdata->points.p[i].h = points[i]->v3WorldPos[2] / mpMap->mdSLAMScaleToMetricScale;
        }
        shdata->points.size = points.size();
        shdata->points.timestamp = time();
    }

    /*** <custom> get confidence by map quality ***/
    double Tracker::getConfidence() {
        return (double) getNumFoundPoints() / getNumAttemptedPoints();
    }

    /*** <custom> number of points that were found in this frame ***/
    int Tracker::getNumFoundPoints() {
        int nTotalFound = 0;
        for (int i = 0; i < LEVELS; i++)
            nTotalFound += manMeasFound[i];
        return nTotalFound;
    }

    /*** <custom> number of points that were attempted in this frame ***/
    int Tracker::getNumAttemptedPoints() {
        int nTotalAttempted = 0;
        for (int i = 0; i < LEVELS; i++)
            nTotalAttempted += manMeasAttempted[i];
        return nTotalAttempted;
    }

    /*** <custom><wallvizmap> ***/
    void Tracker::saveWallvizFrame() {
        static long mappoint_id = 0;
        static ostringstream mapxmls[20];
        static int keyframe_id = -1;
        keyframe_id++;

        /*** get map id and derivatives **/
        int mapid = mpMap->MapID();

        /*** get base folder **/
        char buf[13];
        sprintf(buf, "wallvizmap%d", mapid);
        string folder = string(buf);
        mkdir(buf, S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
        mkdir((folder + "/KeyFrames").c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);

        /*** save SLAMScaleToMetricScale **/
        ofstream f;
        f.open((folder + "/slamtometric").c_str());
        f << mpMap->mdSLAMScaleToMetricScale << endl << "# mm^-1";
        f.close();

        /*** save image as png **/
        //ostringstream path;
        //path << folder << "/KeyFrames/" << keyframe_id << ".png";
        //try {
        //    img_save(mCurrentKF.aLevels[0].im, path.str());
        //} catch (CVD::Exceptions::All err) {
        //    cerr << " Failed to save image " << path.str() << ": " << err.what << endl;
        //}

        /*** save color image as png **/
        ostringstream path2;
        path2 << folder << "/KeyFrames/" << keyframe_id << ".png";
        try {
            img_save(*colorFrame, path2.str());
        } catch (CVD::Exceptions::All err) {
            cerr << " Failed to save image " << path2.str() << ": " << err.what << endl;
        }

        /*** save map.xml **/
        f.open((folder + "/map.xml").c_str());
        f << "<map>" << endl;
        mapxmls[mapid] << "  <KeyFrame id=\"" << keyframe_id << "\" pose=\"" << SE3<>::ln(mse3CamFromWorld) << "\">" << endl;
        mapxmls[mapid] << "      <Image file=\"KeyFrames/" << keyframe_id << ".png\" />" << endl;
        mapxmls[mapid] << "  </KeyFrame>" << endl;
        int size = mpMap->vpPoints.size();
        for (unsigned int i = 0; i < size; i++) {
            MapPoint &p = *(mpMap->vpPoints[i]);
            if (!p.pTData) p.pTData = new TrackerData(&p, mirSize);
            p.pTData->Project(mse3CamFromWorld, mCamera);
            if (!p.pTData->bInImage)
                continue;
            int x = p.pTData->v2Image[0];
            int y = p.pTData->v2Image[1];
            mapxmls[mapid] << "  <MapPoint id=\"" << mappoint_id << "\" position=\"" << p.v3WorldPos << "\">" << endl;
            mapxmls[mapid] << "      <SourceKF id=\"" << keyframe_id << "\"  level=\"0\" x=\"" << x << "\" y=\"" << y << "\" />" << endl;
            mapxmls[mapid] << "  </MapPoint>" << endl;
            mappoint_id++;
        }
        f << mapxmls[mapid].str();
        f << "</map>" << endl;
        f.close();

        /*** debug output **/
        cout << "  frame saved" << endl;
    }

    void Tracker::setSLAMScaleToMetricScale() {
        shm::shared_data* shdata = shm::shared_data::get_or_create();
        double measuredAltitude = shdata->nav.altitude;
        if (measuredAltitude < 0.1)
            measuredAltitude = 200; // mm
        double h = mse3CamFromWorld.inverse().get_translation()[2];
        mpMap->mdSLAMScaleToMetricScale = h / measuredAltitude;
        cout << "  [MEASURED ALTITUDE = " << measuredAltitude << " mm]" << endl;
        cout << "  [SLAM TO METRIC = " << mpMap->mdSLAMScaleToMetricScale << " mm^-1]" << endl;
    }
}
