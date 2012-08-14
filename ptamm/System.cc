// Copyright 2009 Isis Innovation Limited
#include "System.h"
#include "OpenGL.h"
#include <gvars3/GStringUtil.h>
#include <stdlib.h>
#include "ATANCamera.h"
#include "MapMaker.h"
#include "Tracker.h"
#include "ARDriver.h"
#include "MapViewer.h"
#include "MapSerializer.h"
#include "Games.h"

#ifdef _LINUX
#include <fcntl.h>
#endif

#ifdef WIN32
#include <Windows.h>
#endif

// <custom>
#include "CustomSettings.h"

namespace PTAMM {

    using namespace CVD;
    using namespace std;
    using namespace GVars3;

    System::System() : mGLWindow(mVideoSource.Size(), "PTAMM") {
        //mGLWindow.set_size(mVideoSource.Size()*2);
        mGLWindow.set_size(mVideoSource.Size());
        mGLWindow2 = NULL;

        GUI.RegisterCommand("exit", GUICommandCallBack, this);
        GUI.RegisterCommand("quit", GUICommandCallBack, this);

        //PTAMM commands
        GUI.RegisterCommand("SwitchMap", GUICommandCallBack, this);
        GUI.RegisterCommand("NewMap", GUICommandCallBack, this);
        GUI.RegisterCommand("DeleteMap", GUICommandCallBack, this);
        GUI.RegisterCommand("ResetAll", GUICommandCallBack, this);

        GUI.RegisterCommand("LoadMap", GUICommandCallBack, this);
        GUI.RegisterCommand("SaveMap", GUICommandCallBack, this);
        GUI.RegisterCommand("SaveMaps", GUICommandCallBack, this);

        GV2.Register(mgvnLockMap, "LockMap", 0, SILENT);
        GV2.Register(mgvnDrawMapInfo, "MapInfo", 0, SILENT);

#ifdef _LINUX
        GV2.Register(mgvnSaveFIFO, "SaveFIFO", 0, SILENT);
        GV2.Register(mgvnBitrate, "Bitrate", 15000, SILENT);
#endif  

        GUI.RegisterCommand("KeyPress", GUICommandCallBack, this);


        mimFrameBW.resize(mVideoSource.Size());
        if (SECONDCAMERA)
            mimFrameBW2.resize(mVideoSource.Size2());
        mimFrameRGB.resize(mVideoSource.Size());
        // First, check if the camera is calibrated.
        // If not, we need to run the calibration widget.
        Vector<NUMTRACKERCAMPARAMETERS> vTest;

        vTest = GV3::get<Vector<NUMTRACKERCAMPARAMETERS> >("Camera.Parameters", ATANCamera::mvDefaultParams, HIDDEN);
        mpCamera = new ATANCamera("Camera");
        mpCamera->SetImageSize(mVideoSource.Size());
        if (SECONDCAMERA) {
            mpCamera2 = new ATANCamera("Camera2");
            mpCamera2->SetImageSize(mVideoSource.Size2());
        }

        if (vTest == ATANCamera::mvDefaultParams) {
            cout << endl;
            cout << "! Camera.Parameters is not set, need to run the CameraCalibrator tool" << endl;
            cout << "  and/or put the Camera.Parameters= line into the appropriate .cfg file." << endl;
            exit(1);
        }


        //create the first map
        mpMap = new Map();
        mvpMaps.push_back(mpMap);
        mpMap->mapLockManager.Register(this);
        mpMapMaker = new MapMaker(mvpMaps, mpMap);
        mpTracker = new Tracker(mVideoSource.Size(), *mpCamera, mvpMaps, mpMap, *mpMapMaker, &mimFrameRGB, 1);
        if (SECONDCAMERA) {
            mpMap2 = new Map();
            mvpMaps2.push_back(mpMap2);
            mpMap2->mapLockManager.Register(this);
            mpMapMaker2 = new MapMaker(mvpMaps2, mpMap2);
            mpTracker2 = new Tracker(mVideoSource.Size2(), *mpCamera2, mvpMaps2, mpMap2, *mpMapMaker2, &mimFrameRGB, 2);
        }
        mpARDriver = new ARDriver(*mpCamera, mVideoSource.Size(), mGLWindow, *mpMap);
        mpMapViewer = new MapViewer(mvpMaps, mpMap, mGLWindow);
        mpMapSerializer = new MapSerializer(mvpMaps);

        //These commands have to be registered here as they call the classes created above
        GUI.RegisterCommand("NextMap", GUICommandCallBack, mpMapViewer);
        GUI.RegisterCommand("PrevMap", GUICommandCallBack, mpMapViewer);
        GUI.RegisterCommand("CurrentMap", GUICommandCallBack, mpMapViewer);

        GUI.RegisterCommand("LoadGame", GUICommandCallBack, mpARDriver);
        GUI.RegisterCommand("Mouse.Click", GUICommandCallBack, mpARDriver);

        //create the menus
        GUI.ParseLine("GLWindow.AddMenu Menu Menu");
        GUI.ParseLine("Menu.ShowMenu Root");
        GUI.ParseLine("Menu.AddMenuButton Root \"Reset All\" ResetAll Root");
        GUI.ParseLine("Menu.AddMenuButton Root Reset Reset Root");
        GUI.ParseLine("Menu.AddMenuButton Root Spacebar PokeTracker Root");
        GUI.ParseLine("Menu.AddMenuButton Root Demos \"\" Demos");
        GUI.ParseLine("DrawAR=0");
        GUI.ParseLine("DrawMap=0");
        GUI.ParseLine("Menu.AddMenuToggle Root \"Draw AR\" DrawAR Root");

        //Games. This function can be found in Games.cc. Add your games to it.
        InitializeGameMenu();

        GUI.ParseLine("GLWindow.AddMenu MapsMenu Maps");
        GUI.ParseLine("MapsMenu.AddMenuButton Root \"New Map\" NewMap Root");
        GUI.ParseLine("MapsMenu.AddMenuButton Root \"Serialize\" \"\" Serial");
        GUI.ParseLine("MapsMenu.AddMenuButton Serial \"Save Maps\" SaveMaps Root");
        GUI.ParseLine("MapsMenu.AddMenuButton Serial \"Save Map\" SaveMap Root");
        GUI.ParseLine("MapsMenu.AddMenuButton Serial \"Load Map\" LoadMap Root");
#ifdef _LINUX
        GUI.ParseLine("MapsMenu.AddMenuToggle Serial \"Save Video\" SaveFIFO Serial");
        GUI.ParseLine("MapsMenu.AddMenuSlider Serial Bitrate Bitrate 100 20000 Serial");
#endif
        GUI.ParseLine("LockMap=0");
        GUI.ParseLine("MapsMenu.AddMenuToggle Root \"Lock Map\" LockMap Root");
        GUI.ParseLine("MapsMenu.AddMenuButton Root \"Delete Map\" DeleteMap Root");
        GUI.ParseLine("MapInfo=0");
        GUI.ParseLine("MapsMenu.AddMenuToggle Root \"Map Info\" MapInfo Root");

        GUI.ParseLine("GLWindow.AddMenu MapViewerMenu Viewer");
        GUI.ParseLine("MapViewerMenu.AddMenuToggle Root \"View Map\" DrawMap Root");
        GUI.ParseLine("MapViewerMenu.AddMenuButton Root Next NextMap Root");
        GUI.ParseLine("MapViewerMenu.AddMenuButton Root Previous PrevMap Root");
        GUI.ParseLine("MapViewerMenu.AddMenuButton Root Current CurrentMap Root");


        mbDone = false;
    }

    /**
     * Destructor
     */
    System::~System() {
        if (mpMap != NULL) {
            mpMap->mapLockManager.UnRegister(this);
        }

    }

    /**
     * Run the main system thread.
     * This handles the tracker and the map viewer.
     */
    void System::Run() {
        /******* <createdataset><tmp><secondcamera> *********/
        const char* fname = "dataset_basistf_2cameras.txt";
        ofstream myfile;
        myfile.open(fname);
        myfile << "$$" << endl;
        /******* </secondcamera></tmp></createdataset> *********/
        while (!mbDone) {
            mGLWindow.make_current();

            //Check if the map has been locked by another thread, and wait for release.
            bool bWasLocked = mpMap->mapLockManager.CheckLockAndWait(this, 0);

            /* This is a rather hacky way of getting this feedback,
               but GVars cannot be assigned to different variables
               and each map has its own edit lock bool.
               A button could be used instead, but the visual
               feedback would not be as obvious.
             */
            mpMap->bEditLocked = *mgvnLockMap; //sync up the maps edit lock with the gvar bool.

            // We use two versions of each video frame:
            // One black and white (for processing by the tracker etc)
            // and one RGB, for drawing.

            // Grab new video frame...
            mVideoSource.GetAndFillFrameBWandRGB(mimFrameBW, mimFrameRGB);
            if (SECONDCAMERA)
                mVideoSource.GetAndFillFrameBW2(mimFrameBW2);
            static bool bFirstFrame = true;
            if (bFirstFrame) {
                mpARDriver->Init();
                bFirstFrame = false;
            }
            mGLWindow.SetupViewport();
            mGLWindow.SetupVideoOrtho();
            mGLWindow.SetupVideoRasterPosAndZoom();

            if (!mpMap->IsGood()) {
                mpARDriver->Reset();
            }

            if (bWasLocked) {
                mpTracker->ForceRecovery();
            }

            static gvar3<int> gvnDrawMap("DrawMap", 0, HIDDEN | SILENT);
            static gvar3<int> gvnDrawAR("DrawAR", 0, HIDDEN | SILENT);

            bool bDrawMap = mpMap->IsGood() && *gvnDrawMap;
            bool bDrawAR = mpMap->IsGood() && *gvnDrawAR;

            mpTracker->TrackFrame(mimFrameBW, !bDrawAR && !bDrawMap);
            mpTracker->writePosToFile(myfile); //<createdataset><tmp><secondcamera>

            if (bDrawMap) {
                mpMapViewer->DrawMap(mpTracker->GetCurrentPose());
            } else if (bDrawAR) {
                if (!mpTracker->IsLost()) {
                    mpARDriver->AdvanceLogic();
                }
                mpARDriver->Render(mimFrameRGB, mpTracker->GetCurrentPose(), mpTracker->IsLost());
            }

            if (*mgvnDrawMapInfo) {
                DrawMapInfo();
            }

            string sCaption;
            if (bDrawMap) {
                sCaption = mpMapViewer->GetMessageForUser();
            } else {
                sCaption = mpTracker->GetMessageForUser();
            }
            mGLWindow.DrawCaption(sCaption);
            mGLWindow.DrawMenus();

#ifdef _LINUX
            // note: not important (default off)
            if (*mgvnSaveFIFO) {
                SaveFIFO();
            }
#endif

            mGLWindow.swap_buffers();
            mGLWindow.HandlePendingEvents();

            /*** do second camera ***/
            if (SECONDCAMERA) {
                if (mGLWindow2 == NULL) {
                    mGLWindow2 = new GLWindow2(mVideoSource.Size2(), "PTAMM camera 2");
                    mGLWindow2->set_size(mVideoSource.Size2()*2);
                }
                mGLWindow2->make_current();
                mGLWindow2->SetupViewport();
                mGLWindow2->SetupVideoOrtho();
                mGLWindow2->SetupVideoRasterPosAndZoom();
                mpTracker2->TrackFrame(mimFrameBW2, true);
                mpTracker2->writePosToFile(myfile); //<createdataset><tmp><secondcamera>
                // get caption
                string sCaption2 = mpTracker2->GetMessageForUser();
                mGLWindow2->DrawCaption(sCaption2);
                mGLWindow2->swap_buffers();
                mGLWindow2->HandlePendingEvents();
            }
        }
    }

    /**
     * Parse commands sent via the GVars command system.
     * @param ptr Object callback
     * @param sCommand command string
     * @param sParams parameters
     */
    void System::GUICommandCallBack(void *ptr, string sCommand, string sParams) {
        if (sCommand == "quit" || sCommand == "exit") {
            static_cast<System*> (ptr)->mbDone = true;
        } else if (sCommand == "SwitchMap") {
            int nMapNum = -1;
            if (static_cast<System*> (ptr)->GetSingleParam(nMapNum, sCommand, sParams)) {
                static_cast<System*> (ptr)->SwitchMap(nMapNum);
            }
        } else if (sCommand == "ResetAll") {
            static_cast<System*> (ptr)->ResetAll();
            return;
        } else if (sCommand == "NewMap") {
            cout << "Making new map..." << endl;
            static_cast<System*> (ptr)->NewMap();
        } else if (sCommand == "DeleteMap") {
            int nMapNum = -1;
            if (sParams.empty()) {
                static_cast<System*> (ptr)->DeleteMap(static_cast<System*> (ptr)->mpMap->MapID());
            } else if (static_cast<System*> (ptr)->GetSingleParam(nMapNum, sCommand, sParams)) {
                static_cast<System*> (ptr)->DeleteMap(nMapNum);
            }
        } else if (sCommand == "NextMap") {
            static_cast<MapViewer*> (ptr)->ViewNextMap();
        } else if (sCommand == "PrevMap") {
            static_cast<MapViewer*> (ptr)->ViewPrevMap();
        } else if (sCommand == "CurrentMap") {
            static_cast<MapViewer*> (ptr)->ViewCurrentMap();
        } else if (sCommand == "SaveMap" || sCommand == "SaveMaps" || sCommand == "LoadMap") {
            static_cast<System*> (ptr)->StartMapSerialization(sCommand, sParams);
        } else if (sCommand == "LoadGame") {
            static_cast<ARDriver*> (ptr)->LoadGame(sParams);
        } else if (sCommand == "Mouse.Click") {
            vector<string> vs = ChopAndUnquoteString(sParams);

            if (vs.size() != 3) {
                return;
            }

            istringstream is(sParams);
            int nButton;
            ImageRef irWin;
            is >> nButton >> irWin.x >> irWin.y;
            static_cast<ARDriver*> (ptr)->HandleClick(nButton, irWin);

        } else if (sCommand == "KeyPress") {
            if (sParams == "q" || sParams == "Escape") {
                GUI.ParseLine("quit");
                return;
            }

            bool bUsed = static_cast<System*> (ptr)->mpTracker->HandleKeyPress(sParams);
            if (SECONDCAMERA)
                static_cast<System*> (ptr)->mpTracker2->HandleKeyPress(sParams);

            if (!bUsed) {
                static_cast<System*> (ptr)->mpARDriver->HandleKeyPress(sParams);
            }
        }


    }

    /**
     * Parse and allocate a single integer variable from a string parameter
     * @param nAnswer the result
     * @param sCommand the command (used to display usage info)
     * @param sParams  the parameters to parse
     * @return success or failure.
     */
    bool System::GetSingleParam(int &nAnswer, string sCommand, string sParams) {
        vector<string> vs = ChopAndUnquoteString(sParams);

        if (vs.size() == 1) {
            //is param a number?
            bool bIsNum = true;
            for (size_t i = 0; i < vs[0].size(); i++) {
                bIsNum = isdigit(vs[0][i]) && bIsNum;
            }

            if (!bIsNum) {
                return false;
            }

            int *pN = ParseAndAllocate<int>(vs[0]);
            if (pN) {
                nAnswer = *pN;
                delete pN;
                return true;
            }
        }

        cout << sCommand << " usage: " << sCommand << " value" << endl;

        return false;
    }

    /**
     * Switch to the map with ID nMapNum
     * @param  nMapNum Map ID
     * @param bForce This is only used by DeleteMap and ResetAll, and is
     * to ensure that MapViewer is looking at a safe map.
     */
    bool System::SwitchMap(int nMapNum, bool bForce) {

        //same map, do nothing. This should not actually occur
        if (mpMap->MapID() == nMapNum) {
            return true;
        }

        if ((nMapNum < 0)) {
            cerr << "Invalid map number: " << nMapNum << ". Not changing." << endl;
            return false;
        }


        for (size_t ii = 0; ii < mvpMaps.size(); ii++) {
            Map * pcMap = mvpMaps[ ii ];
            if (pcMap->MapID() == nMapNum) {
                mpMap->mapLockManager.UnRegister(this);
                mpMap = pcMap;
                mpMap->mapLockManager.Register(this);
            }
        }

        if (mpMap->MapID() != nMapNum) {
            cerr << "Failed to switch to " << nMapNum << ". Does not exist." << endl;
            return false;
        }

        /*  Map was found and switched to for system.
            Now update the rest of the system.
            Order is important. Do not want keyframes added or
            points deleted from the wrong map.
  
            MapMaker is in its own thread.
            System,Tracker, and MapViewer are all in this thread.
         */

        *mgvnLockMap = mpMap->bEditLocked;


        //update the map maker thread
        if (!mpMapMaker->RequestSwitch(mpMap)) {
            return false;
        }

        while (!mpMapMaker->SwitchDone()) {
#ifdef WIN32
            Sleep(1);
#else
            usleep(10);
#endif
        }

        //update the map viewer object
        mpMapViewer->SwitchMap(mpMap, bForce);

        //update the tracker object
        //   mpARDriver->Reset();
        mpARDriver->SetCurrentMap(*mpMap);

        if (!mpTracker->SwitchMap(mpMap)) {
            return false;
        }

        return true;
    }

    /**
     * Create a new map and switch all
     * threads and objects to it.
     */
    void System::NewMap() {

        *mgvnLockMap = false;
        mpMap->mapLockManager.UnRegister(this);
        mpMap = new Map();
        mpMap->mapLockManager.Register(this);
        mvpMaps.push_back(mpMap);

        //update the map maker thread
        mpMapMaker->RequestReInit(mpMap);
        while (!mpMapMaker->ReInitDone()) {
#ifdef WIN32
            Sleep(1);
#else
            usleep(10);
#endif
        }

        //update the map viewer object
        mpMapViewer->SwitchMap(mpMap);

        //update the tracker object
        mpARDriver->SetCurrentMap(*mpMap);
        mpARDriver->Reset();
        mpTracker->SetNewMap(mpMap);

        cout << "New map created (" << mpMap->MapID() << ")" << endl;

    }

    /**
     * Moves all objects and threads to the first map, and resets it.
     * Then deletes the rest of the maps, placing PTAMM in its
     * original state.
     * This reset ignores the edit lock status on all maps
     */
    void System::ResetAll() {

        //move all maps to first map.
        if (mpMap != mvpMaps.front()) {
            if (!SwitchMap(mvpMaps.front()->MapID(), true)) {
                cerr << "Reset All: Failed to switch to first map" << endl;
            }
        }
        mpMap->bEditLocked = false;

        //reset map.
        mpTracker->Reset();

        //lock and delete all remaining maps
        while (mvpMaps.size() > 1) {
            DeleteMap(mvpMaps.back()->MapID());
        }

    }

    /**
     * Delete a specified map.
     * @param nMapNum map to delete
     */
    bool System::DeleteMap(int nMapNum) {
        if (mvpMaps.size() <= 1) {
            cout << "Cannot delete the only map. Use Reset instead." << endl;
            return false;
        }


        //if the specified map is the current map, move threads to another map
        if (nMapNum == mpMap->MapID()) {
            int nNewMap = -1;

            if (mpMap == mvpMaps.front()) {
                nNewMap = mvpMaps.back()->MapID();
            } else {
                nNewMap = mvpMaps.front()->MapID();
            }

            // move the current map users elsewhere
            if (!SwitchMap(nNewMap, true)) {
                cerr << "Delete Map: Failed to move threads to another map." << endl;
                return false;
            }
        }



        // find and delete the map
        for (size_t ii = 0; ii < mvpMaps.size(); ii++) {
            Map * pDelMap = mvpMaps[ ii ];
            if (pDelMap->MapID() == nMapNum) {

                pDelMap->mapLockManager.Register(this);
                pDelMap->mapLockManager.LockMap(this);
                delete pDelMap;
                mvpMaps.erase(mvpMaps.begin() + ii);

                ///@TODO Possible bug. If another thread (eg serialization) was using this
                /// and waiting for unlock, would become stuck or seg fault.
            }
        }

        return true;
    }

    /**
     * Set up the map serialization thread for saving/loading and the start the thread
     * @param sCommand the function that was called (eg. SaveMap)
     * @param sParams the params string, which may contain a filename and/or a map number
     */
    void System::StartMapSerialization(std::string sCommand, std::string sParams) {
        if (mpMapSerializer->Init(sCommand, sParams, *mpMap)) {
            mpMapSerializer->start();
        }
    }

    /**
     * Draw a box with information about the maps.
     */
    void System::DrawMapInfo() {
        int nLines = static_cast<int> (mvpMaps.size()) + 2;
        int x = 5, y = 120, w = 160, nBorder = 5;

        mGLWindow.DrawBox(x, y, w, nLines, 0.7f);

        y += 17;

        glColor3f(1, 1, 1);
        std::ostringstream os;
        os << "Maps " << mvpMaps.size();
        mGLWindow.PrintString(ImageRef(x + nBorder, y + nBorder), os.str());
        os.str("");

        for (size_t i = 0; i < mvpMaps.size(); i++) {
            Map * pMap = mvpMaps[i];
            if (pMap == mpMap) {
                glColor3f(1, 1, 0);
            } else if (pMap->bEditLocked) {
                glColor3f(1, 0, 0);
            } else {
                glColor3f(1, 1, 1);
            }

            os << "M: " << pMap->MapID() << "  P: " << pMap->vpPoints.size() << "  K: " << pMap->vpKeyFrames.size();
            mGLWindow.PrintString(ImageRef(x + nBorder, y + nBorder + (i + 1)*17), os.str());
            os.str("");
        }
    }

    /**
     * Save the current frame to a FIFO.
     * This function is called on each frame to create a video.
     * The GVar SaveFIFO starts and stops the saving, and the GVar
     * Bitrate sets the quality.
     * Bitrate can only be set before the first call of SaveFIFO.
     */
    void System::SaveFIFO() {
#ifdef _LINUX
        //Some static variables
        static CVD::byte* pcImage = NULL;
        static int fd = 0;
        static bool bFIFOInitDone = false;
        static ImageRef irWindowSize;

        if (!bFIFOInitDone) {
            irWindowSize = mGLWindow.size();

            ostringstream os;
            os << /*"/bin/bash\n" <<*/
                    "file=\"`date '+%Y-%m-%d_%H-%M-%S'`.avi\"; " <<
                    "if [ ! -e FIFO ]; then mkfifo FIFO; echo Made FIFO...; fi; " <<
                    "echo Mencoding to $file....; " <<
                    "cat FIFO |nice mencoder -flip -demuxer rawvideo -rawvideo fps=30:w=" <<
                    irWindowSize.x << ":h=" << irWindowSize.y <<
                    ":format=rgb24 -o $file -ovc lavc -lavcopts vcodec=mpeg4:vbitrate=" << *mgvnBitrate <<
                    ":keyint=45 -ofps 30 -ffourcc DIVX - &";

            cout << "::" << os.str() << "::" << endl;
            int i = system(os.str().c_str());
            if (i != 0) {
                cerr << "ERROR: could not set up the FIFO!" << endl;
                return;
            }

            posix_memalign((void**) (&pcImage), 16, irWindowSize.x * irWindowSize.y * 3);
            string s = "FIFO";
            fd = open(s.c_str(), O_RDWR | O_ASYNC);

            bFIFOInitDone = true;
        }

        if (irWindowSize != mGLWindow.size()) {
            cerr << "ERROR: Aborting FIFO as window size has changed!!" << endl;
            *mgvnSaveFIFO = 0;
            return;
        }

        glReadBuffer(GL_BACK);
        glReadPixels(0, 0, irWindowSize.x, irWindowSize.y, GL_RGB, GL_UNSIGNED_BYTE, pcImage);
        write(fd, (char*) pcImage, irWindowSize.x * irWindowSize.y * 3);

#else
        cout << "Video Saving using FIFOs is only available under Linux" << endl;
#endif

    }


}

