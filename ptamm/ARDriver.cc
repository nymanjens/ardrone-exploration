// Copyright 2009 Isis Innovation Limited
#define GL_GLEXT_PROTOTYPES 1
#include "ARDriver.h"
#include "Map.h"
#include "Games.h"

#include <cvd/image_io.h>

namespace PTAMM {

using namespace GVars3;
using namespace CVD;
using namespace std;

static bool CheckFramebufferStatus();

/**
 * Constructor
 * @param cam Reference to the camera
 * @param irFrameSize the size of the frame
 * @param glw the glwindow
 * @param map the current map
 */
ARDriver::ARDriver(const ATANCamera &cam, ImageRef irFrameSize, GLWindow2 &glw, Map &map)
  :mCamera(cam), mGLWindow(glw), mpMap( &map )
{
  mirFrameSize = irFrameSize;
  mCamera.SetImageSize(mirFrameSize);
  mbInitialised = false;
  
}


/**
 * Initialize the AR driver
 */
void ARDriver::Init()
{
  mbInitialised = true;
  mirFBSize = GV3::get<ImageRef>("ARDriver.FrameBufferSize", ImageRef(1200,900), SILENT);
  glGenTextures(1, &mnFrameTex);
  glBindTexture(GL_TEXTURE_RECTANGLE_ARB,mnFrameTex);
  glTexImage2D(GL_TEXTURE_RECTANGLE_ARB, 0,
	       GL_RGBA, mirFrameSize.x, mirFrameSize.y, 0,
	       GL_RGBA, GL_UNSIGNED_BYTE, NULL);
  MakeFrameBuffer();

  try {
    CVD::img_load(mLostOverlay, "ARData/Overlays/searching.png");
  }
  catch(CVD::Exceptions::All err) {
    cerr << "Failed to load searching image " << "\"ARData/Overlays/searching.png\"" << ": " << err.what << endl;
  }  
  
}


/**
 * Reset the game and the frame counter
 */
void ARDriver::Reset()
{
  if(mpMap->pGame) {
    mpMap->pGame->Reset();
  }

  mnCounter = 0;
}


/**
 * Render the AR composite image
 * @param imFrame The camera frame
 * @param se3CfromW The camera position
 * @param bLost Is the camera lost
 */
void ARDriver::Render(Image<Rgb<CVD::byte> > &imFrame, SE3<> se3CfromW, bool bLost)
{
  if(!mbInitialised)
  {
    Init();
    Reset();
  };

  mse3CfromW = se3CfromW;
  mnCounter++;

  // Upload the image to our frame texture
  glBindTexture(GL_TEXTURE_RECTANGLE_ARB, mnFrameTex);
  glTexSubImage2D(GL_TEXTURE_RECTANGLE_ARB,
		  0, 0, 0,
		  mirFrameSize.x, mirFrameSize.y,
		  GL_RGB,
		  GL_UNSIGNED_BYTE,
		  imFrame.data());

  // Set up rendering to go the FBO, draw undistorted video frame into BG
  glBindFramebufferEXT(GL_FRAMEBUFFER_EXT,mnFrameBuffer);
  CheckFramebufferStatus();
  glViewport(0,0,mirFBSize.x,mirFBSize.y);
  DrawFBBackGround();
  glClearDepth(1);
  glClear(GL_DEPTH_BUFFER_BIT);

  // Set up 3D projection
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();

  //only draw 3d stuff if not lost.
  if(!bLost)
  {
    glMultMatrix(mCamera.MakeUFBLinearFrustumMatrix(0.005, 100));
    glMultMatrix(se3CfromW);

    DrawFadingGrid();

    if(mpMap->pGame) {
      mpMap->pGame->Draw3D( mGLWindow, *mpMap, se3CfromW);
    }
  }

  glDisable(GL_DEPTH_TEST);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glDisable(GL_BLEND);

  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();

  // Set up for drawing 2D stuff:
  glBindFramebufferEXT(GL_FRAMEBUFFER_EXT,0);

  DrawDistortedFB();
  
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  mGLWindow.SetupViewport();
  mGLWindow.SetupVideoOrtho();
  mGLWindow.SetupVideoRasterPosAndZoom();

  
  //2d drawing
  if(!bLost)
  {   
    if(mpMap->pGame) {
      mpMap->pGame->Draw2D(mGLWindow, *mpMap);
    }
  }
  else
  {
    //draw the lost ar overlays
    glEnable(GL_BLEND);
    glRasterPos2i( ( mGLWindow.size().x - mLostOverlay.size().x )/2,
                   ( mGLWindow.size().y - mLostOverlay.size().y )/2 );
    glDrawPixels(mLostOverlay);
    glDisable(GL_BLEND);
  }

}


/**
 * Make the frame buffer
 */
void ARDriver::MakeFrameBuffer()
{
  // Needs nvidia drivers >= 97.46
  cout << "  ARDriver: Creating FBO... ";

  glGenTextures(1, &mnFrameBufferTex);
  glBindTexture(GL_TEXTURE_RECTANGLE_ARB,mnFrameBufferTex);
  glTexImage2D(GL_TEXTURE_RECTANGLE_ARB, 0,
	       GL_RGBA, mirFBSize.x, mirFBSize.y, 0,
	       GL_RGBA, GL_UNSIGNED_BYTE, NULL);
  glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_MAG_FILTER, GL_NEAREST);

  GLuint DepthBuffer;
  glGenRenderbuffersEXT(1, &DepthBuffer);
  glBindRenderbufferEXT(GL_RENDERBUFFER_EXT, DepthBuffer);
  glRenderbufferStorageEXT(GL_RENDERBUFFER_EXT, GL_DEPTH_COMPONENT24, mirFBSize.x, mirFBSize.y);

  glGenFramebuffersEXT(1, &mnFrameBuffer);
  glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, mnFrameBuffer);
  glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT,
			    GL_TEXTURE_RECTANGLE_ARB, mnFrameBufferTex, 0);
  glFramebufferRenderbufferEXT(GL_FRAMEBUFFER_EXT, GL_DEPTH_ATTACHMENT_EXT,
  			       GL_RENDERBUFFER_EXT, DepthBuffer);

  CheckFramebufferStatus();
  cout << " .. created FBO." << endl;
  glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, 0);
}


/**
 * check the status of the frame buffer
 */
static bool CheckFramebufferStatus()
{
  GLenum n;
  n = glCheckFramebufferStatusEXT(GL_FRAMEBUFFER_EXT);
  if(n == GL_FRAMEBUFFER_COMPLETE_EXT)
    return true; // All good
  
  cout << "glCheckFrameBufferStatusExt returned an error." << endl;
  return false;
}


/**
 * Draw the background (the image from the camera)
 */
void ARDriver::DrawFBBackGround()
{
  static bool bFirstRun = true;
  static GLuint nList;
  mGLWindow.SetupUnitOrtho();

  glEnable(GL_TEXTURE_RECTANGLE_ARB);
  glBindTexture(GL_TEXTURE_RECTANGLE_ARB, mnFrameTex);
  glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glDisable(GL_POLYGON_SMOOTH);
  glDisable(GL_BLEND);
  // Cache the cpu-intesive projections in a display list..
  if(bFirstRun)
    {
      bFirstRun = false;
      nList = glGenLists(1);
      glNewList(nList, GL_COMPILE_AND_EXECUTE);
      glColor3f(1,1,1);
      // How many grid divisions in the x and y directions to use?
      int nStepsX = 24; // Pretty arbitrary..
      int nStepsY = (int) (nStepsX * ((double) mirFrameSize.x / mirFrameSize.y)); // Scaled by aspect ratio
      if(nStepsY < 2)
	nStepsY = 2;
      for(int ystep = 0; ystep< nStepsY; ystep++)
	{
	  glBegin(GL_QUAD_STRIP);
	  for(int xstep = 0; xstep <= nStepsX; xstep++)
	    for(int yystep = ystep; yystep<=ystep+1; yystep++) // Two y-coords in one go - magic.
	      {
		Vector<2> v2Iter;
		v2Iter[0] = (double) xstep / nStepsX;
		v2Iter[1] = (double) yystep / nStepsY;
		// If this is a border quad, draw a little beyond the
		// outside of the frame, this avoids strange jaggies
		// at the edge of the reconstructed frame later:
		if(xstep == 0 || yystep == 0 || xstep == nStepsX || yystep == nStepsY)
		  for(int i=0; i<2; i++)
		    v2Iter[i] = v2Iter[i] * 1.02 - 0.01;
		Vector<2> v2UFBDistorted = v2Iter;
		Vector<2> v2UFBUnDistorted = mCamera.UFBLinearProject(mCamera.UFBUnProject(v2UFBDistorted));
		glTexCoord2d(v2UFBDistorted[0] * mirFrameSize.x, v2UFBDistorted[1] * mirFrameSize.y);
		glVertex(v2UFBUnDistorted);
	      }
	  glEnd();
	}
      glEndList();
    }
  else
    glCallList(nList);
  glDisable(GL_TEXTURE_RECTANGLE_ARB);
}


/**
 * Draw the distorted frame buffer
 */
void ARDriver::DrawDistortedFB()
{
  static bool bFirstRun = true;
  static GLuint nList;
  mGLWindow.SetupViewport();
  mGLWindow.SetupUnitOrtho();
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  glEnable(GL_TEXTURE_RECTANGLE_ARB);
  glBindTexture(GL_TEXTURE_RECTANGLE_ARB, mnFrameBufferTex);
  glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glDisable(GL_POLYGON_SMOOTH);
  glDisable(GL_BLEND);
  if(bFirstRun)
    {
      bFirstRun = false;
      nList = glGenLists(1);
      glNewList(nList, GL_COMPILE_AND_EXECUTE);
      // How many grid divisions in the x and y directions to use?
      int nStepsX = 24; // Pretty arbitrary..
      int nStepsY = (int) (nStepsX * ((double) mirFrameSize.x / mirFrameSize.y)); // Scaled by aspect ratio
      if(nStepsY < 2)
	nStepsY = 2;
      glColor3f(1,1,1);
      for(int ystep = 0; ystep<nStepsY; ystep++)
	{
	  glBegin(GL_QUAD_STRIP);
	  for(int xstep = 0; xstep<=nStepsX; xstep++)
	    for(int yystep = ystep; yystep<=ystep + 1; yystep++) // Two y-coords in one go - magic.
	      {
		Vector<2> v2Iter;
		v2Iter[0] = (double) xstep / nStepsX;
		v2Iter[1] = (double) yystep / nStepsY;
		Vector<2> v2UFBDistorted = v2Iter;
		Vector<2> v2UFBUnDistorted = mCamera.UFBLinearProject(mCamera.UFBUnProject(v2UFBDistorted));
		glTexCoord2d(v2UFBUnDistorted[0] * mirFBSize.x, (1.0 - v2UFBUnDistorted[1]) * mirFBSize.y);
		glVertex(v2UFBDistorted);
	      }
	  glEnd();
	}
      glEndList();
    }
  else
    glCallList(nList);
  glDisable(GL_TEXTURE_RECTANGLE_ARB);
}

/**
 * Draw the fading grid
 */
void ARDriver::DrawFadingGrid()
{
  double dStrength;
  if(mnCounter >= 60)
    return;
  if(mnCounter < 30)
    dStrength = 1.0;
  dStrength = (60 - mnCounter) / 30.0;

  glColor4f(1,1,1,dStrength);
  int nHalfCells = 8;
  if(mnCounter < 8)
    nHalfCells = mnCounter + 1;
  int nTot = nHalfCells * 2 + 1;
  Vector<3>  aaVertex[17][17];
  for(int i=0; i<nTot; i++)
    for(int j=0; j<nTot; j++)
      {
	Vector<3> v3;
	v3[0] = (i - nHalfCells) * 0.1;
	v3[1] = (j - nHalfCells) * 0.1;
	v3[2] = 0.0;
	aaVertex[i][j] = v3;
      }

  glEnable(GL_LINE_SMOOTH);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glLineWidth(2);
  for(int i=0; i<nTot; i++)
    {
      glBegin(GL_LINE_STRIP);
      for(int j=0; j<nTot; j++)
	glVertex(aaVertex[i][j]);
      glEnd();

      glBegin(GL_LINE_STRIP);
      for(int j=0; j<nTot; j++)
	glVertex(aaVertex[j][i]);
      glEnd();
    };
}


/**
 * What to do when the user clicks on the screen.
 * Calculates the 3d postion of the click on the plane
 * and passes info to a game, if there is one.
 * @param nButton the button pressed
 * @param irWin the window x, y location 
 */
void ARDriver::HandleClick(int nButton, ImageRef irWin )
{
  //The window may have been resized, so want to work out the coords based on the orignal image size
  Vector<2> v2VidCoords = mGLWindow.VidFromWinCoords( irWin );
  
  
  Vector<2> v2UFBCoords;
#ifdef WIN32
  Vector<2> v2PlaneCoords;   v2PlaneCoords[0] = numeric_limits<double>::quiet_NaN();   v2PlaneCoords[1] = numeric_limits<double>::quiet_NaN();
#else
  Vector<2> v2PlaneCoords;   v2PlaneCoords[0] = NAN;   v2PlaneCoords[1] = NAN;
#endif
  Vector<3> v3RayDirn_W;

  // Work out image coords 0..1:
  v2UFBCoords[0] = (v2VidCoords[0] + 0.5) / mCamera.GetImageSize()[0];
  v2UFBCoords[1] = (v2VidCoords[1] + 0.5) / mCamera.GetImageSize()[1];

  // Work out plane coords:
  Vector<2> v2ImPlane = mCamera.UnProject(v2VidCoords);
  Vector<3> v3C = unproject(v2ImPlane);
  Vector<4> v4C = unproject(v3C);
  SE3<> se3CamInv = mse3CfromW.inverse();
  Vector<4> v4W = se3CamInv * v4C;
  double t = se3CamInv.get_translation()[2];
  double dDistToPlane = -t / (v4W[2] - t);

  if(v4W[2] -t <= 0) // Clicked the wrong side of the horizon?
  {
    v4C.slice<0,3>() *= dDistToPlane;
    Vector<4> v4Result = se3CamInv * v4C;
    v2PlaneCoords = v4Result.slice<0,2>(); // <--- result
  }

  // Ray dirn:
  v3RayDirn_W = v4W.slice<0,3>() - se3CamInv.get_translation();
  normalize(v3RayDirn_W);

  if(mpMap->pGame) {
    mpMap->pGame->HandleClick(v2VidCoords, v2UFBCoords, v3RayDirn_W, v2PlaneCoords, nButton);
  }
}



/**
 * Handle the user pressing a key
 * @param sKey the key the user pressed.
 */
void ARDriver::HandleKeyPress( std::string sKey )
{
  if(mpMap && mpMap->pGame ) {
    mpMap->pGame->HandleKeyPress( sKey );
  }

}


/**
 * Load a game by name.
 * @param sName Name of the game
 */
void ARDriver::LoadGame(std::string sName)
{
  if(mpMap->pGame)
  {
    delete mpMap->pGame;
    mpMap->pGame = NULL;
  }

  mpMap->pGame = LoadAGame( sName, "");
  if( mpMap->pGame ) {
    mpMap->pGame->Init();
  }
 
}



/**
 * Advance the game logic
 */
void ARDriver::AdvanceLogic()
{
  if(mpMap->pGame) {
    mpMap->pGame->Advance();
  }
}


}
