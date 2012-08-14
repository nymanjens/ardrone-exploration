// -*- c++ -*-
// Copyright 2009 Isis Innovation Limited
// ARDriver.h
// This file declares the ARDriver class
//
// ARDriver provides basic graphics services for drawing augmented
// graphics. It manages the OpenGL setup and the camera's radial
// distortion so that real and distorted virtual graphics can be
// properly blended.
//
#ifndef __AR_Driver_H
#define __AR_Driver_H
#include <TooN/se3.h>
#include "ATANCamera.h"
#include "GLWindow2.h"
#include "OpenGL.h"
#include <cvd/image.h>
#include <cvd/rgb.h>
#include <cvd/byte.h>

#ifndef NAN
#include <limits>
#endif 

namespace PTAMM {

using namespace std;
using namespace CVD;

class Map;

class ARDriver
{
  public:
    ARDriver(const ATANCamera &cam, ImageRef irFrameSize, GLWindow2 &glw, Map &map);
	void Render(Image<Rgb<CVD::byte> > &imFrame, SE3<> se3CamFromWorld, bool bLost);
    void Reset();
    void Init();

    void HandleClick(int nButton, ImageRef irWin );
    void HandleKeyPress( std::string sKey );
    void AdvanceLogic();
    void LoadGame(std::string sName);
  
    void SetCurrentMap(Map &map) { mpMap = &map; mnCounter = 0; }

  protected:
    void DrawFadingGrid();
    void MakeFrameBuffer();
    void DrawFBBackGround();
    void DrawDistortedFB();
    void SetFrustum();
    
    bool PosAndDirnInPlane(Vector<2> v2VidCoords, Vector<2> &v2Pos, Vector<2> &v2Dirn);

    
  protected:
    ATANCamera mCamera;
    GLWindow2 &mGLWindow;
    Map *mpMap;
  
    // Texture stuff:
    GLuint mnFrameBuffer;
    GLuint mnFrameBufferTex;
    GLuint mnFrameTex;
    
    int mnCounter;
    ImageRef mirFBSize;
    ImageRef mirFrameSize;
    SE3<> mse3CfromW;
    bool mbInitialised;

	Image<Rgba<CVD::byte> > mLostOverlay;


};

}

#endif
