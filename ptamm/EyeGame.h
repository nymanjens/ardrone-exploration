// -*- c++ -*-
// Copyright 2009 Isis Innovation Limited
//
// EyeGame.h
// Declares the EyeGame class
// EyeGame is a trivial AR app which draws some 3D graphics
// Draws a bunch of 3d eyeballs remniscient of the 
// AVL logo
//
#ifndef __EYEGAME_H
#define __EYEGAME_H

#include <TooN/TooN.h>
#include "OpenGL.h"
#include "Game.h"

namespace PTAMM {

using namespace TooN;

class EyeGame : public Game
{
  public:
    EyeGame( );
    void Draw3D( const GLWindow2 &glWindow, Map &map, SE3<> se3CfromW );
    void Reset();
    void Init();

  
  protected:
    void DrawEye();
    void LookAt(int nEye, Vector<3> v3, double dRotLimit);
    void MakeShadowTex();

  protected:
    GLuint mnEyeDisplayList;                  // Display list for an eye
    GLuint mnShadowTex;                       // Texture for the shadow
    double mdEyeRadius;                       // The size of the eye
    double mdShadowHalfSize;                  // The size of the shadow
    SE3<> ase3WorldFromEye[4];                // The locations of the eyes
    int mnFrameCounter;                       // A frame counter

};

}

#endif
