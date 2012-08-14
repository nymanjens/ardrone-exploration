// -*- c++ -*-
// Copyright 2009 Isis Innovation Limited
//
// This is the base abstract game class.
// All games should derive from this.

#ifndef __GAME_H
#define __GAME_H

#include <vector>
#include <TooN/TooN.h>
 #include <TooN/se3.h>
#include <fstream>
#include "GLWindow2.h"

namespace PTAMM {

using namespace TooN;
using namespace std;
class Map;
class ARDriver;
class ATANCamera;


/**
 * The base game class
 */
class Game
{
  public:
    Game( std::string sName ) : msName(sName), mbInitialised(false) {}
    virtual ~Game() {};
    virtual void Draw3D(const GLWindow2 &glWindow, Map &map, SE3<> se3CfromW ) = 0;  /// draw the game for a map
    virtual void Draw2D(const GLWindow2 &glWindow, Map &map) {}                      /// draw any 2D overlays needed.
    virtual void Reset() = 0;                                                        /// reset
    virtual void Init() = 0;                                                         /// initialize
    virtual std::string Save(std::string sMapPath)                                   /// save game data (DO NOT MAKE GL CALLS!)
            { std::cout << msName << " has no data to save" << std::endl; return "";}
    virtual void Load(std::string sDataPath)                                         /// load game data (DO NOT MAKE GL CALLS!)
            { std::cout << msName << " has no data to load" << std::endl; }
    
    virtual void HandleClick(Vector<2> v2VidCoords, Vector<2> v2UFB, Vector<3> v3RayDirnW, 
                             Vector<2> v2Plane, int nButton) {}                      /// Handle a mouse click
    virtual void HandleKeyPress( std::string sKey ) {}                               /// Handle a key press
    virtual void Advance() {}                                                        /// Advance the game logic

    std::string Name() { return msName; }                                            /// The name of the game

  protected:
    const std::string msName;                                                       // Name of the game
    bool mbInitialised;                                                             // Has the game been initialized
};

}

#endif

