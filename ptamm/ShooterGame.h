// Copyright 2009 Isis Innovation Limited
//
// C++ Interface: ShooterGame
//
// Description: A simple AR game where the user has to shoot the sprouting targets.
//
//
// Author: Robert Castle <bob@robots.ox.ac.uk>, (C) 2009
//
//
#ifndef SHOOTERGAME_H
#define SHOOTERGAME_H

#include "Game.h"
#include "ShooterGameTarget.h"

#include <list>

namespace PTAMM {

struct Bullet {
    Vector<3> v3Position;
    Vector<3> v3Direction;
    double dVelocity;
    int nStrength;
    int nDamage;

    Bullet() : dVelocity(0.01), nStrength(100), nDamage(20) {}
};
  
/**
 * A simple AR game where targets sprout from random map points and the user
 * has to aim the camera to shot them.
 * @author Robert Castle <bob@robots.ox.ac.uk>
*/
class ShooterGame : public Game
{
  public:
    ShooterGame();
    ~ShooterGame();

    void Draw3D( const GLWindow2 &glWindow, Map &map, SE3<> se3CfromW);
    void Draw2D( const GLWindow2 &glWindow, Map &map);
    
    void Reset();
    void Init();
    void HandleKeyPress( std::string sKey );
    void Advance();
    
    std::string Save( std::string sMapPath );
    void Load( std::string sDataFileName );

  private:
    void _ResetGameState();
    void _CreateInitialTargets();
        
  private:
    Map * mpMap;                                  // The associated map
    SE3<> mse3CfW;                                // The camera postion

    GLuint mnSphereDisplayList;                   // The sphere display list

    std::list< ShooterGameTarget * > mTargets;    // The list of targets
    std::list< Bullet > mBullets;                 // The list of bullets
    int mnScore;                                  // The player's score
    int mnHealth;                                 // The player's health
    bool mbAlive;                                 // Is the player alive
    const double mdVersion;                       // Current game version
};

}

#endif
