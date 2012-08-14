// Copyright 2009 Isis Innovation Limited
//
// C++ Interface: ShooterGameTarget
//
// Description: A target class for the shooter game
//
//
// Author: Robert Castle <bob@robots.ox.ac.uk>, (C) 2009
//
// Copyright: See COPYING file that comes with this distribution
//
//
#ifndef PTAMMSHOOTERGAMETARGET_H
#define PTAMMSHOOTERGAMETARGET_H

#include <vector>
#include <TooN/TooN.h>
#include "OpenGL.h"

namespace PTAMM {

class TiXmlElement;
  
/**
 * A target for the shooter game.
 * @author Robert Castle <bob@robots.ox.ac.uk>
*/
class ShooterGameTarget
{
  public:
    ShooterGameTarget( TooN::Vector<3> v3Loc, const GLuint & nSphereDisplayList );
    ~ShooterGameTarget() {};

    void Draw();
    void Update();
    bool HitCheck( TooN::Vector<3> v3Pos );
    int & Strength() { return mnStrength; }  /// Get or set the strength of the target

    void Save( TiXmlElement * targetsNode );
    void Load( TiXmlElement * targetsNode );

  private:
    void _AssignProperties();

  private:
    const GLuint & mnSphereDisplayList;       // The sphere display list
    float mafColor[3];                        // Its colour
    TooN::Vector<3> v3Position;               // Its location in the world
    double mdScale;                           // Its scale
    double mdGrowthRate;                      // Its growth rate
    int mnStrength;                           // Its health

};

}

#endif
