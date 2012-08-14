// Copyright 2009 Isis Innovation Limited

#include "ShooterGameTarget.h"
#include "Utils.h"
#include "tinyxml.h"

#include <cvd/gl_helpers.h>
#include <sstream>


namespace PTAMM {

using namespace CVD;
using namespace TooN;

/**
 * Create a target
 * @param v3Loc Location of the target in the world
 * @param nSphereDisplayList the sphere display list
 */
ShooterGameTarget::ShooterGameTarget( TooN::Vector<3> v3Loc, const GLuint & nSphereDisplayList )
  : mnSphereDisplayList(nSphereDisplayList),
    v3Position(v3Loc),
    mnStrength(100)
{
  _AssignProperties();
}


/**
 * Draw the target
 */
void ShooterGameTarget::Draw()
{
  glPushMatrix();
  glTranslate( v3Position );
  glScaled( mdScale, mdScale, mdScale );
  
  glColor3fv( mafColor );
  glCallList( mnSphereDisplayList );

  glPopMatrix();
}


/**
 * Assign some random properties to the target
 * Most of the properties set here could be made into GVars for experimentation
 */
void ShooterGameTarget::_AssignProperties()
{
  //randomly select a colour
  mafColor[0] = static_cast<double>(rand()) / static_cast<double>(RAND_MAX);
  mafColor[1] = static_cast<double>(rand()) / static_cast<double>(RAND_MAX);
  mafColor[2] = static_cast<double>(rand()) / static_cast<double>(RAND_MAX);

  //set initaial scale
  mdScale = 0.01;

  // how fast will it grow
  mdGrowthRate = static_cast<double>(rand()) / (1000 * static_cast<double>(RAND_MAX) );

  //how strong will it be
  mnStrength = (rand() % 50) + 50;
}


/**
 * Update the target. This makes it bigger
 */
void ShooterGameTarget::Update()
{
  mdScale += mdGrowthRate;
}


/**
 * Is the location specified within the target boundary?
 * @param v3Pos the location to check
 */
bool ShooterGameTarget::HitCheck( Vector<3> v3Pos )
{
  Vector<3> v3 = v3Pos - v3Position;
  double distsq = v3[0] * v3[0] + v3[1] * v3[1] + v3[2] * v3[2];

  if( (mdScale * mdScale) > distsq ) {
    return true;
  }

  return false;
}

/**
 * Save a target's state to disk
 * @param targetsNode the tinyXML element to append to
 */
void ShooterGameTarget::Save(  TiXmlElement * targetsNode )
{
  if( !targetsNode ) {
    return;
  }

  TiXmlElement * node = new TiXmlElement( "Target" );
  targetsNode->LinkEndChild( node );

  std::ostringstream os;

  os << mafColor[0] << " " << mafColor[1] << " " << mafColor[2];
  
  node->SetAttribute( "color", os.str() );
  node->SetDoubleAttribute( "scale", mdScale );
  node->SetDoubleAttribute( "growth", mdGrowthRate );
  node->SetAttribute( "strength", mnStrength );

  os.str("");
  std::string s;

  os << v3Position;
  s = os.str();
  PruneWhiteSpace( s );
  node->SetAttribute("position", s );
}

/**
 * Load a target's state from disk
 * @param targetsNode the TinyXML element to read from
 * @return success
 */
void ShooterGameTarget::Load( TiXmlElement * targetsNode )
{
  if( !targetsNode) {
    return;
  }

  std::string tmp;
    
  //read the color
  tmp = targetsNode->Attribute("color");
  sscanf(tmp.c_str(), "%f %f %f", &mafColor[0], &mafColor[1], &mafColor[2]);
  
  //read the position
  tmp = targetsNode->Attribute("position");
  sscanf(tmp.c_str(), "%lf %lf %lf", &v3Position[0], &v3Position[1], &v3Position[2]);
  
  //read the scale, grwoth and strength
  targetsNode->QueryDoubleAttribute("scale", &mdScale);
  targetsNode->QueryDoubleAttribute("growth", &mdGrowthRate);
  targetsNode->QueryIntAttribute("strength", &mnStrength);
}


}
