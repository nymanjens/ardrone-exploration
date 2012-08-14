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

#include "ShooterGame.h"
#include "MapPoint.h"
#include "Map.h"
#include "OpenGL.h"
#include "tinyxml.h"

#include <cvd/gl_helpers.h>
#include <gvars3/instances.h>

#include <sstream>
#include <iomanip>

namespace PTAMM {

using namespace CVD;
using namespace TooN;
using namespace GVars3;
             
class Map;
             
/**
 * Constructor
 */
ShooterGame::ShooterGame()
 : Game( "Shooter" ),
   mpMap(NULL),
   mdVersion(1.0)
{
  Reset();
}

/**
 * Destructor
 */
ShooterGame::~ShooterGame()
{
  _ResetGameState();
  
  if( mbInitialised ) {
    glDeleteLists( mnSphereDisplayList, 1 );
  }
}


/**
 * Reset the game.
 */
void ShooterGame::Reset()
{
  mbInitialised = false;
  _ResetGameState();
  
}


/**
 * Reset the game state for a new run.
 */
void ShooterGame::_ResetGameState()
{
  mnScore = 0;
  mnHealth = 100;
  mbAlive = true;

  list< ShooterGameTarget* >::iterator it;

  for ( it = mTargets.begin(); it != mTargets.end(); ++it ) {
    delete (*it);
  }

  mTargets.clear();
  mBullets.clear();
}


/**
 * Create the initial target set
 */
void ShooterGame::_CreateInitialTargets()
{
  if( !mpMap ) {
    return;
  }
  
  int nTargets = min( static_cast<int>(mpMap->vpPoints.size()),
                      GV3::get<int>("ShooterGame.NumStartTargets", "20", SILENT) );
  
  for( int ii = 0; ii < nTargets; ++ii )
  {
    int n = rand() % static_cast<int>(mpMap->vpPoints.size());
    Vector<3> v3 = mpMap->vpPoints.at(n)->v3WorldPos;

    ShooterGameTarget * pT = new ShooterGameTarget( v3, mnSphereDisplayList );
    mTargets.push_back( pT );
  }  
}


/**
 * Initialize the game.
 */
void ShooterGame::Init()
{
  if( mbInitialised ) {
    return;
  }

  if( !mpMap || !mpMap->bGood ) {
    return;
  }
  
  // make the sphere displaylist - this is used by the targets and the bullets
  GLUquadricObj * quadric = NULL;
  mnSphereDisplayList = glGenLists(1);
  glNewList (mnSphereDisplayList, GL_COMPILE);
  quadric = gluNewQuadric();
  gluSphere(quadric, 1.0, 36,36);
  gluDeleteQuadric(quadric);
  glEndList ();

  // start with some random targets
  _CreateInitialTargets();
  
  mbInitialised = true;
  
}



/**
 * Draw the 3D components (the targets and the bullets)
 * @param glWindow The GL Window
 * @param map The current map
 * @param se3CfromW The current camera position
 */
void ShooterGame::Draw3D( const GLWindow2 &glWindow, Map &map, SE3<> se3CfromW)
{
  if( !mbInitialised ) {
    Init();
  }

  mpMap = &map;
  mse3CfW = se3CfromW;

  if( !mbAlive ) {
    return;
  }
  
  glEnable(GL_BLEND);
  glEnable(GL_DEPTH_TEST);
  glDepthFunc(GL_LEQUAL);
  glEnable(GL_LIGHTING);
  glEnable(GL_LIGHT0);
  glEnable(GL_NORMALIZE);
  glEnable(GL_COLOR_MATERIAL);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  GLfloat tmp[]={0.5, 0.5, 0.5, 1.0};
  glLightfv(GL_LIGHT0, GL_AMBIENT, tmp);
  glLightfv(GL_LIGHT0, GL_DIFFUSE, tmp);
  GLfloat tmp2[]={1.0, 0.0, 1.0, 0.0};
  glLightfv(GL_LIGHT0, GL_POSITION, tmp2);
  GLfloat tmp3[]={1.0, 1.0, 1.0, 1.0};
  glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, tmp3);
  glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 50.0);

  glMatrixMode(GL_MODELVIEW);
  
  glLoadIdentity();
  glMultMatrix(SE3<>());


  // Draw the targets
  std::list< ShooterGameTarget * >::const_iterator itr;
  for( itr = mTargets.begin(); itr != mTargets.end(); ++itr )  {
    (*itr)->Draw();
  }

  // Draw the bullets
  glColor3f( 1.0, 0.0, 1.0);
  double dScale = GV3::get<double>("ShooterGame.BulletSize", "0.001", SILENT);
  std::list< Bullet >::const_iterator it;
  
  for( it = mBullets.begin(); it != mBullets.end(); ++it )
  {
    glPushMatrix();
    glTranslate( (*it).v3Position );
    glScaled( dScale, dScale, dScale );
    glCallList( mnSphereDisplayList );
    glPopMatrix();
  }

  glDisable(GL_LIGHTING);
  glDisable(GL_DEPTH_TEST);

}


/**
 * Draw the 2D components (menus etc)
 * @param glWindow the gl window
 * @param map the associated map
 */
void ShooterGame::Draw2D( const GLWindow2 &glWindow, Map &map)
{
  glEnable(GL_BLEND);

  if( mbAlive )
  {
    //draw the reticule
    ImageRef irCentre = glWindow.size()/2;
    
    glLineWidth(3);
    glColor4f( 0.0f, 0.0f, 0.0f, 0.8f);
    
    glBegin(GL_LINES);
    glVertex2d( irCentre.x - 10, irCentre.y );
    glVertex2d( irCentre.x - 30, irCentre.y );
        
    glVertex2d( irCentre.x + 10, irCentre.y );
    glVertex2d( irCentre.x + 30, irCentre.y );

    glVertex2d( irCentre.x, irCentre.y - 10 );
    glVertex2d( irCentre.x, irCentre.y - 30 );
        
    glVertex2d( irCentre.x, irCentre.y + 10 );
    glVertex2d( irCentre.x, irCentre.y + 30 );
    
    glEnd();

    glPointSize(1);
    glBegin(GL_POINTS);
    glVertex2d( irCentre.x, irCentre.y );
    glEnd();
  }
  else
  {
    int x = glWindow.size().x/2-30;
    int y = glWindow.size().y/2;
    glWindow.DrawBox( x - 10, y - 18, 120, 3, 0.8 );
    glColor3f( 1.0, 1.0, 1.0);    
    glWindow.PrintString( ImageRef( x, y  ), "Game Over" );
    glWindow.PrintString( ImageRef( x, y + 20 ), "Press 's' to Start" );
  }


  // Print the user's health and score.
  ostringstream os;
  os << "Health: " << setfill(' ') << setw(3) << mnHealth << "\t\t Score: " << mnScore;

  glColor3f( 1.0, 1.0, 1.0);
  glWindow.PrintString( ImageRef( 10, glWindow.size().y - 20 ), os.str() );
  
  glDisable(GL_BLEND);
}


/**
 * Handle the user pressing a key
 * Return key shoots, s key restarts the game
 * @param sKey the key pressed
 */
void ShooterGame::HandleKeyPress( std::string sKey )
{
  if( sKey == "Enter" )
  {
    Bullet bullet;
    bullet.v3Position = mse3CfW.inverse().get_translation();
    SO3<> so3 = mse3CfW.inverse().get_rotation();
    Vector<3> v3z = makeVector( 0.0, 0.0, 1.0 );
    bullet.v3Direction = so3 * v3z;
    mBullets.push_back( bullet );
  }
  else if( sKey == "s" || sKey == "S" )
  {
    _ResetGameState();
    _CreateInitialTargets();
  }
  
}



/**
 * Save the game to disk
 * @param sMapPath Path to the map
 * @return the file name
 */
std::string ShooterGame::Save(std::string sMapPath)
{
  std::string sFileName = "ShooterGame.xml";
  std::string sFilePath = sMapPath + "/" + sFileName;

  TiXmlDocument xmlDoc;     //XML file

  //open the map file for writing
  TiXmlDeclaration* decl = new TiXmlDeclaration( "1.0", "", "" );
  xmlDoc.LinkEndChild( decl );

  TiXmlElement * rootNode = new TiXmlElement( Name() );
  xmlDoc.LinkEndChild( rootNode );
  rootNode->SetDoubleAttribute("version", mdVersion);

  TiXmlElement * playerNode = new TiXmlElement( "Player" );
  rootNode->LinkEndChild( playerNode );
  playerNode->SetAttribute( "health", mnHealth );
  playerNode->SetAttribute( "score", mnScore );


  
  TiXmlElement * node = new TiXmlElement( "Targets" );
  rootNode->LinkEndChild( node );
  node->SetAttribute( "size", static_cast<int>(mTargets.size()) );


  std::list< ShooterGameTarget * >::iterator itr;
  for( itr = mTargets.begin(); itr != mTargets.end(); ++itr )  {
    (*itr)->Save( node );
  }

  xmlDoc.SaveFile(sFilePath);

  
  return sFileName;
}


/**
 * Load a game from disk
 * @param sDataFileName the path to the game file.
 */
void ShooterGame::Load( std::string sDataFileName )
{
  TiXmlDocument mXMLDoc;        //XML file

  //load the XML file
  if( !mXMLDoc.LoadFile( sDataFileName ) )  {
    cerr << "Failed to load " << sDataFileName << " game file. Aborting." << endl;
    return;
  }
  
  TiXmlHandle hDoc(&mXMLDoc);
  TiXmlElement* pElem;
  TiXmlHandle hRoot(0);
  
  pElem = hDoc.FirstChildElement().Element();
  // should always have a valid root but handle gracefully if it does not
  if (!pElem)
  {
    cerr << "No root handle in XML file " << sDataFileName << endl;
    return;
  }

  string sID( Name() );
  double dFileVersion = 0.0;
  pElem->QueryDoubleAttribute("version", &dFileVersion);
  
  if( ( sID.compare( pElem->Value() ) != 0 ) &&
      ( dFileVersion != mdVersion ) )
  {
    cerr << "Invalid XML file. Need a version " << mdVersion << " " << sID
         << " XML file. Not a version " << dFileVersion << " " << pElem->Value() << " file." << endl;
    return;
  }

  // save this for later
  hRoot = TiXmlHandle(pElem);

  TiXmlHandle playerNode = hRoot.FirstChild( "Player" );
  playerNode.ToElement()->QueryIntAttribute("health", &mnHealth);
  playerNode.ToElement()->QueryIntAttribute("score", &mnScore);

  int nSize = -1;
  TiXmlHandle node = hRoot.FirstChild( "Targets" );
  node.ToElement()->QueryIntAttribute("size", &nSize);

  for(TiXmlElement* pElem = node.FirstChild().Element();
      pElem != NULL;
      pElem = pElem->NextSiblingElement() )
  {
    ShooterGameTarget * pTarget = new ShooterGameTarget( makeVector(0,0,0), mnSphereDisplayList);
    pTarget->Load( pElem );
    mTargets.push_back(pTarget);
  }
  
  if( static_cast<int>(mTargets.size() ) != nSize ) {
    cerr << "Warning: Loaded the wrong number of targets. " << mTargets.size()
        << " instead of " << nSize << "." << endl;
  }

}


/**
 * Advance the game logic
 */
void ShooterGame::Advance()
{
  if( !mbInitialised || !mpMap || !mpMap->bGood || !mbAlive ) {
    return;
  }

  static int nCounter = 0;
  nCounter++;

  
  //update bullets
  std::list< Bullet >::iterator it;
  for( it = mBullets.begin(); it != mBullets.end(); ++it )
  {
    (*it).v3Position += (*it).v3Direction * (*it).dVelocity;
    (*it).nStrength--;

    if( (*it).nStrength <= 0 )
    {
       it = mBullets.erase(it);
       it--;
    }
  }

  //update targets
  std::list< ShooterGameTarget * >::iterator itr;
  bool bAbort = false;
  for( itr = mTargets.begin(); itr != mTargets.end(); ++itr )
  {
    (*itr)->Update();

    //check for shot collision
    for( it = mBullets.begin(); it != mBullets.end(); ++it )
    {
      bool bHit = (*itr)->HitCheck( (*it).v3Position );
      if( bHit )
      {
        (*itr)->Strength() -= (*it).nDamage;
        it = mBullets.erase( it );
        it--;

        if( (*itr)->Strength() <= 0 )
        {
          itr = mTargets.erase( itr );
          itr--;
          mnScore++;
          bAbort = true;
          break;
        }
      }
    }

    if( bAbort ) {
      bAbort = false;
      break;
    }
      

    //check for camera collision
    if( (*itr)->HitCheck( mse3CfW.inverse().get_translation() ) )
    {
      mnHealth -= (*itr)->Strength()/10;

      itr = mTargets.erase( itr );
      itr--;
      continue;
    }
  }

  //generate some more targets
  if( static_cast<int>( mTargets.size() ) <  GV3::get<int>("ShooterGame.MaxTargets", "100", SILENT) &&
      nCounter > GV3::get<int>("ShooterGame.Delay", "90", SILENT) )
  {
    nCounter = 0;

    int n = rand() % static_cast<int>(mpMap->vpPoints.size());
    Vector<3> v3 = mpMap->vpPoints.at(n)->v3WorldPos;
    
    ShooterGameTarget * pT = new ShooterGameTarget( v3, mnSphereDisplayList );
    mTargets.push_back( pT );
  }

  if( static_cast<int>(mTargets.size()) < GV3::get<int>("ShooterGame.MinForSpawn", "5", SILENT) ) {
    _CreateInitialTargets();
  }

  //have we killed the player yet?
  if( mnHealth <= 0 )
  {
    mnHealth = 0;
    mbAlive = false;
  }

}


}


