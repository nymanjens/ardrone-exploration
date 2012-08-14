// Copyright 2009 Isis Innovation Limited
//
// C++ Implementation: ModelsGame
//
// Description: An AR game for placing 3ds models in a map
//
// Author: Robert Castle <bob@robots.ox.ac.uk>, (C) 2009
//
#include "ModelsGame.h"
#include "OpenGL.h"
#include "MapPoint.h"
#include "Map.h"

#include <cvd/gl_helpers.h>
#include <cvd/image_io.h>
#include <gvars3/instances.h>


namespace PTAMM {

using namespace CVD;
using namespace TooN;
using namespace GVars3;
             
class Map;
             
/**
 * Constructor
 */
ModelsGame::ModelsGame()
 : Game( "Models" ),
   mData( Name() ),
   mModelControls( mData )
{
  Reset();
}

/**
 * Destructor
 */
ModelsGame::~ModelsGame()
{
}



/**
 * Reset the game.
 */
void ModelsGame::Reset()
{
  mData.Reset();
}


/**
 * Initialize the game.
 */
void ModelsGame::Init()
{
  if( mbInitialised ) {
    return;
  }

  bool bOk = false;

  // initialize the model browser
  bOk = mModelBrowser.Init( ImageRef(BROWSER_X, BROWSER_Y), "ARData/Overlays/model_browser.png");

  if( !bOk ) {
    return;
  }

  // initialize the model controls
  mModelControls.Init();
  

  // load the current model target
  try {
    Image<Rgba<CVD::byte> > imTarget;
    CVD::img_load( imTarget, "ARData/Overlays/selected.png" );
    glGenTextures( 1, &mnTargetTex);
    glBindTexture( GL_TEXTURE_2D, mnTargetTex );
    glTexImage2D(imTarget);
    glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR );
    glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR );  
  }
  catch(CVD::Exceptions::All err) {
    cerr << "Failed to load target image " << "ARData/Overlays/selected.png" << ": " << err.what << endl;
    return;
  }

 
  mbInitialised = true;
}



/**
 * Draw the 3D components (the 3DS models)
 * @param glWindow The GL Window
 * @param map the current map
 * @param se3CfromW The current camera position
 */
void ModelsGame::Draw3D( const GLWindow2 &glWindow, Map &map, SE3<> se3CfromW)
{
  if( !mbInitialised ) {
    Init();
  }

  if( mData.mbHideAR )  {
    return;
  }

  mpMap = &map;
  mse3CfW = se3CfromW;
  
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

  // Display the map points?
  if( !mData.mbHidePoints )
  {
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glPointSize(5);
    glColor3f(1,1,1);
    glBegin(GL_POINTS);
    for( size_t ii = 0; ii < map.vpPoints.size(); ++ii )
    {
      MapPoint &p = *(map.vpPoints[ii]);
      if( p.nSourceLevel != 0 ) {
        continue;
      }

      glVertex( p.v3WorldPos );
    }
    glEnd();
  }  


  // Draw the origin axes ?
  if( GV3::get<int>("ModelsGame.DrawOrigin", "0", SILENT) ) {
    _DrawAxes();
  }


  // Draw the models
  std::vector<Model3DS*>::const_iterator itr;
  for ( itr = mData.Models().begin(); itr != mData.Models().end(); ++itr )
  {
    (*itr)->Draw();
  }


  // Find the current model and draw the target
  Model3DS * m = mData.CurrentModel();
  if( m && !mData.mbHideControls )
  {
    glPushMatrix();
    glMultMatrix( m->Pose() );
    glScaled( m->Diameter(), m->Diameter(), m->Diameter() );
    _DrawSelectedTarget();
    glPopMatrix();
  }
  
  glDisable(GL_LIGHTING);
  glDisable(GL_DEPTH_TEST);

  // A rather nasty hack to get around the window event limitations.
  // Check to see if the user is holding the mouse button down and if
  // it is on a repeating button, call it again.
  if( mData.mDisplayState == ModelsGameData::ControlState &&
      glWindow.IsMouseButtonPressed( CVD::GLWindow::BUTTON_LEFT ) ) {
    mModelControls.HandlePressAndHold();
  }
}


/**
 * Draw the 2D components (menus etc)
 * @param glWindow The GL Window
 * @param map The current map
 */
void ModelsGame::Draw2D( const GLWindow2 &glWindow, Map &map)
{
  glEnable(GL_BLEND);
  
  switch(mData.mDisplayState)
  {
    case ModelsGameData::BrowserState:
      mModelBrowser.Draw(glWindow);       // draw the model browser
      break;
    case ModelsGameData::ControlState:
      mModelControls.Draw(glWindow);      // draw the controls
      break;
    case ModelsGameData::HiddenState:     // nothing to draw
    default:
      //do nothing
      break;
  }
  
  glDisable(GL_BLEND);
}


/**
 * Handle a click from the user. The user will either be clicking
 * on a button overlay or on the scene.
 * @param v2VidCoords location in the image coords.
 * @param v2UFB Location in the undistorted framebuffer coords
 * @param v3RayDirnW Direction of the click ray
 * @param v2Plane The in-plane location of the click
 * @param nButton The button clicked
 */
void ModelsGame::HandleClick(Vector<2> v2VidCoords, Vector<2> v2UFB, Vector<3> v3RayDirnW, Vector<2> v2Plane, int nButton)
{
  //select appropriate reponse for each overlay
  switch(mData.mDisplayState)
  { 
    case ModelsGameData::BrowserState:
      _HandleModelBrowserActions( v2VidCoords );
      break;
    case ModelsGameData::ControlState:
      if( !mModelControls.HandleClick( v2VidCoords, nButton ) && mData.mbSnapTo
           && !mData.mbHideControls  && !mData.mbHideAR)
      {
        Model3DS *p = mData.CurrentModel();
        if( p )
        {
          Vector<3> v3Loc = _SelectClosestMapPoint( v3RayDirnW );
          p->MoveTo( v3Loc );
        }
      }
      break;
    default:
      //do nothing
      break;
  }
}



/**
 * Handle and act on the results from the user clicking on the model browser.
 * @param v2VidCoords the video coordinates
 * @return true if used the click
 */
bool ModelsGame::_HandleModelBrowserActions( Vector<2> v2VidCoords )
{
  ModelBrowser::CLICK_STATUS cs = mModelBrowser.HandleClick( v2VidCoords );

  if(cs == ModelBrowser::MODEL)
  {
    const ModelData * pModelData = mModelBrowser.GetSelectedModelData();

    if( pModelData ) {
      mData.AddModel( pModelData->sLocation, pModelData->sModelFile, pModelData->sName, pModelData->v3Rotation );
    }
  }
  else if( cs == ModelBrowser::CANCEL )
  {
    if( mData.NumModels() != 0 ) {
      mData.mDisplayState = ModelsGameData::ControlState;
    }
  }
  else if( cs == ModelBrowser::OK ) {
  }
  else {
    return false;
  }

  return true;

}


/**
 * Select the closest map point to the ray from click
 */
Vector<3> ModelsGame::_SelectClosestMapPoint( Vector<3> v3RayDirnW )
{
  if(!mpMap) {
    return makeVector( 0, 0, 0 );
  }

  Vector<3> v3CamPos = mse3CfW.inverse().get_translation();
  
  Vector<3> v3Best;
  MapPoint* pBest = NULL;
  double dBest = -999999.8;
  for( size_t i = 0; i < mpMap->vpPoints.size(); i++ )
  {
    MapPoint &p = *(mpMap->vpPoints[i]);
    if( p.nSourceLevel != 0 ) {
      continue;
    }
    Vector<3> v3RayToPoint = p.v3WorldPos - v3CamPos;
    normalize(v3RayToPoint);
    if( (v3RayToPoint * v3RayDirnW) > dBest )
    {
      dBest = v3RayToPoint * v3RayDirnW;
      pBest = &p;
    }
  }

  return pBest->v3WorldPos;
}


/**
 * Draw a coordinate frame
 */
void ModelsGame::_DrawAxes()
{
  float w = 0;
  glGetFloatv(GL_LINE_WIDTH, &w);

  glLineWidth(5);
  const float len = 0.5;

  //draw axis
  glBegin(GL_LINES);
  glColor4f(1.0f,0.0f,0.0f,1.0f);
  glVertex3f(0.0f,0.0f,0.0f);
  glVertex3f(len,0.0f,0.0f);

  glColor4f(0.0f,1.0f,0.0f,1.0f);
  glVertex3f(0.0f,0.0f,0.0f);
  glVertex3f(0.0f,len,0.0f);

  glColor4f(0.0f,0.0f,1.0f,1.0f);
  glVertex3f(0.0f,0.0f,0.0f);
  glVertex3f(0.0f,0.0f,len);

  glEnd();

  glLineWidth(w);
}



/**
 * Draw the selection target
 */
void ModelsGame::_DrawSelectedTarget()
{
  glPushMatrix();

  glEnable(GL_BLEND);
  glEnable(GL_DEPTH_TEST);
  glDepthFunc(GL_LEQUAL);
  glEnable(GL_NORMALIZE);
  glEnable(GL_TEXTURE_2D);
  glShadeModel(GL_SMOOTH);
  glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);
  glDisable(GL_LIGHTING);
  
  //draw the selected icon
  glColor4f(1,1,1,1);
  glBindTexture ( GL_TEXTURE_2D , mnTargetTex );
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  glBegin(GL_QUADS);
  glTexCoord2f(0.0,0.0);  glVertex3f( 0.5,  0.5, 0.0);
  glTexCoord2f(1.0,0.0);  glVertex3f( 0.5, -0.5, 0.0);
  glTexCoord2f(1.0,1.0);  glVertex3f(-0.5, -0.5, 0.0);
  glTexCoord2f(0.0,1.0);  glVertex3f(-0.5,  0.5, 0.0);
  glEnd();
  
  glDisable(GL_TEXTURE_2D);
  glDisable(GL_DEPTH_TEST);
  
  glPopMatrix();
}


/**
 * Save the game to disk
 * @param sMapPath Path to the map
 * @return The file name
 */
std::string ModelsGame::Save(std::string sMapPath)
{
  std::string sFileName = "ModelsGame.xml";
  std::string sFilePath = sMapPath + "/" + sFileName;
  
  mData.Save( sFilePath );
  
  return sFileName;
}

/**
 * Load a game from disk
 * @param sDataPath The path to the game file.
 */
void ModelsGame::Load(std::string sDataPath)
{
  mData.Load( sDataPath );
}



}


