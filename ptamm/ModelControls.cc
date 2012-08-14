// Copyright 2009 Isis Innovation Limited

#include "ModelControls.h"
#include "MGButton.h"
#include "Model3ds.h"
#include <TooN/TooN.h>
#include <cvd/glwindow.h>
#include <iostream>

using namespace std;
using namespace CVD;
using namespace TooN;

namespace PTAMM {

/////////////////// callbacks for when the user clicks on a button ////////////////

/**
 * Callback to increase the current model's scale
 * @param data Reference to the ModelGameData class
 */
void ScaleIncrease_cb( ModelsGameData & data )
{
  Model3DS * p = data.CurrentModel();
  if( p )  {
    p->ScaleIncrease();
  }
}

/**
 * Callback to decrease the current model's scale
 * @param data Reference to the ModelGameData class
 */
void ScaleDecrease_cb(  ModelsGameData & data )
{
  Model3DS * p = data.CurrentModel();
  if( p )  {
    p->ScaleDecrease();
  }
}

/**
 * Callback to move the current model up
 * @param data Reference to the ModelGameData class
 */
void MoveUp_cb( ModelsGameData & data )
{
  Model3DS * p = data.CurrentModel();
  if( p )  {
    p->MoveUp();
  }
}

/**
 * Callback to move the current model down
 * @param data Reference to the ModelGameData class
 */
void MoveDown_cb( ModelsGameData & data )
{
  Model3DS * p = data.CurrentModel();
  if( p )  {
    p->MoveDown();
  }
}

/**
 * Callback to move the current model left
 * @param data Reference to the ModelGameData class
 */
void MoveLeft_cb( ModelsGameData & data )
{
  Model3DS * p = data.CurrentModel();
  if( p )  {
    p->MoveLeft();
  }
}

/**
 * Callback to move the current model right
 * @param data Reference to the ModelGameData class
 */
void MoveRight_cb( ModelsGameData & data )
{
  Model3DS * p = data.CurrentModel();
  if( p )  {
    p->MoveRight();
  }
}

/**
 * Callback to move the current model forwards
 * @param data Reference to the ModelGameData class
 */
void MoveFwds_cb( ModelsGameData & data )
{
  Model3DS * p = data.CurrentModel();
  if( p )  {
    p->MoveForwards();
  }
}

/**
 * Callback to move the current model back
 * @param data Reference to the ModelGameData class
 */
void MoveBack_cb( ModelsGameData & data )
{
  Model3DS * p = data.CurrentModel();
  if( p )  {
    p->MoveBack();
  }
}

/**
 * Callback to increase the the current model's roll angle
 * @param data Reference to the ModelGameData class
 */
void RollIncrease_cb( ModelsGameData & data )
{
  Model3DS * p = data.CurrentModel();
  if( p )  {
    p->RollIncrease();
  }
}

/**
 * Callback to decrease the the current model's roll angle
 * @param data Reference to the ModelGameData class
 */
void RollDecrease_cb( ModelsGameData & data )
{
  Model3DS * p = data.CurrentModel();
  if( p )  {
    p->RollDecrease();
  }
}

/**
 * Callback to increase the the current model's pitch angle
 * @param data Reference to the ModelGameData class
 */
void PitchIncrease_cb( ModelsGameData & data )
{
  Model3DS * p = data.CurrentModel();
  if( p )  {
    p->PitchIncrease();
  }
}

/**
 * Callback to decrease the the current model's pitch angle
 * @param data Reference to the ModelGameData class
 */
void PitchDecrease_cb( ModelsGameData & data )
{
  Model3DS * p = data.CurrentModel();
  if( p )  {
    p->PitchDecrease();
  }
}

/**
 * Callback to increase the the current model's yaw angle
 * @param data Reference to the ModelGameData class
 */
void YawIncrease_cb( ModelsGameData & data )
{
  Model3DS * p = data.CurrentModel();
  if( p )  {
    p->YawIncrease();
  }
}

/**
 * Callback to decrease the the current model's yaw angle
 * @param data Reference to the ModelGameData class
 */
void YawDecrease_cb( ModelsGameData & data )
{
  Model3DS * p = data.CurrentModel();
  if( p )  {
    p->YawDecrease();
  }
}

/**
 * Callback to reset the current model's pose and size
 * @param data Reference to the ModelGameData class
 */
void ResetModel_cb( ModelsGameData & data )
{
  Model3DS * p = data.CurrentModel();
  if( p ) {
    p->Reset();
  }
}

/**
 * Callback to switch to the next model in the list
 * @param data Reference to the ModelGameData class
 */
void NextModel_cb( ModelsGameData & data )
{
  data.PrevModel();
}

/**
 * Callback to switch to the previous model in the list
 * @param data Reference to the ModelGameData class
 */
void PrevModel_cb( ModelsGameData & data )
{
  data.NextModel();
}

/**
 * Callback to toggle whether the current model should snap to
 * the map points.
 * @param data Reference to the ModelGameData class
 */
void ToggleSnapTo_cb( ModelsGameData & data )
{
  data.mbSnapTo = !data.mbSnapTo;
}

/**
 * Callback to toggle whether the AR (the models) should be shown or not
 * @param data Reference to the ModelGameData class
 */
void ToggleAR_cb( ModelsGameData & data )
{
  data.mbHideAR = !data.mbHideAR;
}

/**
 * Callback to toggle whether the controls should be shown or not
 * @param data Reference to the ModelGameData class
 */
void ToggleControls_cb( ModelsGameData & data )
{
  data.mbHideControls = !data.mbHideControls;
}

/**
 * Callback to toggle whether to show the points or not
 * @param data Reference to the ModelGameData class
 */
void TogglePoints_cb( ModelsGameData & data )
{
  data.mbHidePoints = !data.mbHidePoints;
}

/**
 * Callback to delete the current model
 * @param data Reference to the ModelGameData class
 */
void DeleteCurrentModel_cb( ModelsGameData & data )
{
  data.DeleteCurrentModel();
}

/**
 * Callback to delete all of the models
 * @param data Reference to the ModelGameData class
 */
void DeleteAllModels_cb( ModelsGameData & data )
{
  data.DeleteAllModels();
}


/**
 * Callback to add a new model
 * @param data Reference to the ModelGameData class
 */
void AddModel_cb( ModelsGameData & data )
{
  data.mDisplayState = ModelsGameData::BrowserState;
}


////////////////////////// end of callbacks ///////////////////////////

  
/**
 * ModelControls constructor
 * @param data reference to the game data struct
 */
ModelControls::ModelControls( ModelsGameData & data)
  : mData( data ),
    mpButton(NULL),
    mControlsTopLeft(10, 100),
    mControlsBottomRight(10, 100),
    mbInitialized(false),
    msHelpTip( "Right click for help" ),
    mnHelpCounter(-1)
{
}


/**
 * Destructor
 */
ModelControls::~ModelControls()
{
  for( size_t ii = 0; ii < mvButtons.size(); ++ii )  {
    delete mvButtons[ii];
  }

}


/**
 * Initialise the controls
 */
void ModelControls::Init( )
{
  // initialize the control buttons
  MGButton * pButton = NULL;
  int x = mControlsTopLeft.x, y = mControlsTopLeft.y;
  ImageRef irNextPos( x, y );

  //first row
  pButton = new MGButton( "Move up", true );
  pButton->Init( irNextPos, ImageRef(40,40), MoveUp_cb, "ARData/Overlays/up.png", "ARData/Overlays/up_a.png");
  mvButtons.push_back( pButton );
  
  irNextPos.x = pButton->GetPosition().x + pButton->GetSize().x;
  irNextPos.y = pButton->GetPosition().y;

  pButton = new MGButton( "Move forwards", true );
  pButton->Init( irNextPos, ImageRef(40,40), MoveFwds_cb, "ARData/Overlays/forward.png", "ARData/Overlays/forward_a.png");
  mvButtons.push_back( pButton );

  irNextPos.x = pButton->GetPosition().x + pButton->GetSize().x;
  irNextPos.y = pButton->GetPosition().y;
  
  pButton = new MGButton( "Move down", true );
  pButton->Init( irNextPos, ImageRef(40,40), MoveDown_cb, "ARData/Overlays/down.png", "ARData/Overlays/down_a.png");
  mvButtons.push_back( pButton );

  //second row
  irNextPos.x = x;
  irNextPos.y = pButton->GetPosition().y + pButton->GetSize().y;

  pButton = new MGButton( "Move left", true );
  pButton->Init(irNextPos, ImageRef(40,40), MoveLeft_cb, "ARData/Overlays/left.png", "ARData/Overlays/left_a.png");
  mvButtons.push_back( pButton );
  
  irNextPos.x = pButton->GetPosition().x + pButton->GetSize().x;
  irNextPos.y = pButton->GetPosition().y;

  pButton = new MGButton( "Reset position" );
  pButton->Init( irNextPos, ImageRef(40,40), ResetModel_cb, "ARData/Overlays/reset.png", "ARData/Overlays/reset_a.png");
  mvButtons.push_back( pButton );

  irNextPos.x = pButton->GetPosition().x + pButton->GetSize().x;
  irNextPos.y = pButton->GetPosition().y;
  
  pButton = new MGButton( "Move right", true );
  pButton->Init( irNextPos, ImageRef(40,40), MoveRight_cb, "ARData/Overlays/right.png", "ARData/Overlays/right_a.png");
  mvButtons.push_back( pButton );

  //third row
  irNextPos.x = x;
  irNextPos.y = pButton->GetPosition().y + pButton->GetSize().y;;

  pButton = new MGButton( "Increase scale", true );
  pButton->Init(irNextPos, ImageRef(40,40), ScaleIncrease_cb, "ARData/Overlays/scale_plus.png", "ARData/Overlays/scale_plus_a.png");
  mvButtons.push_back( pButton );
  
  irNextPos.x = pButton->GetPosition().x + pButton->GetSize().x;
  irNextPos.y = pButton->GetPosition().y;

  pButton = new MGButton( "Move backwards", true );
  pButton->Init( irNextPos, ImageRef(40,40), MoveBack_cb, "ARData/Overlays/back.png", "ARData/Overlays/back_a.png");
  mvButtons.push_back( pButton );

  irNextPos.x = pButton->GetPosition().x + pButton->GetSize().x;
  irNextPos.y = pButton->GetPosition().y;
  
  pButton = new MGButton( "Decrease scale", true );
  pButton->Init( irNextPos, ImageRef(40,40), ScaleDecrease_cb, "ARData/Overlays/scale_minus.png", "ARData/Overlays/scale_minus_a.png");
  mvButtons.push_back( pButton );

  //fourth row
  irNextPos.x = x;
  irNextPos.y = pButton->GetPosition().y + pButton->GetSize().y;;

  pButton = new MGButton( "Roll Increase", true );
  pButton->Init(irNextPos, ImageRef(40,40), RollIncrease_cb, "ARData/Overlays/roll_plus.png", "ARData/Overlays/roll_plus_a.png");
  mvButtons.push_back( pButton );
  
  irNextPos.x = pButton->GetPosition().x + pButton->GetSize().x;
  irNextPos.y = pButton->GetPosition().y;

  pButton = new MGButton( "Pitch Increase", true );
  pButton->Init( irNextPos, ImageRef(40,40), PitchIncrease_cb, "ARData/Overlays/pitch_plus.png", "ARData/Overlays/pitch_plus_a.png");
  mvButtons.push_back( pButton );

  irNextPos.x = pButton->GetPosition().x + pButton->GetSize().x;
  irNextPos.y = pButton->GetPosition().y;
  
  pButton = new MGButton( "Yaw Increase", true );
  pButton->Init( irNextPos, ImageRef(40,40), YawIncrease_cb, "ARData/Overlays/yaw_plus.png", "ARData/Overlays/yaw_plus_a.png");
  mvButtons.push_back( pButton );

  //fifth row
  irNextPos.x = x;
  irNextPos.y = pButton->GetPosition().y + pButton->GetSize().y;

  pButton = new MGButton( "Roll Decrease", true );
  pButton->Init(irNextPos, ImageRef(40,40), RollDecrease_cb, "ARData/Overlays/roll_minus.png", "ARData/Overlays/roll_minus_a.png");
  mvButtons.push_back( pButton );
  
  irNextPos.x = pButton->GetPosition().x + pButton->GetSize().x;
  irNextPos.y = pButton->GetPosition().y;

  pButton = new MGButton( "Pitch Decrease", true );
  pButton->Init( irNextPos, ImageRef(40,40), PitchDecrease_cb, "ARData/Overlays/pitch_minus.png", "ARData/Overlays/pitch_minus_a.png");
  mvButtons.push_back( pButton );

  irNextPos.x = pButton->GetPosition().x + pButton->GetSize().x;
  irNextPos.y = pButton->GetPosition().y;
  
  pButton = new MGButton( "Yaw Decrease", true );
  pButton->Init( irNextPos, ImageRef(40,40), YawDecrease_cb, "ARData/Overlays/yaw_minus.png", "ARData/Overlays/yaw_minus_a.png");
  mvButtons.push_back( pButton );
  
  //sixth row
  irNextPos.x = x;
  irNextPos.y = pButton->GetPosition().y + pButton->GetSize().y;;

  pButton = new MGButton( "Show/hide map points", false, true );
  pButton->Init(irNextPos, ImageRef(40,40), TogglePoints_cb, "ARData/Overlays/points.png", "ARData/Overlays/points_a.png");
  mvButtons.push_back( pButton );
  
  irNextPos.x = pButton->GetPosition().x + pButton->GetSize().x;
  irNextPos.y = pButton->GetPosition().y;

  pButton = new MGButton( "Snap to points", false, true );
  pButton->Init( irNextPos, ImageRef(40,40), ToggleSnapTo_cb, "ARData/Overlays/snap.png", "ARData/Overlays/snap_a.png");
  mvButtons.push_back( pButton );

  irNextPos.x = pButton->GetPosition().x + pButton->GetSize().x;
  irNextPos.y = pButton->GetPosition().y;
  
  pButton = new MGButton( "Hide AR", false, true );
  pButton->Init( irNextPos, ImageRef(40,40), ToggleAR_cb, "ARData/Overlays/hidear.png", "ARData/Overlays/hidear_a.png");
  mvButtons.push_back( pButton );

  
  //seventh row
  irNextPos.x = x;
  irNextPos.y = pButton->GetPosition().y + pButton->GetSize().y;;
    
  pButton = new MGButton( "Add model" );
  pButton->Init( irNextPos, ImageRef(40,40), AddModel_cb, "ARData/Overlays/add.png", "ARData/Overlays/add_a.png");
  mvButtons.push_back( pButton );

  irNextPos.x = pButton->GetPosition().x + pButton->GetSize().x;
  irNextPos.y = pButton->GetPosition().y;
  
  pButton = new MGButton( "Delete model" );
  pButton->Init( irNextPos, ImageRef(40,40), DeleteCurrentModel_cb, "ARData/Overlays/delete.png", "ARData/Overlays/delete_a.png");
  mvButtons.push_back( pButton );

  irNextPos.x = pButton->GetPosition().x + pButton->GetSize().x;
  irNextPos.y = pButton->GetPosition().y;
  
  pButton = new MGButton( "Delete all models" );
  pButton->Init( irNextPos, ImageRef(40,40), DeleteAllModels_cb, "ARData/Overlays/delete_all.png", "ARData/Overlays/delete_all.png");
  mvButtons.push_back( pButton );

  // eighth  row
  irNextPos.x = x;
  irNextPos.y = pButton->GetPosition().y + pButton->GetSize().y;;
    
  pButton = new MGButton( "Previous model" );
  pButton->Init( irNextPos, ImageRef(40,40), PrevModel_cb, "ARData/Overlays/prev.png", "ARData/Overlays/prev_a.png");
  mvButtons.push_back( pButton );

  irNextPos.x = pButton->GetPosition().x + pButton->GetSize().x;
  irNextPos.y = pButton->GetPosition().y;
  
  pButton = new MGButton( "Next model" );
  pButton->Init( irNextPos, ImageRef(40,40), NextModel_cb, "ARData/Overlays/next.png", "ARData/Overlays/next_a.png");
  mvButtons.push_back( pButton );

  irNextPos.x = pButton->GetPosition().x + pButton->GetSize().x;
  irNextPos.y = pButton->GetPosition().y;
  
  pButton = new MGButton( "Hide controls", false, true );
  pButton->Init( irNextPos, ImageRef(40,40), ToggleControls_cb, "ARData/Overlays/hide.png", "ARData/Overlays/hide.png");
  mvButtons.push_back( pButton );
  
  mControlsBottomRight = irNextPos + pButton->GetSize();
  mirHelpLocation = ImageRef( mControlsTopLeft.x, mControlsBottomRight.y + 20 );

  mbInitialized = true;
}


/**
 *  Draw the controls 
 */
void ModelControls::Draw( const GLWindow2 &glWindow )
{
  if( !mbInitialized ) {
    Init();
  }

  if( mnHelpCounter != -1 ) {
    mnHelpCounter++;
  }
  
  if(mData.mbHideControls) {
    return;
  }

  // draw the buttons
  for( size_t ii = 0; ii < mvButtons.size(); ii++ )  {
    mvButtons[ii]->Draw();
  }

  // draw the help text
  if( mnHelpCounter < 150 ) // display for ~5 seconds
  {
    glColor3f(1.0,1.0,0.0);
    glWindow.PrintString( mirHelpLocation, msHelpTip );
  }
}


/**
 * Handle the user clicking on the controls
 * @param v2VidCoords The location of the click
 * @param nButton the button pressed
 * @return 
 */
bool ModelControls::HandleClick(Vector<2> v2VidCoords, int nButton)
{
  mpButton = NULL;
  
  // handle hidden controls
  if( mData.mbHideControls ) {
    if( nButton == CVD::GLWindow::BUTTON_RIGHT ) {
      mData.mbHideControls = false;
      return true;
    }
    return false;
  }

  // check button bounds
  if( v2VidCoords[0] < mControlsTopLeft.x || v2VidCoords[1] < mControlsTopLeft.y ||
      v2VidCoords[0] > mControlsBottomRight.x || v2VidCoords[1] > mControlsBottomRight.y )  {
    return false;
  }

  bool bClicked = false;
  for( size_t ii = 0; ii < mvButtons.size(); ++ii )
  {
    bClicked = mvButtons[ii]->IsMouseOver( v2VidCoords );

    if( bClicked )
    {
      if( nButton == CVD::GLWindow::BUTTON_RIGHT )
      {
        mnHelpCounter = 0; //start the counter
        msHelpTip = mvButtons[ii]->GetName();
      }
      else  {
        if( mvButtons[ii]->IsRepeatEnabled() ) {
          mpButton = mvButtons[ii];
        }
        else {
          mpButton = NULL;
        }
        
        mvButtons[ii]->Action( mData );
      }
      
      return true;
    }
  }

  return false;
}



/**
 * The user has pressed a button and held it down, so keep updating it.
 */
void ModelControls::HandlePressAndHold()
{
  if( mpButton ) {
    mpButton->Action( mData );
  }
}


}
