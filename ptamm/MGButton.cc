// Copyright 2009 Isis Innovation Limited

#include "MGButton.h"
#include <cvd/gl_helpers.h>
#include <cvd/image_io.h>
#include "Model3ds.h"

namespace PTAMM {

using namespace TooN;
using namespace CVD;
using namespace std;

/**
 * Button constructor.
 * @param sName the name of the button (used when no icons and for tooltip)
 * @param bToggle is the button a toggle button?
 */
MGButton::MGButton( std::string sName, bool bEnableRepeat, bool bToggle )
  : msName(sName),
    mbActive(false),
    mpActionCallback(NULL),
    mnActiveCounter(0),
    mbToggle(bToggle),
    mbEnableRepeat(bEnableRepeat)
{
    v4BorderColor = makeVector( 0.0, 0.4, 0.2, 0.7 );       // Colors for the button
    v4BackColor   = makeVector( 0.0, 0.8, 0.4, 0.5 );
    v4FontColor   = makeVector( 0.0, 0.0, 0.0, 0.0 );
    v4ActiveColor = makeVector( 0.0, 0.4, 0.8, 0.5 );
}


/**
 * Initialize the button.
 * If icon images are used then both normal and active are required for them to display.
 * @param irPosition button location (top left)
 * @param irSize button size in pixels - icon images, if used, overide this
 * @param action callback function pointer
 * @param sIconFileName icon image file (optional)
 * @param sActiveIconFileName active icon image file (optional)
 */
void MGButton::Init( CVD::ImageRef irPosition, CVD::ImageRef irSize, ActionCallback action,
                     std::string sIconFileName, std::string sActiveIconFileName)
{
  mirPosition = irPosition;
  mirSize = irSize;
  bool bOK = false;
  
  try {
    CVD::img_load( mimIcon, sIconFileName );
    bOK = true;
  }
  catch(CVD::Exceptions::All err) {
    cerr << "Failed to load icon image " << sIconFileName << ": " << err.what << endl;
    bOK = false;
  }

  if(bOK)
  {
    try {
      CVD::img_load( mimIconActive, sActiveIconFileName );
      bOK = true;
    }
    catch(CVD::Exceptions::All err) {
      cerr << "Failed to load icon image " << sActiveIconFileName << ": " << err.what << endl;
      bOK = false;
    }
  }

  mpActionCallback = action;
  mbIcons = bOK;
}


/**
 * Is the mouse over this button?
 * @param  v2VidCoords the current mouse location
 * @return over?
 */
bool MGButton::IsMouseOver( Vector<2> v2VidCoords )
{
  if( v2VidCoords[0] < mirPosition.x || v2VidCoords[1] < mirPosition.y ||
      v2VidCoords[0] > mirPosition.x + mirSize.x || v2VidCoords[1] > mirPosition.y + mirSize.y ) {
    return false;
  }

  return true;
}


/**
 * Call the callback for this button
 * @param data reference to the game data
 */
void MGButton::Action( ModelsGameData & data )
{
  if( mbToggle ) {
    mbActive = !mbActive;
  }
  else {
    mnActiveCounter = 10;
    mbActive = true;
  }
  
  if( mpActionCallback) {
    mpActionCallback( data );
  }
}

/**
 * Draw either the icons or a simple box for the button.
 * Only draws icons if both the regular and active icons were loaded
 */
void MGButton::Draw()
{
  //if it is a regular button
  if( !mbToggle ) {
    // and is active, decrement the counter
    if( mbActive && mnActiveCounter > 0 ) {
      mnActiveCounter--;
    }
    else {
      //unset active when counter reaches 0
      mbActive = false;
    }
  }

  if( mbIcons )
  {
    glRasterPos(mirPosition);
    mbActive ? glDrawPixels( mimIconActive ) : glDrawPixels( mimIcon );
  }
  else {
    glColor( v4BorderColor );
    _DrawLineBox();
    mbActive ? glColor( v4ActiveColor ) : glColor( v4BackColor );
    _DrawFillBox();
    glColor( v4FontColor );
  }
}


/**
 * Draw the button outline
 */
void MGButton::_DrawLineBox()
{
  glLineWidth( 1 );
  glEnable(GL_LINE_SMOOTH);
  
  glBegin(GL_LINE_LOOP);
  glVertex2i( mirPosition.x,             mirPosition.y );
  glVertex2i( mirPosition.x,             mirSize.y + mirPosition.y );
  glVertex2i( mirPosition.x + mirSize.x, mirSize.y + mirPosition.y );
  glVertex2i( mirPosition.x + mirSize.x, mirPosition.y );
  glEnd();  
}

/**
 * Draw the button background
 */
void MGButton::_DrawFillBox()
{
  glBegin(GL_QUADS);
  glVertex2i( mirPosition.x,             mirPosition.y );
  glVertex2i( mirPosition.x,             mirSize.y + mirPosition.y );
  glVertex2i( mirPosition.x + mirSize.x, mirSize.y + mirPosition.y );
  glVertex2i( mirPosition.x + mirSize.x, mirPosition.y );
  glEnd();
}


}

