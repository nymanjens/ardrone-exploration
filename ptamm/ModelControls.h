// Copyright 2009 Isis Innovation Limited
//
// C++ Interface: ModelControls
//
// Description: ModelControls user interface. Displays the control buttons,
// and handles user interaction
//
// Author: Robert Castle <bob@robots.ox.ac.uk>, (C) 2009
//
// Copyright: See COPYING file that comes with this distribution
//

#ifndef PTAMMMODELCONTROLS_H
#define PTAMMMODELCONTROLS_H

#include <vector>
#include <string>

#include "GLWindow2.h"
#include "ModelsGameData.h"


namespace PTAMM {

  class MGButton;
  class Model3DS;

  using namespace TooN;


/**
 * The control buttons for placing and orienting models.
 * @author Robert Castle <bob@robots.ox.ac.uk>
 */
class ModelControls{
  public:
    ModelControls( ModelsGameData & data );
    ~ModelControls();

    void Init();
    void Draw( const GLWindow2 &glWindow );
    bool HandleClick(Vector<2> v2VidCoords, int nButton);
    void HandlePressAndHold();

  private:
    ModelsGameData & mData;                   // Reference to the game data object
    std::vector< MGButton* > mvButtons;       // The buttons
    MGButton * mpButton;                      // The button that is currently being pressed
    
    CVD::ImageRef mControlsTopLeft;           // The location of the top left of the control panel
    CVD::ImageRef mControlsBottomRight;       // The location of the bottom right of the control panel
    
    bool mbInitialized;                       // Has the panel been initialized

    std::string msHelpTip;                    // The help tip
    int mnHelpCounter;                        // How many frames has the tip been displayed for
    CVD::ImageRef mirHelpLocation;            // Location of the help tip
};

}

#endif
