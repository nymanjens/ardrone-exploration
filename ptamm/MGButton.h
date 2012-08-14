// Copyright 2009 Isis Innovation Limited

#ifndef MGBUTTON_H
#define MGBUTTON_H

#include <iostream>
#include <vector>
#include "OpenGL.h"

#include <TooN/TooN.h>

#include "GLWindow2.h"
#include "ModelsGameData.h"

namespace PTAMM {

using namespace TooN;

class ModelControls;

/**
 * A button for the model controls
 */
class MGButton {
  
  public:
    MGButton( std::string sName, bool bEnableRepeat = false, bool bToggle = false );
    ~MGButton() {}

    // The form of the callback function
    typedef void (*ActionCallback)( ModelsGameData & data );

    void Init( CVD::ImageRef irPosition, CVD::ImageRef irSize, ActionCallback action,
               std::string sIconFileName = "", std::string sActiveIconFileName = "" );
    void Action( ModelsGameData & data );
    bool IsMouseOver( Vector<2> v2VidCoords );
    void Draw();
    
    CVD::ImageRef GetSize() { return mirSize; }           /// The size of the button
    CVD::ImageRef GetPosition() { return mirPosition; }   /// The position of the button
    std::string GetName() { return msName; }              /// The name of the button
    bool IsToggle() { return mbToggle; }                  /// Is it a toggle button
    bool IsRepeatEnabled() { return mbEnableRepeat; }     /// Is repeat action enabled

  public:
    Vector<4> v4BorderColor;                              // Colours for the button
    Vector<4> v4BackColor;
    Vector<4> v4FontColor;
    Vector<4> v4ActiveColor;

  private:
    void _DrawLineBox();
    void _DrawFillBox();

  private:
    std::string msName;                                   // Name of the button
    CVD::ImageRef mirPosition;                            // The top left corner of the button
    CVD::ImageRef mirSize;                                // The size of the button
    bool mbActive;                                        // Is the button activated

    CVD::Image< CVD::Rgba<CVD::byte> > mimIcon;           // The button's icon
    CVD::Image< CVD::Rgba<CVD::byte> > mimIconActive;     // Its active icon
    bool mbIcons;                                         // Use icons?

    ActionCallback mpActionCallback;                      // The callback function

    int mnActiveCounter;                                  // Number of frames left to derw as active
    const bool mbToggle;                                  // Is it a toggle button?
    const bool mbEnableRepeat;                            // enable repeat actions for press and hold
};

}

#endif

