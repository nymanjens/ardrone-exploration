// Copyright 2009 Isis Innovation Limited
//
// C++ Interface: ModelsGameData
//
// Description: Holds all of the game data for the ModelsGame
//
//
// Author: Robert Castle <bob@robots.ox.ac.uk>, (C) 2009
//
//
#ifndef PTAMMMODELSGAMEDATA_H
#define PTAMMMODELSGAMEDATA_H

#include <vector>
#include <string>

#include "Model3ds.h"

namespace PTAMM {

/**
 * This class holds all of the relevant data for the ModelsGame,
 * such as the list of models and various bools for game state information.
 * 
 * @author Robert Castle <bob@robots.ox.ac.uk>
*/
class ModelsGameData{
  public:
    ModelsGameData( std::string sName );
    ~ModelsGameData();

    Model3DS * CurrentModel();
    void NextModel();
    void PrevModel();
    bool AddModel( std::string sModelDir, std::string sFileName, std::string sName, TooN::Vector<3> v3Rotation );
    
    void DeleteCurrentModel();
    void DeleteAllModels();
    void Reset();
    size_t NumModels() { return mvModels.size(); }
    const std::vector< Model3DS* > & Models() { return mvModels; }

    void Save( std::string sDataFileName );
    void Load( std::string sDataFileName );

    bool mbHideAR;                              // Hide the AR
    bool mbSnapTo;                              // Snap to the map points
    bool mbHideControls;                        // Hide the controls
    bool mbHidePoints;                          // Hide the map points

    enum DisplayState {BrowserState, ControlState, HiddenState};
    DisplayState mDisplayState;                 // Game display state

  private:
    std::vector< Model3DS* > mvModels;          // The list of models
    int mnModelIdx;                             // The current model index
    const double mdVersion;                     // Current game version
    const std::string msName;                   // Name of the game

};

}

#endif
