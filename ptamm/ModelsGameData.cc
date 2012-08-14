// Copyright 2009 Isis Innovation Limited

#include "ModelsGameData.h"
#include "tinyxml.h"
#include <iostream>


namespace PTAMM {

using namespace std;
  
/**
 * ModelsGameData constructor
 */
ModelsGameData::ModelsGameData( string sName ) :
  mbHideAR(false),
  mbSnapTo(false),
  mbHideControls(false),
  mbHidePoints(true),
  mDisplayState(BrowserState),
  mnModelIdx(-1),
  mdVersion(1.0),
  msName( sName )
{
}


/**
 * ModelsGameData destructor
 */
ModelsGameData::~ModelsGameData()
{
  DeleteAllModels();
}


/**
 * Return the current model
 * @return Pointer to the current model. NULL if none.
 */
Model3DS * ModelsGameData::CurrentModel()
{
  if( mnModelIdx < 0 || mnModelIdx >= static_cast<int>(mvModels.size()) ) {
    return NULL;
  }

  return mvModels[mnModelIdx];
}


/**
 * Advance to the next model in the list
 */
void ModelsGameData::NextModel()
{
  if( mvModels.empty() ) {
    mnModelIdx = -1;
    return;
  }

  mnModelIdx++;
  
  if( (mnModelIdx < 0) || (mnModelIdx >= static_cast<int>(mvModels.size()) ) )
  {
    mnModelIdx = 0;
  }

}



/**
 * Advance to the previous model in the list
 */
void ModelsGameData::PrevModel()
{
  if( mvModels.empty() ) {
    mnModelIdx = -1;
    return;
  }

  mnModelIdx--;
  
  if( (mnModelIdx < 0) || ( mnModelIdx >= static_cast<int>(mvModels.size()) ) ) {
    mnModelIdx = static_cast<int>(mvModels.size()) - 1;
  }

}



/**
 * Add a new model
 * @param sFileName Path and name of the model file
 * @param sModelDir The directory containing the model
 * @param sName The name of the model
 * @return success
 */
bool ModelsGameData::AddModel( std::string sModelDir, std::string sFileName, std::string sName, TooN::Vector<3> v3Rotation )
{
  Model3DS * pModel = new Model3DS();

  if( pModel->Load( sModelDir, sFileName, v3Rotation ) )
  {
    pModel->Name() = sName;
  
    mvModels.push_back(pModel);
    mnModelIdx = static_cast<int>(mvModels.size()) - 1;

    mDisplayState = ControlState;
    
    std::cout << "Loaded model " << pModel->Name() << std::endl;
    return true;
  }
  else
  {
    delete pModel;
    return false;
  }
}

/**
 * Delete the current model
 */
void ModelsGameData::DeleteCurrentModel()
{
    if( mvModels.empty() || mnModelIdx < 0 || mnModelIdx >= static_cast<int>(mvModels.size()) ) {
    return;
  }

  delete mvModels[mnModelIdx];
  mvModels.erase( mvModels.begin()+mnModelIdx );

  if( mvModels.empty() ) {
    mnModelIdx = -1;
    mDisplayState = BrowserState;
  }
  else {
    mnModelIdx = 0;
  }
}



/**
 * Delete all of the models
 */
void ModelsGameData::DeleteAllModels()
{
  mnModelIdx = -1;
  
  for( size_t ii = 0; ii < mvModels.size(); ++ii )
  {
    delete mvModels[ii];
  }
  
  mvModels.clear();
  mDisplayState = BrowserState;
}


/**
 * Reset the game data.
 * Clears everything out and resets to the initial state.
 */
void ModelsGameData::Reset()
{
  DeleteAllModels();
  mbHideAR = false;
  mbSnapTo = false;
  mbHideControls = false;
  mbHidePoints = true;
  mDisplayState = BrowserState;
  mnModelIdx = -1;
}


/**
 * Save the game state to disk
 * @param sDataFileName The file name to save to
 */
void ModelsGameData::Save( std::string sDataFileName )
{
  TiXmlDocument xmlDoc;     //XML file

  //open the map file for writing
  TiXmlDeclaration* decl = new TiXmlDeclaration( "1.0", "", "" );
  xmlDoc.LinkEndChild( decl );

  TiXmlElement * rootNode = new TiXmlElement( msName );
  xmlDoc.LinkEndChild( rootNode );
  rootNode->SetDoubleAttribute("version", mdVersion);

  TiXmlElement * modelsNode = new TiXmlElement( "Models" );
  rootNode->LinkEndChild( modelsNode );
  modelsNode->SetAttribute( "size", static_cast<int>(mvModels.size()) );


  for( size_t ii = 0; ii < mvModels.size(); ++ii )
  {
    mvModels[ii]->Save( modelsNode );
  }

  xmlDoc.SaveFile(sDataFileName);
  
}

/**
 * Load a saved game
 * @param sDataFileName the file to load
 */
void ModelsGameData::Load( std::string sDataFileName )
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

  string sID( msName );
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


  int nSize = -1;
  TiXmlHandle pNode = hRoot.FirstChild( "Models" );
  pNode.ToElement()->QueryIntAttribute("size", &nSize);

  for(TiXmlElement* pElem = pNode.FirstChild().Element();
      pElem != NULL;
      pElem = pElem->NextSiblingElement() )
  {
    //Load a model - this is a delayed load so no GL calls are made.
    //In this function as it will be called from the MapSerialization thread
    Model3DS * pModel = new Model3DS();
    if( pModel->Load( pElem ) ) {
      mvModels.push_back(pModel);
    }
  }
  
  mnModelIdx = static_cast<int>(mvModels.size()) - 1;
  mDisplayState = ControlState;

  if( static_cast<int>(mvModels.size() ) != nSize ) {
    cerr << "Warning: Loaded the wrong number of models. " << mvModels.size()
        << " instead of " << nSize << "." << endl;
  }
  
  
}



}

