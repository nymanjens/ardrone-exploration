// Copyright 2009 Isis Innovation Limited

#include "ModelBrowser.h"
#include "tinyxml.h"

#include <cvd/gl_helpers.h>
#include <cvd/image_io.h>
#include <gvars3/instances.h>

namespace PTAMM {

using namespace TooN;
using namespace CVD;
using namespace std;
using namespace GVars3;


/**
 * Browser Item constructor
 */
BrowserItem::BrowserItem()
  : irSize( BROWSER_ITEM_W, BROWSER_FONT_H ),
    mbIcon(false)
{
  
}

/**
 * Draw a browser item
 * @param glWin the glwindow
 */
void BrowserItem::Draw( const GLWindow2 & glWin )
{
  ImageRef irTextPos(irPosition.x + mimIcon.size().x + BROWSER_FONT_X,
                     irPosition.y + 0.5*(mimIcon.size().y + BROWSER_FONT_H) );
  
  if( mbIcon )
  {
    glRasterPos(irPosition);
    glDrawPixels( mimIcon );
  }
  else {
    irTextPos.y = irPosition.y + BROWSER_FONT_H;
  }
    
  glWin.PrintString( irTextPos, sName );

}


/**
 * Try and load an icon file.
 * Only files that are BROWSER_ICON_W x BROWSER_ICON_H will be accepted
 * @param sIconFile The path to the icon file
 * @return success
 */
bool BrowserItem::LoadIcon(std::string sIconFile)
{
  if( !sIconFile.empty() )
  {
    try {
      CVD::img_load( mimIcon, sIconFile );

      if( mimIcon.size() == ImageRef(BROWSER_ICON_W, BROWSER_ICON_H) ) {
        mbIcon = true;
        irSize.y = BROWSER_ICON_H;
      }
      else {
        cerr << "ERROR Image " << sIconFile << " is not " << BROWSER_ICON_W << "x" << BROWSER_ICON_H << " pixels" << endl;
        mbIcon = false;
      }
    }
    catch(CVD::Exceptions::All err) {
      cerr << "Failed to load category icon " << sIconFile << ": " << err.what << endl;
      mbIcon = false;
    }
  }

  return mbIcon;
}


/**
 * Is the mouse over this item?
 * @param v2VidCoords the mouse position
 * @return true if over
 */
bool BrowserItem::IsMouseOver( Vector<2> v2VidCoords )
{
  if( (v2VidCoords[0] > irPosition.x) && ( v2VidCoords[0] <= irPosition.x + irSize.x ) &&
      (v2VidCoords[1] > irPosition.y) && ( v2VidCoords[1] <= irPosition.y + irSize.y ) ) {
    return true;
  }

  return false;
}


///////////////////////// Page functions //////////////////////////


/**
 * Page destructor
 */
Page::~Page()
{
  for( size_t ii = 0; ii < vItems.size(); ii++ )  {
    delete vItems[ii];
  }
}


///////////////////////// Pages functions //////////////////////////

/**
 * Pages destructor
 */
Pages::~Pages()
{
  for( size_t ii = 0; ii < vPages.size(); ii++ )  {
    delete vPages[ii];
  }
}

  
/////////////////////// ModelBrowser Code ///////////////////////

/**
 * Model Browser Constructor
 */
ModelBrowser::ModelBrowser()
  : mpSelectedModelData(NULL),
    mPages(NULL),
    mirPosition(BROWSER_X,BROWSER_Y),
    mbInitialized(false)
{
  Reset();
}


/**
 * Destructor
 */
ModelBrowser::~ModelBrowser()
{
  
}


/**
 * Reset the model browser
 */
void ModelBrowser::Reset()
{
  mpCurrentPageSet = &mPages;
}


/**
 * Initialize the model browser
 * @param irPosition the location of the browser
 * @param  sBrowserImageFileName the image file for the browser
 * @return success
 */
bool ModelBrowser::Init( CVD::ImageRef irPosition, std::string sBrowserImageFileName)
{
  if( mbInitialized ) {
    return true;
  }
  
  mirPosition = irPosition;
  
  try {
    CVD::img_load( mimBrowser, sBrowserImageFileName );
  }
  catch(CVD::Exceptions::All err) {
    cerr << "Failed to load icon image " << sBrowserImageFileName << ": " << err.what << endl;
    return false;
  }

  // Read the database
  if( !_ParseModelDB() ) {
    return false;
  }

  _SetupCategoryPages( mPages, mvModelsDB );

  mbInitialized = true;
  return true;
}


/**
 * Draw the model browser
 * @param glWindow the gl window
 */
void ModelBrowser::Draw(  const GLWindow2 &glWindow )
{
  if( !mbInitialized) {
    return;
  }
  
  glRasterPos( mirPosition );
  glDrawPixels( mimBrowser );

  ImageRef irDrawPos( mirPosition.x + BROWSER_ITEM_X, mirPosition.y + BROWSER_ITEM_Y);

  const std::vector< BrowserItem* > & vItems =  mpCurrentPageSet->CurrentPage().vItems;
  for( size_t ii = 0; ii < vItems.size(); ++ii )
  {
    vItems[ii]->Draw( glWindow );
  }
}


/**
 * Handle the user clicking on the model browser
 * @param v2VidCoords the window coords
 * @return user clicked on something
 */
ModelBrowser::CLICK_STATUS ModelBrowser::HandleClick(Vector<2> v2VidCoords)
{
  //check general bounds
  if(v2VidCoords[0] < mirPosition.x ||
      v2VidCoords[0] > (mirPosition.x + mimBrowser.size().x) ||
      v2VidCoords[1] < mirPosition.y ||
      v2VidCoords[1] > (mirPosition.y + mimBrowser.size().y) ) {
    return NONE;
  }

  //check button bounds
  if(v2VidCoords[1] > mirPosition.y + BROWSER_BUTTON_Y &&
      v2VidCoords[1] < (mirPosition.y + BROWSER_BUTTON_Y + BROWSER_BUTTON_H))
  {
    if(v2VidCoords[0] > mirPosition.x + BROWSER_BUTTON_X &&
        v2VidCoords[0] < (mirPosition.x + BROWSER_BUTTON_X + BROWSER_BUTTON_W) ) //back
    {
      Pages * pp = mpCurrentPageSet->Parent();
      if( pp != NULL ) {
        mpCurrentPageSet = pp;
      }
      return OK;
    }
    else if(v2VidCoords[0] > (mirPosition.x + BROWSER_BUTTON_X + BROWSER_BUTTON_W + BROWSER_BUTTON_SEP) &&
            v2VidCoords[0] < (mirPosition.x + BROWSER_BUTTON_X + 2*BROWSER_BUTTON_W + BROWSER_BUTTON_SEP)) //up
    {
      mpCurrentPageSet->PrevPage();
      return OK;
    }
    else if(v2VidCoords[0] > (mirPosition.x + BROWSER_BUTTON_X + 2*BROWSER_BUTTON_W + 2*BROWSER_BUTTON_SEP) &&
            v2VidCoords[0] < (mirPosition.x + BROWSER_BUTTON_X + 3*BROWSER_BUTTON_W + 2*BROWSER_BUTTON_SEP)) //down
    {
      mpCurrentPageSet->NextPage();
      return OK;
    }
    else if(v2VidCoords[0] > (mirPosition.x + BROWSER_BUTTON_X + 3*BROWSER_BUTTON_W + 3*BROWSER_BUTTON_SEP) &&
            v2VidCoords[0] < (mirPosition.x + BROWSER_BUTTON_X + 4*BROWSER_BUTTON_W + 3*BROWSER_BUTTON_SEP)) //cancel
    {
      return CANCEL;
    }
  }
  else if(v2VidCoords[0] > mirPosition.x + BROWSER_ITEM_X &&
          v2VidCoords[0] < (mirPosition.x + BROWSER_ITEM_X + BROWSER_ITEM_W))   //check item bounds
  {
     const std::vector< BrowserItem* > & vItems = mpCurrentPageSet->CurrentPage().vItems;
    
    for( size_t ii = 0; ii < vItems.size(); ++ii )
    {
      if( vItems[ii]->IsMouseOver( v2VidCoords ) )
      {
        ModelItem *pMI = dynamic_cast< ModelItem* >( vItems[ii] );
        if( pMI )
        {
          cout << "Loading model ... " << endl;
          cout << pMI->data.sName << endl;
          cout << pMI->data.sModelFile << endl;
          cout << pMI->data.sLocation << endl;
          mpSelectedModelData = &pMI->data;
          return MODEL;
        }
          
        CategoryItem *pCI = dynamic_cast< CategoryItem* >( vItems[ii] );
        if( pCI )
        {
          mpCurrentPageSet = &pCI->pages;
          return OK;
        }
        
      }

    }
  }
    
  return NONE;
}


/**
 * Read in the model database and initialize the browser components.
 * @return success
 */
bool ModelBrowser::_ParseModelDB()
{
  TiXmlDocument mXMLDoc;                                 //XML file
  //string sModelDBFileName = "ARData/Models/models.xml";
  string sModelDBFileName = GV3::get<string>("ModelsGame.XMLFile", "ARData/Models/models.xml", SILENT);
  
  size_t found;
  found = sModelDBFileName.find_last_of("/\\");
  string sPath = "";

  if( found != string::npos ) {
    sPath = sModelDBFileName.substr(0,found + 1);
  }

  //load the XML file
  if( !mXMLDoc.LoadFile( sModelDBFileName ) )  {
    cerr << "Failed to load " << sModelDBFileName << ". Aborting." << endl;
    return false;
  }
  
  TiXmlHandle hDoc(&mXMLDoc);
  TiXmlElement* pElem;
  TiXmlHandle hRoot(0);

  pElem = hDoc.FirstChildElement().Element();
  // should always have a valid root but handle gracefully if it does not
  if (!pElem)  {
    cerr << "No root handle in XML file " << sModelDBFileName << endl;
    return false;
  }

  string sID( MODEL_DB_ID );
  string sVersion( MODEL_DB_VERSION );
  string sFileVersion = pElem->Attribute("version");
  
  if( ( sID.compare( pElem->Value() ) != 0 ) &&
      ( sVersion.compare( sFileVersion ) != 0 ) )
  {
    cerr << "Invalid XML file. Need a version " << sVersion << " " << sID
         << " XML file. Not a version " << sFileVersion << " " << pElem->Value() << " file." << endl;
    return false;
  }

  // save this for later
  hRoot = TiXmlHandle(pElem);


  //read in each category
  for(TiXmlElement* pCatElem = hRoot.FirstChild( "Category" ).Element();
      pCatElem != NULL;
      pCatElem = pCatElem->NextSiblingElement() )
  {
    CategoryData catData;
    catData.sName = pCatElem->Attribute("name");

    string sIconFile = pCatElem->Attribute("image");
    if( !sIconFile.empty() ) {
      catData.sIconPath = sPath + sIconFile;
    }

    //read in each model in the category
    for(TiXmlElement* pModelElem = pCatElem->FirstChildElement( "Model" );
      pModelElem != NULL;
      pModelElem = pModelElem->NextSiblingElement() )
    {
      ModelData modelData;
      modelData.sName = pModelElem->Attribute( "name" );
      modelData.sLocation = sPath + pModelElem->Attribute( "dir" );
      modelData.sModelFile = pModelElem->Attribute( "file" );

      string sIconFile = pModelElem->Attribute("image");
      if( !sIconFile.empty() ) {
        modelData.sIconPath = modelData.sLocation + "/" + sIconFile;  ///@TODO naughty - handle path creation better
      }

      pModelElem->QueryDoubleAttribute( "roll",  &modelData.v3Rotation[0] );
      pModelElem->QueryDoubleAttribute( "pitch", &modelData.v3Rotation[1] );
      pModelElem->QueryDoubleAttribute( "yaw",   &modelData.v3Rotation[2] );

      modelData.v3Rotation *= M_PI/180.0;

      catData.vModels.push_back( modelData );
    }

    mvModelsDB.push_back( catData );
    
  }

  return true;
}




/**
 * Set up the model pages for the browser
 * @param pages each page is added to this
 * @param vModels the model info loaded from the xml file
 */
void ModelBrowser::_SetupModelPages( Pages & pages, const std::vector< ModelData > & vModels )
{
  //top left location of the first item
  const ImageRef irTopDrawPos( mirPosition.x + BROWSER_ITEM_X, mirPosition.y + BROWSER_ITEM_Y);
  const int nMaxY = mirPosition.y + BROWSER_ITEM_Y + BROWSER_TOTAL_ITEM_HEIGHT;
  ImageRef irDrawPos = irTopDrawPos;

  Page * pPage = new Page();

  for( size_t ii = 0; ii < vModels.size(); ++ii )
  {
    ModelItem * pMI = new ModelItem( vModels[ii] );
    pMI->sName = vModels[ii].sName;
    pMI->LoadIcon( vModels[ii].sIconPath );

    if( (irDrawPos.y + pMI->irSize.y) > nMaxY )
    {
      // new page
      pages.AddPage( pPage );
      pPage = new Page();
      irDrawPos = irTopDrawPos;
    }

    pMI->irPosition = irDrawPos;
    irDrawPos.y += pMI->irSize.y + BROWSER_ITEM_SEP;

    pPage->vItems.push_back( pMI );

  }

  pages.AddPage( pPage );
}


/**
 * Set up the pages for each category
 * @param pages each page for a catefory gets added to this
 * @param vCategories the list of categories loaded from the xml file.
 */
void ModelBrowser::_SetupCategoryPages( Pages & pages, const std::vector< CategoryData > & vCategories )
{
  //top left location of the first item
  const ImageRef irTopDrawPos( mirPosition.x + BROWSER_ITEM_X, mirPosition.y + BROWSER_ITEM_Y);
  const int nMaxY = mirPosition.y + BROWSER_ITEM_Y + BROWSER_TOTAL_ITEM_HEIGHT;
  ImageRef irDrawPos = irTopDrawPos;

  Page * pPage = new Page();

  for( size_t ii = 0; ii < vCategories.size(); ++ii )
  {
    CategoryItem * pCI = new CategoryItem(&mPages);
    pCI->sName = vCategories[ii].sName;
    pCI->LoadIcon( vCategories[ii].sIconPath );

    if( (irDrawPos.y + pCI->irSize.y) > nMaxY )
    {
      // new page
      pages.AddPage( pPage );
      pPage = new Page();
      irDrawPos = irTopDrawPos;
    }

    pCI->irPosition = irDrawPos;
    irDrawPos.y += pCI->irSize.y + BROWSER_ITEM_SEP;

    _SetupModelPages( pCI->pages, mvModelsDB[ii].vModels );

    pPage->vItems.push_back( pCI );

  }

  pages.AddPage( pPage );

}


}
