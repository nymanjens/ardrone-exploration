// Copyright 2009 Isis Innovation Limited
//
// C++ Interface: ModelBrowser
//
// Description: A Class to allow the user to browser
// through the list of models using a scrollable list
//
// Author: Robert Castle <bob@robots.ox.ac.uk>, (C) 2009
//
// Copyright: See COPYING file that comes with this distribution
//

#ifndef PTAMMMODELBROWSER_H
#define PTAMMMODELBROWSER_H

#include <iostream>
#include <vector>
#include "OpenGL.h"
#include <TooN/TooN.h>
#include "GLWindow2.h"

namespace PTAMM {

using namespace TooN;

//This defines what version of the model database can be read.
#define MODEL_DB_ID "PTAMM_Model_DB"
#define MODEL_DB_VERSION "1.0"

/// @TODO Some hard coded locations and sizes for the model browser.
/// This should be in a file that is read alongside the png image.

//where to place the browser
#define BROWSER_X 0
#define BROWSER_Y 120

//the size and location of the buttons (up, down, back)
#define BROWSER_BUTTON_W 50
#define BROWSER_BUTTON_H 50
#define BROWSER_BUTTON_X 11
#define BROWSER_BUTTON_Y 5
#define BROWSER_BUTTON_SEP 2

//the size and location of the items (categories and models)
#define BROWSER_ICON_W 64
#define BROWSER_ICON_H 64
#define BROWSER_FONT_X 8
#define BROWSER_FONT_H 14
#define BROWSER_ITEM_X 12
#define BROWSER_ITEM_Y 62
#define BROWSER_ITEM_SEP 3
#define BROWSER_TOTAL_ITEM_HEIGHT 265
#define BROWSER_ITEM_W 190


/**
 * A struct to hold the model data once read from the XML file
 */
struct ModelData {
  std::string sName;                          // Name of the model
  std::string sIconPath;                      // Path to the icon (if there is one)
  std::string sLocation;                      // Location of the file relative to the model db
  std::string sModelFile;                     // The model file
  TooN::Vector<3> v3Rotation;                 // initial rotation of the model to make it plane aligned.
};

/**
 * A struct to hold the category data once read from the XML file
 */
struct CategoryData
{
  std::string   sName;                        // Name of the category
  std::string sIconPath;                      // Path to the icon (if there is one)
  std::vector< ModelData > vModels;           // The models in a category
};


/**
 * An entry in the browser. This is what the user sees for each model entry, and category
 */
class BrowserItem
{
  public:
    BrowserItem();
    virtual ~BrowserItem() {};

    void Draw( const GLWindow2 & glWin );
    bool IsMouseOver( Vector<2> v2VidCoords );
    bool LoadIcon(std::string sIconFile);

    std::string   sName;                           // Name of the item (it will be a model or a category)
    CVD::ImageRef irSize;                          // The size of the item
    CVD::ImageRef irPosition;                      // its location

  private:
    CVD::Image< CVD::Rgba<CVD::byte> > mimIcon;    // icon for the model (optional)
    bool mbIcon;                                   // Icon enabled
};
    

/**
 * A page class. This holds a list of items to display in the browser
 */
class Page
{
  public:
    Page() {};
    ~Page();
    std::vector< BrowserItem* > vItems;            // The list of browser items
};


/**
 * A class to hold a set of page classes.
 * Each category object has a pages object,
 * which holds a set of page objects,
 * which hold the browser item objects.
 */
class Pages
{
  public:
    Pages( Pages * pParent) : nIndex(0), mpParent(pParent) {};
    ~Pages();
    const Page & CurrentPage() { return *vPages[nIndex]; }
    void NextPage() { nIndex++; if( nIndex >= (int)vPages.size() ) nIndex = (int)vPages.size() -1; }
    void PrevPage() { nIndex--; if( nIndex < 0 ) nIndex = 0; }
    Pages * Parent() { return mpParent; }
    void AddPage( Page * p ) { if( p ) vPages.push_back( p ); }

  private:
    std::vector< Page* > vPages;              // A list of page objects
    int nIndex;                               // The current page
    Pages * mpParent;                         // The parent pages object
};
    

/**
 * A model Item, a type of BrowserItem.
 */
class ModelItem : public BrowserItem
{
  public:
    ModelItem( const ModelData & model ) : data(model) {};
    ~ModelItem() {};

    const ModelData & data;                   // The model data
};


/**
 * A Category item, a type of BrowserItem
 */
class CategoryItem : public BrowserItem
{
  public:
    CategoryItem( Pages * pParent ) : pages(pParent) {};
    ~CategoryItem() {};

    Pages pages;                              // Pages object
};



  
/**
 * GUI interface for allowing the user to browse the models
 * @author Robert Castle <bob@robots.ox.ac.uk>
 */
class ModelBrowser {
  public:
    ModelBrowser();
    ~ModelBrowser();

    void Reset();
    bool Init( CVD::ImageRef irPosition, std::string sBrowserImageFileName);
    void Draw(  const GLWindow2 &glWindow );

    enum CLICK_STATUS { OK, NONE, CANCEL, MODEL };
    CLICK_STATUS HandleClick(Vector<2> v2VidCoords);
    const ModelData * GetSelectedModelData() { return mpSelectedModelData; }

  private:
    bool _ParseModelDB();
    void _SetupModelPages( Pages & pages, const std::vector< ModelData > & vModels );
    void _SetupCategoryPages( Pages & pages, const std::vector< CategoryData > & vCategories );
    
  private:
    std::vector< CategoryData > mvModelsDB;        // holds the data for the list of categories and the associated models
    const ModelData * mpSelectedModelData;         // The selected model's data
    Pages mPages;                                  // the top level of pages
    Pages * mpCurrentPageSet;                      // for displaying the current page and associated info
    
    CVD::Image< CVD::Rgba<CVD::byte> > mimBrowser; // image for the browser
    CVD::ImageRef mirPosition;                     // The top left corner of the button set
    bool mbInitialized;                            // has the browser been initialised
};


}

#endif
