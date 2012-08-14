// Copyright 2009 Isis Innovation Limited
//
// C++ Interface: Model3ds
//
// Description: A 3ds model class
//
//
// Author: Robert Castle <bob@robots.ox.ac.uk>, (C) 2009
//
#ifndef MODEL3DS_H
#define MODEL3DS_H

#include <lib3ds.h>
#include <string>

#include "OpenGL.h"

#include <TooN/TooN.h>

namespace PTAMM {

using namespace TooN;

class TiXmlElement;

typedef float Vector3D[3];  //for use with lib3ds

/**
 * A 3DS model class
 * @author Robert Castle <bob@robots.ox.ac.uk>
*/
class Model3DS{

  public:
    Model3DS();
    ~Model3DS();

    bool Load( std::string sModelDir, std::string sFileName, TooN::Vector<3> v3Rotation );
    void Save( TiXmlElement * modelsNode );
    bool Load( TiXmlElement * modelsNode );

    void Draw();

    void ScaleIncrease();
    void ScaleDecrease();
    void MoveForwards();
    void MoveBack();
    void MoveLeft();
    void MoveRight();
    void MoveUp();
    void MoveDown();
    void PitchIncrease();
    void PitchDecrease();
    void RollIncrease();
    void RollDecrease();
    void YawIncrease();
    void YawDecrease();
    void MoveTo( Vector<3> v3Position );

    const SE3<> & Pose() { return mse3MfromW; }
    const double Scale() { return mdScale; }
    void Reset();

    std::string & Name() { return msName; }
    double Diameter() { return 3*mdScale*mdDiameter; }

  protected:
    bool _Load();
    GLuint _GenerateDisplayList( Lib3dsFile * pModel, bool bWireframe );
    void _DrawAxes();
    void _FindBoundingBox( Lib3dsFile * pModel );
    void _DrawBoundingBox();

    void _DelayedLoad();
    void _LoadTextures( Lib3dsFile * pModel );

  protected:
    GLuint mnDisplayList[2];        // The OpenGL display lists for the model
    bool mbLoaded;                  // Has the model been loaded

    std::string msName;             // Name of the model
    double mdScaleMult;             // the scale multiplier for the model
    double mdScale;                 // the model's scale;
    SE3<> mse3MfromW;               // Model's pose and location in the world.
    SE3<> mse3ModelOffset;          // Initial rotation of model to line up with the axes
    
    Vector<3> mv3DimensionMin;      // The minimum and maximum values for the model's
    Vector<3> mv3DimensionMax;      // boundary. Used to form the bounding box
    Vector<3> mv3CentreOfMass;      // The model's centre or mass. Model rotates about this
    Vector<3> mv3Offset;            // This displacement make the centre of the model's base the origin.
    double mdDiameter;                // This is used to set the radius of the selected target

    std::string msModelDir;         // The directory containing the model file and its textures
    std::string msModelFile;        // The model file name

    bool mbDelayedLoad;             // Is a delayed load pending?
};


}


#endif
