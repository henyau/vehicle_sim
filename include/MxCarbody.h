#pragma once
#include "RigidBody.h"
#include "Matrix.h"
#include "ode/ode.h"

//holds RB parameters (ODE) and collision geometry of car body (sprung)
class CMxCarbody: public RigidBody 
{
public:
	CMxCarbody(void);
	~CMxCarbody(void);

	static void setStepSize(float h);

	void Load();
	void Update();

	float m_rWidth;
	float m_rHeight;
	float m_rLength;

	//dBodyID odeBody;
	//dMass   odeMass;
 // 	dGeomID odeGeom;

	//Car body Static Attributes
	float rChassisMass, rOneOverMass;                        // mass
    float minusMG;                                  // -mass * g
    Vector3p cOfM;                                   // centre of mass (in old body coords)
	// spacial/physical
	Vector3p vSize; //dimensions of chassis
	float rCoeffOfRestitution;     // Bounciness
	Vector3p vAeroCenter,         // Drag center relative to CG
           vAeroCoeff;          // Drag coefficients (OpenGL axis system)
	float   rAeroArea;           // Drag area


		//Accessors
	// Static attributes
	float    GetMassChassis(){ return rChassisMass; }
	float    GetCoeffRestitution(){ return rCoeffOfRestitution; }
	Vector3p *GetSize(){ return &vSize; }
	Vector3p m_vDown;
  



};
