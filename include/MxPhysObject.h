#pragma once
#include "ode/ode.h"
#include "matrix.h"
#define DllExport   __declspec( dllexport ) 

class DllExport CMxPhysObject
{
public:
	CMxPhysObject(void);
	CMxPhysObject(dWorldID odeWorld, dSpaceID odeSpace,float mass, float length, float height, float width, 	 float posx, float posy,float posz);
	~CMxPhysObject(void);

	dBodyID odeBody;
	dMass   odeMass;
  	dGeomID odeGeom;

	  float mass, oneOverMass;                        // mass
    float minusMG;                                  // -mass * g
    Vector3p cOfM;                                   // centre of mass (in old body coords)

	Vector3p linPos, linVel;     
	Matrix3x3 rotPos;     
	Quaternionp qRotPos;                             // Quaternionp of above matrix (matrix and Quaternionp are kept the same)
	Vector3p rotVel;                                 // rotational velocity (diagonalised body coords)

// current forces on body
	Vector3p linForce, rotForce;                     // current forces

	//Vector3p vInitialPos;


	void Update();
	void doForces();
	void setPosition(float x, float y, float z);

	//	
	float rLength;
	float rWidth;
	float rHeight;
};

class CMxPhysBox:CMxPhysObject
{
	float rLength;
	float rWidth;
	float rHeight;

};

class CMxPhysSphere:CMxPhysObject
{
	float rRadius;

};
