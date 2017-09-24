#ifndef RIGIDBODY_H
#define RIGIDBODY_H

#include "Matrix.h"
#include "ode/ode.h"

class RigidBody
{
// Runge-Kutta integration stuff
	//intermediate states
    Vector3p linPos0, linVel0, rotVel0;
    Quaternionp qRotPos0;
    Matrix3x3 rotPos0;
    Vector3p linVel1, rotVel1, linAcc1, rotAcc1;
    Vector3p linVel2, rotVel2, linAcc2, rotAcc2;
    Vector3p linVel3, rotVel3, linAcc3, rotAcc3;

public:		

	// current state of body
	Vector3p linPos, linVel, linVelBC;                         // linear position and velocity
	Matrix3x3 rotPos;                               // rotational position. transform from
									                // diagonalised body coord to world coords is
									                // Rw = rotPos * Rd + linPos
    Quaternionp qRotPos;                             // Quaternionp of above matrix (matrix and Quaternionp are kept the same)
	Vector3p rotVel;                                 // rotational velocity (diagonalised body coords)
	Matrix3x3    inertiaTensor;	//diagonalized

// current forces on body
	Vector3p linForce, rotForce;                     // current forces


public:
	// properties of body (these are public in case you need access, but don't change them)
	dBodyID odeBody;
	dMass   odeMass;
  	dGeomID odeGeom;
	//for ODE
	dBodyID      GetODEBody(){ return odeBody; }
	dMass        GetODEMass(){ return odeMass; }
	dGeomID      GetODEGeom(){ return odeGeom; }

    float mass, oneOverMass;                        // mass
    float minusMG;                                  // -mass * g
    Vector3p cOfM;                                   // centre of mass (in old body coords)
    float Ix, Iy, Iz;                               // Ix = sum of mx
    float Ixx, Iyy, Izz;                            // Ixx = sum of m(y^2 + z^2)
    float Ixy, Iyz, Izx;                            // Ixy = sum of mxy
                                                    // these are in old body coords

    Matrix3x3 diagRotPos;                           // transform from old body coords to diagonalised body coords.
    Vector3p diagLinPos;                             // Rd = diagRotPos * Rb + diagLinPos
    float diagIxx, oneOverDiagIxx;                  // transformed moment of inertia (diagonalised body coords)
    float diagIyy, oneOverDiagIyy;                  // also 1 / Ixx * h for Euler integrater
    float diagIzz, oneOverDiagIzz;
    float diagIyyMinusIzzOverIxx;
    float diagIzzMinusIxxOverIyy;
    float diagIxxMinusIyyOverIzz;
                                                    // (Iyy - Izz) / Ixx * h, etc..
    float diagIzzMinusIyy;
    float diagIxxMinusIzz;
    float diagIyyMinusIxx;
    float linKD;                                    // linear damping (-ve and multiplied by mass)
    float rotKDx, rotKDy, rotKDz;                   // angular damping (-ve and multiplied by Ixx, etc..)

	
    static float RKh, RKh2, RKh4, RKh6;             // step sizes

	//ctor dtor
	RigidBody();
	~RigidBody();
// functions to set up rigid body mass distribution
	void setBox(float xRadius, float yRadius, float zRadius, float mass);
    void setCylinder(float radius, float height, float mass);
    void setSphere(float radius, float mass);
    void clear(void);
    void combine(RigidBody & other, Matrix3x3 & rotPos, Vector3p & linPos);

// functions for initialising rigid body
    void diagonalise(void);
    void initialise(Vector3p & oLinPos, Vector3p & oLinVel, Quaternionp & oQRotPos, Vector3p & oRotVel,
        float linKD, float rotKD);
	static void setStepSize(float h);

// functions to update from one frame to another
    void RKInit(void);
    void EulerStep(void);
    void RKStep1(void);
    void RKStep2(void);
    void RKStep3(void);
    void RKStep4(void);
    void undo(void);

		// Integration
	void IntegrateInit();
	void IntegrateEuler();
	void PostIntegrate();


// functions to add force and torque
	void addBodyTorque(Vector3p & torque);
	void addWorldTorque(Vector3p & torque);
	void addWorldWorldForce(Vector3p & force, Vector3p & pos);
	void addWorldBodyForce(Vector3p & force, Vector3p & pos);
	void addBodyBodyForce(Vector3p & force, Vector3p & pos);

// functions to get current position and velocity of points
	void findWorldPos(Vector3p & bodyPos, Vector3p & worldPos);
	void findBodyPos(Vector3p & worldPos, Vector3p & bodyPos);
	void findBodyWorldVel(Vector3p & bodyPos, Vector3p & worldVel);
	void findWorldWorldVel(Vector3p & worldPos, Vector3p & worldVel);
    void findWorldPosVel(Vector3p & bodyPos, Vector3p & worldPos, Vector3p & worldVel);

// functions for energy and momentum (for testing)
    float energy(void);
    void momentum(Vector3p & lin, Vector3p & rot);
	//

	Vector3p    *GetLinPos(){ return &linPos; }
	Vector3p   *GetLinVel(){ return &linVel; }
	Vector3p    *GetLinVelBC(){ return &linVelBC; }
	Vector3p    *GetRotVel(){ return &rotVel; }
	Matrix3x3    *GetRotPosM(){ return &rotPos; }
	Quaternionp *GetRotPosQ(){ return &qRotPos; }
	Matrix3x3    *GetInertia(){ return &inertiaTensor; }
//	Vector3p    *GetTotalTorque(){ return &totalTorque; }
//	Vector3p    *GetTotalForce(){ return &totalForce; }

};

#endif


