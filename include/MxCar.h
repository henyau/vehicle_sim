#pragma once
#include "MxWheel.h"
#include "MxSusp.h"
#include "MxEngine.h"
#include "MxTransmission.h"
#include "MxCarbody.h"
#include "MxCarAttrib.h"
#include "MxDifferential.h"
#include "MxAero.h"
#include "MxDriveline.h"

#define DllExport   __declspec( dllexport ) 

class CMxSusp;
class CMxWheel;
class CMxDifferential;
class CMxAero;
class CMxDriveline;

class /*DllExport*/ CMxCar
{
public:
//	CMxCar(dWorldID odeWorld, dSpaceID odeSpace);
	
	CMxCar(void);
	CMxCar( CMxCarAttribute _CarAttrib, dWorldID odeWorld, dSpaceID odeSpace, float posx, float posy, float posz, unsigned int number);
	
	~CMxCar(void);

	CMxCarAttribute m_CarAttrib;

	void Initialize(Vector3p & linPos, Quaternionp & qRotPos);
	void Update();
	void Reload(CMxCarAttribute _CarAttrib, float posx, float posy, float posz);
	void CalcRollCenters();

	//car's components
	CMxWheel *mWheels[4];
	CMxSusp *mSusp[4];	
	CMxEngine *mEngine;
	//CMxDifferential *mDifferential; // i'm moving this to the driveline
	CMxAero *mAerodynamics;
	CMxTransmission *mTransmission;
	CMxDriveline *mDriveline; //contains engine/diffs/transmission/clutch/handbrake

    CMxCarbody mCarbody;//contains rigid body information that can be used in place of ODE
	///

    Vector3p frontWheelAxis0[2];                     // axis of front wheels when steering is zero
    Vector3p dWheelAxis[2];                          // change in front wheels axis due to steering
    float wOver2L;                                  // Ackerman steering constant.
    float steeringLock;                             // max steering angle (rads)
    int driveType;                                  // DRIVE_FWD, DRIVE_RWD or DRIVE_4WD

	//Vector3p suspAxis[4];// suspension transformed to car coords.
	//don't use this anymore

	Matrix3x3 mBodyMat;//
	dSpaceID mOdeSpace;
	dWorldID mOdeWorld;
	//RigidBody m_Body;

	//	dBodyID  m_wheelBodyID[4];
	//	dBodyID  m_chassisBodyID;
	//	dGeomID m_wheelGeomID[4];
	//	dGeomID m_chassisGeomID;
	//ODE Geom for test, don't 
	//dGeomID mOdeWheelGeom[4];

	//Force Functions
    void getBodyPos(Vector3p & linPos, Matrix3x3 & rotPos);      // get pos of body
    void getWheelPos(Vector3p linPos[4], Matrix3x3 rotPos[4]);   // get pos of wheels
   
    float getDriveVel(void);                                    // get speed of drive shaft
    float getWheelVel(int i);                                   // get angular vel of wheels
    void getSuspPosVel(int i, float & suspPos, float & suspVel);// get suspension pos and vel
    void getTirePosVel(int i, Vector3p & linPos, Vector3p & linVel, Vector3p & axis, float & rotVel);

	float getDriveAccel();
	float getWheelOldVel(int i);

	void setSteering(float steer);                              // set current steer position

    void addDriveTorque(float driveTorque);                     // apply torque to                                                             
	void addWheelBrakingTorque(int i, float wheelTorque);              // apply torque for brakes
	void addSuspForce(int i, float suspForce);                  // apply suspension forces
    void addTireForce(int i, Vector3p & worldPos, Vector3p & worldForce);
    //void addBodyForce(Vector3p & worldPos, Vector3p & worldForce);// { mainBody.addWorldWorldForce(worldForce, worldPos); }
	//test ODE dVector
	void addBodyForce(Vector3p worldPos, Vector3p & worldForce);// { mainBody.addWorldWorldForce(worldForce, worldPos); }
	void addBodyTorque(Vector3p bodyAxis, float torqueScale);

	bool bTireInitialized;

	//To evolve suspension and wheels. RACER says it uses an implicit euler for the damper. look into it
	void Evolve(float h, float steer, float accel, float brake, float handBrake, float reverse);
	void RKInit();
	void RKStep1();
	void RKStep2();
	void RKStep3();
	void RKStep4();
	void EulerStep();

	void forces();

	float maxBackBrakeTorque;
	float maxFrontBrakeTorque;

	float maxHandBrakeTorque;

	float backBrakeTorque;
	float frontBrakeTorque;
	float backBrakeFriction;
	float frontBrakeFriction;

	unsigned int m_iNumber;

	
	
	///
	Vector3p vInitalPos;
	Vector3p vRollAxis_WC;///dynamic
	//Vector3p vSideViewRollCenterFront;
	//Vector3p vSideViewRollCenterRear;
	//Vector3p vFrontViewRollCenterFront;
	//Vector3p vFrontViewRollCenterRear;

	Vector3p vRollCenterFront_CC; // side view is (y,x) front view is (z,y)
	Vector3p vRollCenterRear_CC;
	Vector3p vRollCenterFront_WC;
	Vector3p vRollCenterRear_WC;

	void addApplySuspForce(int i, Vector3p suspForce);

	float fCoMY;
	float fCoMZ;

	float tempSteerPos;

	//for virtools. I don't want to keep anything other than physics info inside the mxphyslib library, but otherwise I need to keep a separate array just for this 
	unsigned int mVT_tireGroup;
	
	
};
