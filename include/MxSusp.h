#pragma once
#include "Matrix.h"
#include "MxCar.h"
#include "MxWheel.h"

#include "MxCarAttrib.h"

class CMxCar;
class CMxWheel;

class CMxSusp
{
	
public:
	CMxSusp(CMxCarAttribute *_pCarAttr,CMxCar *_Car, int _which);
	~CMxSusp(void);

	void Initialize();
	void Update();
	void Reload(CMxCarAttribute *_pCarAttr);

	int m_iWhich;
	
	CMxCar *pCar;
	CMxWheel *pWheel;

	

	// Spring
	float restLength,                  // Unsprung length of entire suspension
			minLength,                   // Minimal physical length
			maxLength;                   // Maximal physical length
	float   ks;                           // Spring constant/wheel rate (N/m)
	// Damper
	float   bumpRate,                    // Nominal compression rate (N/m/s)
				reboundRate;
	float   radius;                      // Radius of cylinder
	
	float       arbKs;                 // N/m

	//state
	float linearPosition;                      // Current length
	float linearVelocity;              // Speed at m_iWhich piston is moving
	float linearForce;
	Vector3p forceBody,                   // Force that body is putting on us
           forceWheel;                  // Force that wheels puts on us

	Vector3p forceSpring,                 // Total force of the spring
           forceDamper,                 // Damping force of the damper
           forceARB;                    // Anti-roll bar
	//RK4 intermediate states
	float linearVelocity0, linearVelocity1, linearVelocity2, linearVelocity3;
	float linearPosition0;
	float linearAcceleration0, linearAcceleration1, linearAcceleration2, linearAcceleration3;

	float linearAcceleration;

	////////////suspension geometry properties
	//states
	Vector3p vSideViewIC_CC;
	Vector3p vFrontViewIC_CC;

	Vector3p vSideViewIC_WC;
	Vector3p vFrontViewIC_WC;

	Vector3p vInstantaneousAxisCC;//for adding forces to body and finding IC;


	////
	Vector3p position;//position of wheel at full compression (origin of suspension)// in car coords
	Vector3p suspPos0;//same as above in world diagonal coords.
	Vector3p axis;//axis in CC
	Vector3p axisWC;//axis in WC

	//static
	//for now, base these off of position +/- 
	Vector3p vMountTopCC; //where the suspension is mounted on car body// should be in Car Coords
	Vector3p vMountBottomCC;

	//state
	Vector3p vMountTopWC; //same as above in world coords;
	Vector3p vMountBottomWC;

	//static
	float fLengthTop;
	float fLengthBottom;

	//state
	Vector3p vLowerBallJointCC;
	Vector3p vUpperBallJointCC;
	Vector3p vLowerBallJointWC;
	Vector3p vUpperBallJointWC;

	Vector3p vKnucklePosCC;
	Vector3p vKnucklePosWC;
	float fKnuckleOffset[3];//x,y (-,+)
	
	Matrix3x3 mKnuckleRotPosWC; // wheelRotPosNoSpin is equal to this


	Vector3p vInstantRollAxis;

	void DoForces(float wheelForce);

	//more properties
	float fRCYPos;
	float fRCZPos;
	float fInstallationRatio;
	float fMountLength;
	float fMountingAngle;//from car up.

	//for SLA
	float fSpindleLength;
	//for semi-trail
	float fSweepAngle;
	float fSTABackYOffset;

	float fCasterAngle;
	int iType;
	//#define SUSP_TYPE_MACPHERSON 0
	//#define SUSP_TYPE_SLA 1
	////rear only
	//#define SUSP_TYPE_SEMITRAILINGARM 2
	//#define SUSP_TYPE_5LINK 3

	float upperarmLength;

	Vector3p tempAltitudeBase;
	
};
