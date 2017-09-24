#pragma once
#include "MxCar.h"
#include "MxSusp.h"
#include "MxTire.h"
#include "Matrix.h"
//#include "CollisionPoint.h"
#include "ode/ode.h"
#include "MxPhysLib.h"
#include "MxDifferential.h"

#include "MxCarAttrib.h"
//#include "RigidBody.h"

#include "MxTrack.h"

class CMxCar;
class CMxSusp;
class CMxTire;

typedef struct
{
	float u, v, t;
	Vector3p normal;
	 int triangleIndex;
	 int lastIndex;//cache last index since you're probably still on it
	 
	 float fSurfaceType;//0...1
	 float fRollingResistanceFactor;
	 float fSurfaceFriction;

	 float fFeedbackFactor;
	 

}tireContactData;

class CMxWheel
{
public:
	CMxWheel(CMxCarAttribute *_pCarAttr, CMxCar *car , CMxSusp *susp, int i);
	~CMxWheel(void);
	void Reload(CMxCarAttribute *_pCarAttr);
	void getPosition(Vector3p &linearPos, Matrix3x3 &rotationalPos);
	void getPosVel();

	int m_iWhich;

	CMxSusp *pSusp;
	CMxCar *pCar;
	CMxTire *pTire;

	//Physical parameters
	float mass;
	float oneOverMass;
	float radius;
	float width;
	float momentOfInertia;
	float oneOverMOfI;

	//float   rollingCoeff;                // Rolling resistance coefficient
	float m_rRollingResistanceSpeed;
	float m_rRollingResistanceBasic;
	float m_rRollingResistance;

	float   toe;                         // Pre-rotating the wheel
	//float   ackermanFactor;              // Scaling the steering angle//moved to CMxCar
	float   staticCamber;                // Camber at default susp/steering

	Vector3p axis; //rotation axis of the wheel.  calculate from toe and camber and input
							//points to lefthand curl
	float dynamicCamber; // actual camber angle wrt track normal
	float normDynamicCamber;//normalized to load and friction
	float m_CamberStiffness;

	Matrix3x3 noRotOrient;// orientation of the wheel if no spin. 
	Matrix3x3 rotPosWorld;


	///test
	//Vector3p m_TireContact;// from Moeller's ray triangle intersection
	//Vector3p m_TrackNormal;
	tireContactData m_tireContact;
	///

	// Spring values
	float   tireSpringRate;			// Spring vertical rate of tire

	

	//Vector3p linPos, linVel;//for slip calcs

	int forces(Vector3p &fz);
		// Translation
	Vector3p position;                    // Position in CC wrt the body (!)
	Vector3p velocity,                    // Velocity in wheel coords
           acceleration;                // For the suspension

	Vector3p positionWld;
	Vector3p velocityWld;
	Vector3p axisWorld;

	Vector3p forcePosWld;
	
	//graphics debug
	Vector3p debugForceLat;
	Vector3p debugForceLong;
	Vector3p debugForceLoad;

	// Rotation (state)
	float rotPos; 
	float rotVel,rotAccel;   
	float rotForce;
	
	//float oldRotForce;

	float oldRotVel;

	//load
	float load; //scalar, *surface normal is load vector

	//RK4  intermediate states
	float rotPos0; 
	float rotVel0, rotVel1, rotVel2, rotVel3;
	float	rotAccel0, rotAccel1, rotAccel2, rotAccel3;
	//
	//RK4 Funcs

	// End forces
	Vector3p forceVerticalCC;             // Result of suspension+gravity
	
	//evolves slip ratio, slip angle 
	void EulerStep();
	void RKInit();
	void RKStep1();
	void RKStep2();
	void RKStep3();
	void RKStep4();

	float tanSlipAngleVel1;
	float tanSlipAngleVel2;
	float tanSlipAngleVelT;
	float tanSlipAngleVel;
	float tanSlipAnglePosT;
	float tanSlipAnglePos;

	//float slipRatioPos2;//calc Directly
	//float slipAnglePos2;//calc directly

	float slipRatioVel1;
	float slipRatioVel2;
	float slipRatioVelT;
	float slipRatioVel;
	float slipRatioPosT;
	float slipRatioPos;
	
	float torqueRollingResist;

	bool bLocked ;

	//low speed, most of this stuff belongs in CMxTire
	float normSlipCurve(float k, float C, float E);//pacejka expressed with 3 constants

	float m_rCornerStiffness;
	float m_rLongitudinalStiffness;

	void Initialize();
	void Update();
	void CalcSlip();
    
	float oldLatForce;
	float oldLongForce; //forces from the previous time step.used to keep track of occilations
	
	float Mz;

	//Use Accel to calc vel/rotpos/ instead of Forces and Torques
	float calcWheelAccel();

	torqueAxis InputIn; 
    torqueAxis InputOut;


	Vector3p tempLeftSplineTrace;
	Vector3p tempRightSplineTrace;

	int latSplineIndex;
	int longSplineIndex;


	///TIre parameters
	float magicC;
	float magicE;
	float kMuZ; 
	float kMuY;
	float mu0; 

	float vxMin ;
	float Lx;
	float Ly;

	float rTemperature;
	float m_rRelaxationFactor;

	Vector3p m_vDown;

	Vector3p m_vWorldTireContact;
	Vector3p m_vCCTireContact;
	float normLateralSlip(float B, float C, float D, float E, float alpha);

	
    
};
