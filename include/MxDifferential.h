#pragma once
//#include "MxCar.h"
#include "MxCarAttrib.h"
#include "math.h"

//class CMxCar;
typedef struct
{
	float	rotVel;
	float	torque;
	float	brakingTorque;
	float	inertia;
} torqueAxis;

class CMxDifferential
{

public:
	CMxDifferential(void);
	//CMxDifferential(	CMxCarAttribute *_pCarAttr, CMxCar *_Car);
	~CMxDifferential(void);

	void Initialize();
	void Reload(CMxCarAttribute *_pCarAttr, int which, bool bpowered);

	//void Forces(float _torqueIn, float _inertiaIn);
	void DoForces();
	void DoSpoolForces();
	void EulerStep();
	void UpdateSpool();

	float GetDriveshaftVelocity();
	float GetDriveshaftAcceleration();
	



	////void findTorque
	//bool m_bAWD;

	//bool m_bLocked0, m_bLocked1, m_bLocked2, m_bLocked3;
	//unsigned int m_uiDiffTypeFront;//0 -open , 1 - Viscous LSD, 2- locked, 3 - Salisbury LSD
	//unsigned int m_uiDiffTypeRear;
	//unsigned int m_uiDiffTypeCenter;

	//float m_rRotVelIn;
	//float m_rRotVelOut[2]; 

	//float m_rTorqueIn;
	//float m_rTorqueOut[2];
	//
	//float m_rInertiaIn; 
	//float m_rInertiaOut[2];
	
//	float m_rLockingCoef; // for ViscLSD torqueLocking = lockingCoef*outputRotspeedDiff (rad/sec)
//	float m_rDiffRatio; 
//	int m_iNumClutchs;

	
	#define DIFF_NONE		0
	#define DIFF_SPOOL		1
	#define DIFF_FREE		2
	#define DIFF_LIMITED_SLIP	3
	#define DIFF_VISCOUS_COUPLER	4

	int type;
    float	m_rFinalDrive;
    float	m_rInertia;
    float m_rEfficiency;
    float	m_rTorqueBias;
    float	m_rTorqueBiasMin;
    float	m_rTorqueBiasMax;
    float	m_rSlipBiasMax;
    float	m_rLockingTorque;
    float	m_rViscosity;
    float	m_rViscosityMax;

	int m_iWhich;
	bool m_bPowered;

	//CMxCar *pCar; // car has pointers to engine and wheels.
   
	//Input
	torqueAxis InputIn; //add into diff
    torqueAxis InputOut; //from diff to what ever is connected to input
	
	//output
    torqueAxis *OutputIn[2]; //what's coming into the diff from output shaft
    torqueAxis *OutputOut[2];//what's going out of the diff from the output shaft

	//States
	/*float rotAccel;
	float rotVel;*/

};