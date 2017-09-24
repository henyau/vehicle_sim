#pragma once
#include "MxCar.h"
#include "MxCarAttrib.h"

class CMxCar;
class CMxDifferential
{//input: torque/vel from engine
//output: torque to add to each wheel.  This makes the assumption that cars must 
//be fwd, rwd, or awd, have 4 wheels, 1 or 3 differentials

public:
	CMxDifferential(	CMxCarAttribute *_pCarAttr, CMxCar *_Car);
	~CMxDifferential(void);
	void Initialize();
	void Reload(CMxCarAttribute *_pCarAttr);
	void Forces(float _torqueIn, float _inertiaIn);

	float GetDriveshaftVelocity();
	float GetDriveshaftAcceleration();


	//void findTorque

	bool m_bAWD;
	bool m_bLocked0, m_bLocked1, m_bLocked2, m_bLocked3;
	unsigned int m_uiDiffTypeFront;//0 -open , 1 - Viscous LSD, 2- locked, 3 - Salisbury LSD
	unsigned int m_uiDiffTypeRear;
	unsigned int m_uiDiffTypeCenter;

	float m_rRotVelIn;//From engine
	float m_rRotVelOut[4]; //at each wheel output shaft of F/R diffs

	float m_rTorqueIn;
	float m_rTorqueOut[4];
	
	float m_rInertiaIn; // add later... probably not much influence
	float m_rInertiaOut[4];
	

	

	//float m_rLockingPercentage;
	float m_rTorqueBiasCenter; // torqueBias = torqueGrippy/torqueSlippy
	float m_rTorqueBiasRear;
	float m_rTorqueBiasFront;

	float m_rLockingCoefCenter; // for ViscLSD torqueLocking = lockingCoef*outputRotspeedDiff (rad/sec)
	float m_rLockingCoefRear;
	float m_rLockingCoefFront;

	float m_rDiffRatioRear; //final Drive if diff on driveAxel (assume centerdiff doesn't alter drive ratios)
	float m_rDiffRatioFront;

	int m_iNumClutchsRear;
	int m_iNumClutchsCenter;
	int m_iNumClutchsFront;

	CMxCar *pCar; // car has pointers to engine and wheels.



	
	/*
	float m_rInertiaRingGear;    
	//viscous LSD
	float m_rLockingCoef;
	int m_iNumClutches;

	//Salisbury
	float m_rPowerAngle;
	float m_rCoastAngle;
	float m_rClutchFactor;
*/

};