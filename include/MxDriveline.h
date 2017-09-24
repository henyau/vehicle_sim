#pragma once

#include "Matrix.h"
#include "MxTransmission.h"
#include "MxEngine.h"
#include "MxDifferential.h"
#include "MxCarAttrib.h"
#include "MxCar.h"

#define DllExport   __declspec( dllexport ) 
CMxEngine;
CMxTransmission;
CMxDifferential;
CMxCar;

#define FRONT_DIFF	0
	#define REAR_DIFF		1
	#define CENTER_DIFF	2

class DllExport CMxDriveline
{
public:
	CMxDriveline(CMxCarAttribute *_pCarAttr, CMxEngine *_pEngine, CMxTransmission *_pTransmission);
	~CMxDriveline(void);

	void Reload(CMxCarAttribute *_pCarAttr);

	CMxCar *pCar;
	CMxEngine *pEngine;
	CMxTransmission *pTransmission;
	
	//CMxDifferential *pDifferential;

	
	CMxDifferential mDifferential[3];

	//properties
	float m_rFinalGear; //
	float m_rEfficiency;

	//clutch state
	float rTorqueClutch;
	float rClutchVel;
	float rClutchNormalForce; // = m_rClutchMaxNormalForce * clutchPos^2;
	float m_rClutchMaxNormalForce;
	int m_iClutchInterfaces;
	float m_rClutchMeanRadius; //(Inner Ring +outer ring )*0.5
	float m_rClutchFriction;
	///
	
	float m_fClutchKv;
    int   m_iFrontRearAWD;          // 0:«eÅX  1:«áÅX  2:AWD
	#define DRIVETYPE_FWD 0
	#define DRIVETYPE_RWD 1
	#define DRIVETYPE_AWD 2

    float m_fFRPowerSplit;
	
	

	//
	float rDriveshaftTorque; //torque after clutch (drive line torque), should then be mult by Gfd
	float rDriveshaftAccel;
		
	
	//bool states
	bool bAutoClutch;
	bool bClutchSlipping; // slip can mean "clutch fully depressed" or "cluch/engine not completely locked"
	bool bInGear;	 //need since shifting means out of gear
	bool bDiffLocked;

	bool IsClutchSlipping(){ return bClutchSlipping; }
	void LockEngineClutch(){ bClutchSlipping=false; }
	void UnlockEngineClutch(){ bClutchSlipping=true; }
	
	float GetClutchTorque(){ return rTorqueClutch; }

	bool IsAutoClutchActive(){ return bAutoClutch; }
	void EnableAutoClutch(){ bAutoClutch=true; }
	void DisableAutoClutch(){ bAutoClutch=false; }

	void SetInput(int ctlClutch,int ctlHandbrake);
	void DoForces(float omegaD);
	void EulerStep();

	//accessors
	float GetClutchVel();
	float GetFinalDriveRatio();
	float GetOverallRatio();
	float GetDriveWheelVel();

	float GetDriveWheelAccel();
	float GetClutchAccel();

	float GetDifferentialInertia();
	///
	float m_Inertia; // total inertia of system (from gearbox to wheels)
	float m_DriveShaftInertia;

	
};
