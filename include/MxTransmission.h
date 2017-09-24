#pragma once
#include "Matrix.h"
//#include "MxPhysLib.h"
#include "MxCarAttrib.h"
#include "MxEngine.h"
#include "MXTimer.h"

//NOTE:::Going to put all the Transmission stuff in CMxEngine for now. they are both very small classes and similar too
//NOTE2:: Moving the transmission stuff back

class CMxDifferential;
class CMxEngine;
//
//typedef struct
//{
//	int state;
//	#define CLUTCH_APPLIED	 1
//	#define CLUTCH_RELEASED  0
//	#define CLUTCH_RELEASING 2
//
//	int mode;
//	#define CLUTCH_AUTO	0
//	#define CLUTCH_MANUAL	1
//
//	float timeToRelease;
//	float releaseTime;
//	float transferValue;
//	float plip;
//}tClutch;



class CMxTransmission
{
public:
	CMxTransmission(CMxCarAttribute *_pCarAttr, CMxEngine *_pEngine);
	~CMxTransmission(void);

	///pointers
	CMxDifferential *pDiff; 
	CMxEngine *pEngine;

	void Reload(CMxCarAttribute *_pCarAttr);	
	void BeginUpdate(float _throttle, float _brakePos, float _clutchPos, bool shiftUp, bool shiftDown, int timeMS);//,  float _clutchPos, float _handBrakePos);
	void DoForces(float omegaD);

	float GetCurrentRatio();
	//float GetFinalDriveRatio();//part of drive axel...not really transmission...who cares....i do


	unsigned int m_iMaxGears;
	float m_rBrakePos;
	float m_rClutchPos;
	int m_iCurrentGear;
	float m_rHandBrakePos;


//Transmission data

	float rCurrentInertia;
//
//	float m_fClutchKv;
//	float m_fClutchTorque;
//
//    int   m_iFrontRearAWD;          // 0:«eÅX  1:«áÅX  2:AWD
//#define DRIVETYPE_FWD 0
//#define DRIVETYPE_RWD 1
//#define DRIVETYPE_AWD 2
//
//    float m_fFRPowerSplit;

	///////

    float m_fEffiency;
 
    int   m_iNumGears;

	float m_rAutoClutchBrake;

	float m_rFowardGearRatios[7];
	float m_rFowardGearInertias[7];
	

	float m_rReverseGearRatio;
	float m_rReverGearInertia;
	float m_rFinalDrive;
	float m_rFinalDriveInertia;

	float m_AutoShiftUp,m_AutoShiftDown;
	int m_iAutoTrans; //0 auto 1 manual

	CMxTimer *m_ShiftTimer;

	bool m_bOldShiftUpButton;
	bool m_bOldShiftDownButton;
	
	bool m_bIsShiftingUp; // in the middle of shifting (clutch depressed, throttle closed)
	bool m_bIsShiftingDown;
	bool m_bChangedGears;
	unsigned int m_uiShiftTimer;


	float m_fDriveshaftTorque;


	//states
	float omegaE;
	float omegaG;

	float GetCurrentInertia();

	int m_iTimeStartShift;
	int m_iTimeDepressClutch;
	int m_iTimeApplyClutch;
	
	bool m_bAutoBlipThrottle;// on down shift
	
};
