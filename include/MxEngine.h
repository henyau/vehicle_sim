#pragma once
#include "Matrix.h"
#include "MxTransmission.h"
//#include "MxPhysLib.h"
#include "MxCarAttrib.h"

#define DllExport   __declspec( dllexport ) 
class CMxCar;

class DllExport CMxEngine
{
public:
	CMxEngine(CMxCarAttribute *_pCarAttr, CMxCar *_pCar);
	~CMxEngine(void);
	void Reload(CMxCarAttribute *_pCarAttr);
	
	CMxCar *pCar;

	//	CMxTransmission *pTransmission;

	float m_rLength, m_rHeight, m_rWidth;
	float m_rPosx, m_rPosy,m_rPosz;
//	unsigned int m_iMaxGears;
//	int	 m_iCurrentGear;

	float	m_rTorqueCurve[10];
	
	float	m_rRevLimit;
	float m_rThrottlePos;
	float m_rBrakePos;
	float m_rClutchPos;
	

    
	float m_frictionTorque;
    float m_frictionKv;
    float m_mOfI;
	float oneOverMOI;
    // torque = f(omega) * throttle

    float h;


	//Output
	float m_rBrakeForce;
	float m_rTorque;

	//state variables
	float vel, vel0;//vel is the free reving velocity
	float vel1, acc1;
	float vel2, acc2;	float vel3, acc3;


    //int create(CarEngineData & data, LookupTable & table);

	
	//float CreateTorquePoly();// see how fast i can create a quadradic polynomial for every 3 points first...
	float GetTorque(float omega);//returns torque given RPM
	//float CalcAcceleration();

	void Reset();
	
	//RK4 funks
    void Initialize(void) { vel = 0.0;/* Reload();*/}
    //void beginUpdate(float _throttle, float _brakePos, bool ShiftUp, bool ShiftDown, bool _clutchEngaged);
    void RKStep1(float clutchTorque);
    void RKStep2(float clutchTorque);
    void RKStep3(float clutchTorque);
    void RKStep4(float clutchTorque);
    void undo(void);

	void EulerStep(float clutchTorque);
	////////////////////////////
	//void forces(float omegaD); //moved to transmission. will pass engine velocity back

	float DoForces(float omegaE);
	float GetMaxTorque(float omegaE);
	float GetMinTorque(float omegaE);
	void DoStarter();

	float m_fEngineBrakeCoef;

	float m_fMinTorque;
	float m_fMaxTorque;

////Transmission data
//	float m_fClutchKv;
//
//	float m_fClutchTorque;
//
//    int   m_iFrontRearAWD;          // 0:«eÅX  1:«áÅX  2:AWD
//    float m_fFRPowerSplit;
//    float m_fEffiency;
//    float m_fEngineBrakeCoef;
//    int   m_iNumGears;
//
//	float m_rAutoClutchBrake;
//
//	float m_rFowardGearRatios[7];
//	float m_rReverseGearRatio;
//	float m_rFinalDrive;
//
//	float m_AutoShiftUp,m_AutoShiftDown;
//	int m_iAutoTrans; //0 auto 1 manual
//
//	bool m_bOldShiftUpButton;
//	bool m_bOldShiftDownButton;
//
//	float m_fDriveshaftTorque;

};
