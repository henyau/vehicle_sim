///simple container for all simulation objects (world, track, car)  
//added simple collision objects (boxes)6/23
#pragma once
#include "MxTrack.h"
#include "ode/ode.h"
#include "MxCar.h"
//#include "MxBike.h"
#include "MxPhysLib.h"
#include "MXTimer.h"
#include "MxCarAttrib.h"
#include "MxPhysObject.h"
#include <vector>
#include <iostream>
using namespace std;

void nearCallback2 (void *data, dGeomID o1, dGeomID o2);

//static dContactGeom gWheelContact0;
//static dContactGeom gWheelContact1;
//static dContactGeom gWheelContact2;
//static dContactGeom gWheelContact3;

class CSim
{
public:

	DllExport CSim();
	DllExport ~CSim(void);
	void DllExport Init();
	void DllExport Reload(Vector3p position, Quaternionp orientation);

	//void SetControlInputs(int carNum, float elapsed, float throttlePos, float brakePos, float steeringwheelPos, bool shiftUp, bool shiftDown, bool clutch);	
	void DllExport SetControlInputs(int whichCar, float elapsed, float throttlePos, float brakePos, float steeringwheelPos, bool shiftUp, bool shiftDown,  float _clutchPos, float _handBrakePos);
	//void SetControlInputs(float elapsed, float throttlePos, float brakePos, float steeringwheelPos, bool shiftUp, bool shiftDown, bool clutchEngaged, bool handBrake);	
	void DllExport Update(float stepsize);
	void DllExport  EvolveToNow();

 	//accessor
	static CSim*  GetInstance(){return m_Inst;}
	static CSim DllExport *m_Inst;


	CMxTimer *simTimer;

	//time steps
	//RK4
	static float RKh, RKh2, RKh4 ,RKh6;
	//Euler
	float h;

	
	dWorldID m_odeWorld;
	dSpaceID m_odeSpace;  
	dJointGroupID m_odeContactGroup; 
	CMxTrack *m_Track;
	
	//CMxCar *m_Car[16];//create a vector or linkedlist
	vector <CMxCar*> m_Car;
	vector <CMxPhysObject*> m_Boxes;
	
	//CMxBike *m_Bike[2];
	//	CMxCarAttribute *m_CarAttributes[32];//car settings
	CMxCarAttribute m_CarAttribute;//car settings
	
	//unsigned int m_iNumCars;
	unsigned int m_iPlayerNumber;


	DllExport static dContactGeom debugContact;
	unsigned int m_uiIterations;

	//stuff to smooth out MFC app's weird timing
	unsigned int m_uiIterationsOld;//keep track of the previous
	unsigned int m_uiNumFrameSameRate; //number of frames to with the same rate


	
	///

	//time constants
	///set these as constants!!!
  	int m_iMaxSimTimePerFrame;
 	int m_iTimePerStepMS;
	float m_fTimePerStepSec;

	void SetSimTime(int maxSimTimePerFrame, int timePerStep);

	void AddCar(CMxCarAttribute _CarAttrib, Vector3p position, Matrix3x3 orientation, Vector3p velocity);
	void AddBike(CMxCarAttribute _BikeAttrib, Vector3p position, Matrix3x3 orientation, Vector3p velocity);
	void AddBox(Vector3p size, float mass,Vector3p position, Matrix3x3 orientation, Vector3p velocity);
	void AddParticleSystem();

	void RemoveCar(int carNumber);
	void RemoveBike(int bikeNumber);
	void RemoveBox(int boxNumber);
	void RemoveParticleSystem();

};
