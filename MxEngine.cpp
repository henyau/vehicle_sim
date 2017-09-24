#include "StdAfx.h"
#include "mxengine.h"
#include "Sim.h"

//#include "MxCarEditorDoc.h"
//#include "MxCarAttrib.h"
//#include "RigidBody.h"
//#ifdef debug
//#include "DebugConsole.h"
//#include <iostream>
//using namespace std;
//#endif


#define ENGINE_LERP
//#define ENGINE_POLYINTERP

CMxEngine::CMxEngine(CMxCarAttribute *_pCarAttr, CMxCar *_pCar)
{
	pCar = _pCar;
	Reload(_pCarAttr);
}

CMxEngine::~CMxEngine(void)
{
}

void CMxEngine::Reload(CMxCarAttribute *_pCarAttr)
{
	m_rLength = _pCarAttr->m_Engine.rLength;
	m_rHeight = _pCarAttr->m_Engine.rHeight;
	m_rWidth = _pCarAttr->m_Engine.rWidth;

	m_rPosx = _pCarAttr->m_Body.rPosOfEngine[0];//i know this is confusing... i'll rename it later
	m_rPosy = _pCarAttr->m_Body.rPosOfEngine[1];
	m_rPosz = _pCarAttr->m_Body.rPosOfEngine[2];

//	m_iMaxGears = _pCarAttr->m_GearBox.iTotalGear;//should be in transmission?
//	m_iCurrentGear = 0;

	for(int i = 0 ; i<10; i++)
		m_rTorqueCurve[i] = _pCarAttr->m_Engine.rTorqueCurve[i] ;
    
	m_rRevLimit = _pCarAttr->m_Engine.rMaxRPM;
	m_rThrottlePos = 0.0f;
	m_rBrakePos = 0.0f;
	m_rClutchPos = 0.0f;
 
	m_frictionTorque = _pCarAttr->m_Engine.rMaxFrictionTorque;
	m_frictionKv = _pCarAttr->m_Engine.rEngineFriction;
	m_mOfI = _pCarAttr->m_Engine.rMomentOfInertia;
	oneOverMOI = 1/m_mOfI;

	m_fEngineBrakeCoef = _pCarAttr->m_GearBox.rEngineBrakeCoef;

	/////tranny
	//m_fClutchTorque = 0.0f;
	//m_fClutchKv= _pCarAttr->m_GearBox.rClutchFriction;
	//
	//m_iFrontRearAWD = _pCarAttr->m_GearBox.iDriveType; // 0:«eÅX  1:«áÅX  2:AWD
	//m_fFRPowerSplit = _pCarAttr->m_GearBox.rAWDRearPowerRatio;
	//m_fEffiency = _pCarAttr->m_GearBox.rEfficiency;
	//m_fEngineBrakeCoef = _pCarAttr->m_GearBox.rEngineBrakeCoef;
	//m_iNumGears = _pCarAttr->m_GearBox.iTotalGear;
	//m_rAutoClutchBrake = _pCarAttr->m_GearBox.rAutoClutchBrake;

	//m_rFinalDrive = _pCarAttr->m_GearBox.rFinalDrive;

	//for(i = 0 ; i<7; i++)
	//	m_rFowardGearRatios[i] = _pCarAttr->m_GearBox.rFGearRatio[i] * m_rFinalDrive;

	//m_rReverseGearRatio = _pCarAttr->m_GearBox.rReverseGearRatio;
	//

 //	m_AutoShiftUp = 8000;//TEST ... forgot to put in GUI 3/7/05
	//m_AutoShiftDown = 3500;//TEST...NOTE :: Important, this needs to be changed to torque
 //	m_iAutoTrans = _pCarAttr->m_GearBox.iMode; //0 auto 1 manual
 	 //vel = 3.4f;//TEST; don't want to stall...about 1K rpm
	Reset();
}

void CMxEngine::Reset()
{
//	m_iCurrentGear = 0;
	m_rTorque = 0.0;
	m_rThrottlePos = 0.0f;
	m_rBrakePos= 0.0f;
	m_rBrakeForce =0.0f;
//	m_fDriveshaftTorque = 0.0f;
	vel = 3.4f;
	


}



//Just do lerp for now, polynomial approx is good too, but i cant remember how right now
//Doing a simple quadratic polynomial approx 
// Assume Torque is equally spaced from 0rpm to 9,000rpm
float CMxEngine::GetTorque(float omega)
{
	omega *= 0.00954929658551f ;//convert to rpm/1000

	int index = (int) floor(omega);
	float factor = omega-(float)index;
	float torqueA,torqueB;
	
#ifdef ENGINE_LERP
	if(omega>0)
	{
		if(omega<9)
		{
			torqueA = m_rTorqueCurve[index];
			torqueB = m_rTorqueCurve[index+1];
		}
		else
		{
			torqueA = m_rTorqueCurve[9];
			torqueB = m_rTorqueCurve[9];
		}
	}
	else
	{
		torqueA = 0;
		torqueB = 0;
	}
	m_rTorque = torqueA+(torqueB-torqueA)*factor;// + torqueB*(1.0f-factor);
 	
#endif
//#ifdef ENGINE_POLYINTERP
//
//#endif

	return m_rTorque;
}


/*
void CMxEngine::BeginUpdate(float _throttle, float _brakePos, bool shiftUp, bool shiftDown, bool _clutchEngaged)
{



	//implement analog clutch later.
  	if(!_clutchEngaged)
		m_rClutchPos =1.0f;
	else 
		m_rClutchPos =0.0f;
//	m_rClutchPos = 1.0f-_brakePos;

	m_rBrakePos = _brakePos;
    vel0 = vel;
	this->m_rThrottlePos = _throttle;

	//update auto gear changes
	 if(m_iAutoTrans ==0 )//&& m_iCurrentGear>0) 
	{
		
		if(vel*9.54929658551f >m_AutoShiftUp)//TEST...NOTE :: Important, this needs to be changed to torque
			if(m_iCurrentGear<m_iMaxGears){
				
				vel *= m_rFowardGearRatios[m_iCurrentGear] / m_rFowardGearRatios[m_iCurrentGear - 1];			
				m_iCurrentGear++;				
 			}
 		if(vel*9.54929658551f <m_AutoShiftDown)//TEST...NOTE :: Important, this needs to be changed to torque
			if(m_iCurrentGear>1) {
				
 				vel *= m_rFowardGearRatios[m_iCurrentGear-2] / m_rFowardGearRatios[m_iCurrentGear - 1];				
				m_iCurrentGear--;
			}
	}

	 else//manny		 
	 {
 		 if(shiftUp&&!m_bOldShiftUpButton &&m_iCurrentGear<m_iMaxGears && m_iCurrentGear != 0)
		 {
		
			 vel *= m_rFowardGearRatios[m_iCurrentGear] / m_rFowardGearRatios[m_iCurrentGear- 1];			
			 	 m_iCurrentGear++;
		 }
		else if(shiftUp&&!m_bOldShiftUpButton &&m_iCurrentGear==0 )
		 {
		
			 //vel *= m_rFowardGearRatios[m_iCurrentGear] / m_rFowardGearRatios[m_iCurrentGear- 1];			
				m_iCurrentGear++;
		 }
		  else if(shiftDown&&!m_bOldShiftDownButton&& m_iCurrentGear>0)
		 {
			 
			 vel *= m_rFowardGearRatios[m_iCurrentGear-2] / m_rFowardGearRatios[m_iCurrentGear - 1];
			 m_iCurrentGear--;
		 }
		 else if(shiftDown&&!m_bOldShiftDownButton&&m_iCurrentGear == 0)//neutral shift to reverse
		 {
			//vel *= m_rReverseGearRatio;
			m_iCurrentGear--;		 
		 }

		 else if(shiftUp&&!m_bOldShiftUpButton && m_iCurrentGear==-1)//reverse to neutral
		 {
			 m_iCurrentGear++;		 
		 }

	 }
	m_bOldShiftUpButton = shiftUp;
	m_bOldShiftDownButton = shiftDown;

	
}*/
void CMxEngine::EulerStep(float clutchTorque)
{
	//float torque = GetTorque(vel) * m_rThrottlePos;      // torque from engine
	

	//	if(!pCar->bClutchSlipping)
	//float torque = 
	DoForces(vel);
    float frictionTorque = vel * m_frictionKv;         // find friction
    if (frictionTorque > m_frictionTorque)
        frictionTorque = m_frictionTorque;

	//neutral works ok
	//acc1 = (torque - friction - clutchForce) * oneOverMOI;// * CSim::RKh2;

	//in gear test
	// if(pCar->mTransmission->GetCurrentRatio()!=0)
	

	//if(pCar->mDriveline->IsClutchSlipping())
// 		 acc1 = (torque - friction- clutchForce)* oneOverMOI;
/*
	 else 
		acc1 = pCar->mTransmission->GetFinalDriveRatio()
		* pCar->mTransmission->GetCurrentRatio()
		*pCar->getDriveAccel();//>mWheels[3]->rotAccel;
  */
	if(pCar->mDriveline->IsClutchSlipping()) //engine disconnected (at least partially) from driveline
		acc1 = (m_rTorque - frictionTorque-pCar->mDriveline->rTorqueClutch)* oneOverMOI/	pCar->mDriveline->m_Inertia;
		//acc1 = (m_rTorque - frictionTorque- clutchTorque)* oneOverMOI;

	else//let driveline calculate acc1
		acc1 = 	pCar->mDriveline->GetClutchAccel();//->GetCurrentRatio();//*>mDifferential->GetDriveshaftAcceleration();
		


	 
	 
	//add a revlimiter,
	if(vel*9.54929658551f >m_rRevLimit)
	{
		 //acc1 = ( - friction- clutchForce)* oneOverMOI*200;
		acc1 = 0.0f;
		vel = (m_rRevLimit-200.0f)*0.10471975512f;
	}

	vel0 = vel;
  	vel += acc1 * CSim::RKh;

	if(vel*9.54929658551f <800.0f)//idle controller
	{
 		vel = (800.0f)*0.10471975512f;
	}

	if(vel<1.0f)
		vel = 0.0f;

}

void CMxEngine::RKStep1(float clutchForce)
{
    float torque = GetTorque(vel) * m_rThrottlePos;      // torque from engine
    float friction = vel * m_frictionKv;         // find friction
    if (friction > m_frictionTorque)
        friction = m_frictionTorque;
	acc1 = (torque - friction - clutchForce) * oneOverMOI * CSim::RKh2;
    vel1 = vel;
    vel += acc1;
}

void CMxEngine::RKStep2(float clutchForce)
{
	 
    float torque = GetTorque(vel) * m_rThrottlePos;      // torque from engine
    float friction = vel * m_frictionKv;         // find friction
    if (friction > m_frictionTorque)
        friction =m_frictionTorque;
	acc2 = (torque - friction - clutchForce) * oneOverMOI * CSim::RKh2;
    vel2 = vel;
    vel = vel1 + acc2;
}

void CMxEngine::RKStep3(float clutchForce)
{
    float torque = GetTorque(vel) * m_rThrottlePos;      // torque from engine
    float friction = vel * m_frictionKv;         // find friction
    if (friction > m_frictionTorque)
        friction = m_frictionTorque;
	acc3 = (torque - friction - clutchForce) * oneOverMOI * CSim::RKh;
    vel3 = vel;
    vel = vel1 + acc3;
}

void CMxEngine::RKStep4(float clutchForce)
{
    float torque = GetTorque(vel) * m_rThrottlePos;      // torque from engine
    float friction = vel * m_frictionKv;         // find friction
    if (friction > m_frictionTorque)
        friction = m_frictionTorque;
    vel = vel1 + (acc1 + 2.0f * acc2 + acc3 + (torque - friction - clutchForce) * oneOverMOI * CSim::RKh2) * (1.0f / 3.0f);

	//add a revlimiter,
	if(vel*9.54929658551f >m_rRevLimit)
	{
		vel = (m_rRevLimit-200.0f)*0.10471975512f;
	}

	if(vel*9.54929658551f <800.0f)//idle controller
	{
 		vel = (800.0f)*0.10471975512f;
	}

	if(vel<0.0f)
		vel = 0.0f;
	

}		

void CMxEngine::undo(void)
{
    vel = vel0;
}

float CMxEngine::GetMaxTorque(float omegaE)
{
	//for now use old shit. later normalize to 1 and have a maxTorque constant

	omegaE *= 0.00954929658551f ;//convert to rpm/1000

	int index = (int) floor(omegaE);
	float factor = omegaE-(float)index;
	float torqueA,torqueB;

	if(omegaE>0)
	{
		if(omegaE<9)
		{
			torqueA = m_rTorqueCurve[index];
			torqueB = m_rTorqueCurve[index+1];
		}
		else
		{
			torqueA = m_rTorqueCurve[9];
			torqueB = m_rTorqueCurve[9];
		}
	}
	else
	{
		torqueA = 0;
		torqueB = 0;
	}
//	torqueA+(torqueB-torqueA)*factor;// + torqueB*(1.0f-factor);
 	

	///sanity check
	if (factor >1)
		factor = 1;
	return (torqueA+(torqueB-torqueA)*factor);

}
float CMxEngine::GetMinTorque(float omegaE)
{ //throttle is 0, induces engine braking

	omegaE*= 0.1591549430918f;//revs per second
    return (-m_fEngineBrakeCoef*omegaE);
}

float CMxEngine::DoForces(float omegaE)
{
	m_fMinTorque = GetMinTorque(omegaE);
    m_fMaxTorque = GetMaxTorque(omegaE);
	m_rTorque=(m_fMaxTorque-m_fMinTorque)*m_rThrottlePos+m_fMinTorque;
	return m_rTorque;
}

void CMxEngine::DoStarter()
{ //temp to get engine out of stall range
	if(vel<=0)
		vel =300.0f;
	//acc1  = 50;

}

//void CMxEngine::forces(float omegaD)//drive shaft velocity (from wheels)
//{
//	
////float omegaD = pCar->getDriveVel();
//
//	if(omegaD>1000.0f)//bound the drive velocity... could explode
//		omegaD=1000.0f;
//	
// 	float omegaG; //gearbox velocity
//	
//    if (m_iCurrentGear >= 1) {                         // forward   
//        omegaG = omegaD * m_rFowardGearRatios[m_iCurrentGear-1];
//         //m_fClutchTorque = m_rClutchPos*m_fClutchKv * (vel - omegaG);//vel = engine's velocity
//		m_fClutchTorque = m_rClutchPos*m_fClutchKv * (vel - omegaG);
//        m_fDriveshaftTorque =m_fClutchTorque * m_rFowardGearRatios[m_iCurrentGear - 1];
//
//    }
//    else if (m_iCurrentGear < 0) {                     // reverse
//        omegaG = omegaD * m_rReverseGearRatio;
//        m_fClutchTorque = m_rClutchPos*m_fClutchKv * (vel - omegaG);
//        m_fDriveshaftTorque = m_fClutchTorque * m_rReverseGearRatio;
//    }
//    else {                                          // neutral
//        m_fClutchTorque = m_fDriveshaftTorque = 0.0;
//	}
//
//	if(	m_rBrakePos>m_rAutoClutchBrake)
//	{
//	  m_fClutchTorque = m_fDriveshaftTorque = 0.0;
//	}
// 
//}