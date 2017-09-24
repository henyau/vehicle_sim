#include "StdAfx.h"
//#include "MxCarEditorDoc.h"
#include "sim.h"
//Class encompasses entire physical simulation.
//Creates ODE world, space, bodies, geoms, 


CSim* CSim::m_Inst = NULL;	// Global instance
dContactGeom CSim::debugContact;

float CSim::RKh = 0.0;
float CSim::RKh2 = 0.0;
float CSim::RKh4 = 0.0;
float CSim::RKh6 = 0.0;






CSim::CSim()
{
	m_iPlayerNumber = 0;

	m_odeWorld = dWorldCreate();
	//m_odeSpace = dHashSpaceCreate (0);
	//m_odeSpace =dSimpleSpaceCreate(0);
	dVector3 Center;
	Center[0] = 0;
	Center[1] = 0;
	Center[2] = 0;
	Center[3] = 1;
	dVector3 Extents;
	Extents[0] =1000;
	Extents[1] = 500;
	Extents[2] = 1000;
	Extents[3] = 1;

	Matrix3x3 Ident3x3;
	//Ident3x3 = new Matrix3x3;
	Ident3x3 = Ident3x3.I;

	m_odeSpace = dQuadTreeSpaceCreate (0,  Center, Extents, 5);
	dWorldSetGravity(m_odeWorld, 0, -9.81f, 0);
	m_odeContactGroup = dJointGroupCreate(0);
	m_Track = new CMxTrack(m_odeSpace, "./Media/Max.trk.odeGeom", "./Media/Max.trk.tireGeom");
	
	simTimer = new CMxTimer;
	//m_iNumCars =1;
	Init(); //fill m_CarAttribute with some initial junko.


	//add some test objects
	//m_Car = new CMxCar( m_odeWorld, m_odeSpace, 0.0f, 3.0f, 0.0f);
	//m_Car = new CMxCar[m_iNumCars];

	//for(int i = 0; i<(int)m_iNumCars; i++)
	{		
		//m_Car[i] = new CMxCar( m_CarAttribute,m_odeWorld, m_odeSpace, 0.0f, 3.0f, i*10.0f, i);		
	//m_Car.push_back(new CMxCar( m_CarAttribute,m_odeWorld, m_odeSpace, 0.0f, 3.0f, i*10.0f, i));
	

//	 AddCar(m_CarAttribute ,  Vector3p(0.0f, 15.0f, 15.0f) , Ident3x3, Vector3p(0,0,0));
		AddCar(m_CarAttribute ,  Vector3p(0.0f, 15.0f, 55.0f) , Ident3x3, Vector3p(0,0,0));

	}
	//m_Bike[0] = new CMxBike( m_CarAttribute,m_odeWorld, m_odeSpace, 0.0f, 3.0f, 0.0f, i);


	//m_Car2 = new CMxCar( m_odeWorld, m_odeSpace, 0.0f, 3.0f, 15.0f);
	//	collisionTrack = new CCollisionTrack;
	//	collisionTrack->load("Media/Max.trk.col");

	m_Inst = this;	
	dWorldSetQuickStepNumIterations(m_odeWorld,20);//default is 20

	///MFC timer physics smoother.. wont need for real win32 app, but for now...
	m_uiIterationsOld = 0;
	m_uiNumFrameSameRate = 10;

	///
	for(int i = 0; i<5; i++)//m_Boxes.size()
		AddBox(Vector3p(1,1,1),20 ,Vector3p(0,5,5), Matrix3x3(), Vector3p(0,0,0));
		m_Boxes.push_back(new CMxPhysObject(m_odeWorld, m_odeSpace, 10.1f, 0.2f, 0.4f, 0.2f, 0, 5.0f, i*2.0f)); 

	m_iMaxSimTimePerFrame =20; //in ms, the max number of ms that can be processed during 1 gfx(evolveToNow) frame;
 	m_iTimePerStepMS =2;//in ms
	m_fTimePerStepSec = 0.001f*(float) m_iTimePerStepMS;//in seconds

}


CSim::~CSim(void)
{
	delete m_Track;
	//for(int i = 0 ; i < (int) m_iNumCars; i++)
	//	delete m_Car[i];
	m_Car.clear();

//	delete collisionTrack;
}

void CSim::SetSimTime(int maxSimTimePerFrame, int timePerStep)
{
	m_iMaxSimTimePerFrame =maxSimTimePerFrame; //in ms, the max number of ms that can be processed during 1 gfx(evolveToNow) frame;
 	m_iTimePerStepMS = timePerStep;//in ms
	m_fTimePerStepSec = 0.001f*(float) m_iTimePerStepMS;//in seconds

}
void CSim::Init()
{

	simTimer->Start();
    m_CarAttribute.m_Body.rBodyLength = 4.45f;
    m_CarAttribute.m_Body.rBodyWidth  = 4.75f;
    m_CarAttribute.m_Body.rBodyHeight = 4.40f;
    m_CarAttribute.m_Body.iBodyMass   = 1400.0f;
    m_CarAttribute.m_Body.rPosOfEngine[0]     = 0.0f;
    m_CarAttribute.m_Body.rPosOfEngine[1]     = -0.5f;
    m_CarAttribute.m_Body.rPosOfEngine[2]     = 0.0f;
    //m_CarAttribute.m_Body.rWheelBase  = 2.70f;
	m_CarAttribute.m_Body.rDistToFront =1.3f;
	m_CarAttribute.m_Body.rDistToRear = 1.4f;
	m_CarAttribute.m_Body.rFrontSuspHeight = -.5f;
	m_CarAttribute.m_Body.rRearSuspHeight = -.5f;

    m_CarAttribute.m_Body.rFrontWheelBase     = 1.40f;
    m_CarAttribute.m_Body.rRearWheelBase      = 1.45f;
    m_CarAttribute.m_Body.rDragCoef           = 0.29f;
    m_CarAttribute.m_Body.rFrontDownForceCoef = 0.01f;
    m_CarAttribute.m_Body.rRearDownForceCoef  = 0.01f;

    for ( int i = 0; i < 2; i++)
    {
        m_CarAttribute.m_Wheel[i].rWidth  = 0.225f;    // 單位是公尺
        m_CarAttribute.m_Wheel[i].rRadius = 0.60f;
        m_CarAttribute.m_Wheel[i].rMass   = 20.0f;     // 單位是Kg

        m_CarAttribute.m_Tire[i].rBrakeForce         = 2000.0f;
        m_CarAttribute.m_Tire[i].rCorneringStiffness = 5000.0f;
		m_CarAttribute.m_Tire[i].rLateralStiffness = 2800.0f;
        m_CarAttribute.m_Tire[i].rPrimaryFriction    = 6000.0f;

        m_CarAttribute.m_Suspension[i].rKp = 50000.0f;
        m_CarAttribute.m_Suspension[i].rKdBounce = 5000.0f;
		m_CarAttribute.m_Suspension[i].rKdRebound = 5000.0f;
        m_CarAttribute.m_Suspension[i].rUpLimit = -0.5f;
        m_CarAttribute.m_Suspension[i].rLoLimit = 0.5f;
		
		m_CarAttribute.m_Suspension[i].rLength = 1.0f;        

    }

	m_CarAttribute.m_Engine.rMass =  200.0f;
	m_CarAttribute.m_Engine.rLength =  0.4f;
	m_CarAttribute.m_Engine.rWidth =  0.3f;
	m_CarAttribute.m_Engine.rHeight =  0.2f;

    m_CarAttribute.m_Engine.rMaxRPM         = 8200.0f;
    m_CarAttribute.m_Engine.rTorqueCurve[0] = 10.0f;   // 單位是Kg-M
    m_CarAttribute.m_Engine.rTorqueCurve[1] = 20.0f;
    m_CarAttribute.m_Engine.rTorqueCurve[2] = 35.7f;
    m_CarAttribute.m_Engine.rTorqueCurve[3] = 35.7f;
    m_CarAttribute.m_Engine.rTorqueCurve[4] = 35.7f;
    m_CarAttribute.m_Engine.rTorqueCurve[5] = 35.7f;
    m_CarAttribute.m_Engine.rTorqueCurve[6] = 35.7f;
    m_CarAttribute.m_Engine.rTorqueCurve[7] = 35.7f;
    m_CarAttribute.m_Engine.rTorqueCurve[8] = 35.7f;
    m_CarAttribute.m_Engine.rTorqueCurve[9] = 33.0f;

    m_CarAttribute.m_GearBox.iMode              = 0;     // 自排
    m_CarAttribute.m_GearBox.iDriveType         = 1;     // 後驅
    m_CarAttribute.m_GearBox.rAWDRearPowerRatio = 0.0f;  // AWD的後輪驅動力比例
    m_CarAttribute.m_GearBox.rEfficiency        = 0.9f;  // 變速箱的傳動效率
    m_CarAttribute.m_GearBox.rEngineBrakeCoef   = 1.0f;  // 引擎剎車係數
    m_CarAttribute.m_GearBox.iTotalGear         = 6;     // 總檔位
    m_CarAttribute.m_GearBox.rFGearRatio[0]     = 4.0f;  // 1檔齒輪比
    m_CarAttribute.m_GearBox.rFGearRatio[1]     = 2.5f;
    m_CarAttribute.m_GearBox.rFGearRatio[2]     = 2.0f;
    m_CarAttribute.m_GearBox.rFGearRatio[3]     = 1.25f;
    m_CarAttribute.m_GearBox.rFGearRatio[4]     = 1.0f;
    m_CarAttribute.m_GearBox.rFGearRatio[5]     = 0.8f;
    m_CarAttribute.m_GearBox.rReverseGearRatio  = 2.85f; // 倒檔齒輪比
    m_CarAttribute.m_GearBox.rFinalDrive        = 2.50f; // 終傳比

    m_CarAttribute.m_rMaxSteeringAngle    = 50.0f;
    m_CarAttribute.m_rSteeringSensitility = 0.5f;

	m_CarAttribute.m_Engine.rMaxFrictionTorque = 1000.0f;
	m_CarAttribute.m_Engine.rMomentOfInertia = 1000.0f;
	m_CarAttribute.m_Engine.rEngineFriction = 10.0f;

	m_CarAttribute.m_GearBox.rClutchFriction = 10.0f;

	m_CarAttribute.m_Tire[0].rKp = 100000.0f;
	m_CarAttribute.m_Tire[1].rKp = 100000.0f;

	m_CarAttribute.m_Tire[0].rPeakBrakingCoef = 5000.0f;
	m_CarAttribute.m_Tire[1].rPeakBrakingCoef = 5000.0f;

	m_CarAttribute.m_GearBox.rAutoClutchBrake = 0.3f;
	m_CarAttribute.m_rHandBrakeForce = 2000.0f;

	m_CarAttribute.m_Wheel[0].rCamber = 0.0f;
	m_CarAttribute.m_Wheel[1].rCamber = 0.0f;

	m_CarAttribute.m_Wheel[0].rToe = 0.0f;
	m_CarAttribute.m_Wheel[1].rToe = 0.0f;
	
	m_CarAttribute.m_Suspension[0].rARB  = 0.0f;
	m_CarAttribute.m_Suspension[1].rARB  = 0.0f;

	m_CarAttribute.m_Body.rBodyInertial[0] = 1;
	m_CarAttribute.m_Body.rBodyInertial[1] = 1;
	m_CarAttribute.m_Body.rBodyInertial[2] = 1;

	m_CarAttribute.m_Body.rCoM[0] = 1;
	m_CarAttribute.m_Body.rCoM[1] = 1;
	m_CarAttribute.m_Body.rCoM[2] = 1;

	//m_CarAttribute.m_rRollingCoefSpeed = 1;
	//m_CarAttribute.m_rRollingCoefBasic = 1;

	//m_CarAttribute.m_Differentials = 0;
	//m_CarAttribute.M_GearBox = 0;


}
void CSim::Reload(Vector3p position, Quaternionp orientation)
{
	//extern CMxCarAttribute m_CarAttr;
}


void CSim::SetControlInputs(int whichCar, float elapsed, float throttlePos, float brakePos, float steeringwheelPos, bool shiftUp, bool shiftDown,  float _clutchPos, float _handBrakePos)
{ 
	//for(int i = 0; i< m_iNumCars; i++)
	{
		m_Car[whichCar]->setSteering(steeringwheelPos);
			//m_Car[whichCar]->mTransmission->m_rClutchPos = _clutchPos; 
		
			m_Car[whichCar]->mTransmission->m_rBrakePos = brakePos; // can put these into beginUpdate
			m_Car[whichCar]->mTransmission->m_rHandBrakePos = _handBrakePos;

			//for automatic transmission
		if(m_Car[whichCar]->mTransmission->m_iAutoTrans == 0 && m_Car[whichCar]->mTransmission->m_iCurrentGear > 0)
		{
			if(m_Car[whichCar]->mEngine->vel*9.54929658551f >m_Car[whichCar]->mTransmission->m_AutoShiftUp
				&&(!m_Car[whichCar]->mTransmission->m_bOldShiftDownButton)
				&&(m_Car[whichCar]->mTransmission->m_rClutchPos!=0)
				)
			{	shiftUp = true;
				shiftDown = false;
			}
			else if(m_Car[whichCar]->mEngine->vel*9.54929658551f< m_Car[whichCar]->mTransmission->m_AutoShiftDown
				&&!(m_Car[whichCar]->mTransmission->m_bOldShiftUpButton)
				&& m_Car[whichCar]->mTransmission->m_iCurrentGear > 1
				)
			{	shiftUp = false;
				shiftDown = true;
			}
//			else 
//				shiftUp= shiftDown = false;
			m_Car[whichCar]->mTransmission->BeginUpdate(throttlePos,brakePos, _clutchPos ,shiftUp, shiftDown, simTimer->GetSimTime()); 
		}
		else
			m_Car[whichCar]->mTransmission->BeginUpdate(throttlePos,brakePos, _clutchPos ,shiftUp, shiftDown, simTimer->GetSimTime()); 

		//test traction control // acording to RCVD, TC uses the brake to slow the wheel down then eases off the throttle to prevent over heating
		//for(int i =0; i<4; i++)
		//{
		//	if(m_Car[whichCar]->mWheels[i]->slipRatioPos>1.2f)
		//	{
		//		//if(m_Car[whichCar]->mWheels[i]->rotVel>0)
		//			m_Car[whichCar]->addWheelBrakingTorque(i, -m_Car[whichCar]->mWheels[i]->slipRatioPos*1000.0f);
		//		/*else
		//			m_Car[whichCar]->addWheelBrakingTorque(i, m_Car[whichCar]->mWheels[i]->slipRatioPos*1000.0f);*/
		//	}// that was easy...
		//	
		//}
		//now for stability control... I think we should look at the yaw rate then adjust brakes
		// or difference in slip in powered  wheels then add -torque to faster one...
	/*	if(m_Car[whichCar]->)
		{
			if(m_Car[whichCar]->mWheels[i]->rotVel>0)
				m_Car[whichCar]->addWheelBrakingTorque(i, -m_Car[whichCar]->mWheels[i]->slipRatioPos*1000.0f);
				else
					m_Car[whichCar]->addWheelBrakingTorque(i, m_Car[whichCar]->mWheels[i]->slipRatioPos*1000.0f);
		}*/
	
		//temp-need two clutch states in future..test for now
			if(m_Car[whichCar]->mTransmission->m_iCurrentGear == 0
				||(brakePos>m_Car[whichCar]->mTransmission->m_rAutoClutchBrake  && m_Car[whichCar]->mEngine->vel < 1560.0f)||_handBrakePos>0)
				m_Car[whichCar]->mTransmission->m_rClutchPos =0;
			//


		m_Car[whichCar]->frontBrakeTorque = brakePos*m_Car[whichCar]->maxFrontBrakeTorque;
		m_Car[whichCar]->backBrakeTorque = brakePos*m_Car[whichCar]->maxBackBrakeTorque;
		if(_handBrakePos>0.0f)
		{
			if(m_Car[whichCar]->backBrakeTorque+m_Car[whichCar]->maxHandBrakeTorque>m_Car[whichCar]->maxBackBrakeTorque)
				m_Car[whichCar]->backBrakeTorque = m_Car[whichCar]->maxBackBrakeTorque;
			else
				m_Car[whichCar]->backBrakeTorque += m_Car[whichCar]->maxHandBrakeTorque;
		}
	}

	
	//m_Car[m_iPlayerNumber]->setSteering(steeringwheelPos);
	//m_Car[m_iPlayerNumber]->mTransmission->BeginUpdate(throttlePos,brakePos, shiftUp, shiftDown, clutchEngaged); 
 //	//m_Car[m_iPlayerNumber]->mEngine->beginUpdate(throttlePos,brakePos, shiftUp, shiftDown, clutchEngaged); 
	//
	//
	//m_Car[m_iPlayerNumber]->frontBrakeTorque = brakePos*m_Car[m_iPlayerNumber]->maxBackBrakeTorque;
	//m_Car[m_iPlayerNumber]->backBrakeTorque = brakePos*m_Car[m_iPlayerNumber]->maxFrontBrakeTorque;

	//dBodyAddForce(m_Car[m_iPlayerNumber]->mCarbody.odeBody, 0,throttlePos*150000,0);
	//dBodyAddForce(m_Car[m_iPlayerNumber]->mCarbody.odeBody, 0,-brakePos*150000, 0);
 

	////TEST, move to Evolve when everything works ok.
 //   float *planeEq, t;
 //   for (int i = 0; i < 4; i++) {
	//	m_Car[m_iPlayerNumber]->mWheels[i]->trackContact.move(
	//		m_Car[m_iPlayerNumber]->mWheels[i]->positionWld.x, 
	//		m_Car[m_iPlayerNumber]->mWheels[i]->positionWld.z,
	//		//m_Car[m_iPlayerNumber]->mWheels[i]->trackContact.x, 
	//		//m_Car[m_iPlayerNumber]->mWheels[i]->trackContact.z,
	//		planeEq, t);
	//	m_Car[m_iPlayerNumber]->mWheels[i]->trackTirePlane = (m_Car[m_iPlayerNumber]->mWheels[i]->trackContact.face->p);//&[i].face->p);
 //   }//move tires' contacts

 // 	m_Car[m_iPlayerNumber]->setSteering(steeringwheelPos);

	////test input
 //	//dBodyAddForce(m_Car[m_iPlayerNumber]->mCarbody.odeBody, 0,0,throttlePos*180000);
 // 	//dBodyAddForce(m_Car[m_iPlayerNumber]->mCarbody.odeBody, 0,0,-brakePos*180000);
 //      
 //	m_Car[m_iPlayerNumber]->mEngine->beginUpdate(throttlePos); 

	//m_Car[m_iPlayerNumber]->frontBrakeTorque = brakePos*m_Car[m_iPlayerNumber]->maxBackBrakeTorque;
	//m_Car[m_iPlayerNumber]->backBrakeTorque = brakePos*m_Car[m_iPlayerNumber]->maxFrontBrakeTorque;

 // 	//m_Car[m_iPlayerNumber]->addDriveTorque(throttlePos*500000);
	///*m_Car[m_iPlayerNumber]->mWheels[0]->rotVel= throttlePos*100;
	//m_Car[m_iPlayerNumber]->mWheels[1]->rotVel = throttlePos*100;	
	//m_Car[m_iPlayerNumber]->mWheels[2]->rotVel = throttlePos*100;
 //	m_Car[m_iPlayerNumber]->mWheels[3]->rotVel= throttlePos*100;*/


	//////do evolve step	//not used after replaced with plane equations
	////
 //   //dBodyAddRelTorque(m_Car[m_iPlayerNumber]->mCarbody.odeBody, steeringwheelPos*1000, 0,0);
	////if(gWheelContact.g1 == m_Car[m_iPlayerNumber]->mOdeWheelGeom[0]||
	////	gWheelContact.g1 == m_Car[m_iPlayerNumber]->mOdeWheelGeom[1]||
	////	gWheelContact.g1 == m_Car[m_iPlayerNumber]->mOdeWheelGeom[2]||
	////	gWheelContact.g1 == m_Car[m_iPlayerNumber]->mOdeWheelGeom[3])
	///*
 //	if(gWheelContact0.depth!=0) 
	//{
 // 		Vector3p vforce(gWheelContact0.normal[0], gWheelContact0.normal[1], gWheelContact0.normal[2]);
	//	vforce.mult(gWheelContact0.depth*0 + m_Car[m_iPlayerNumber]->mCarbody.mass*9.81f*0.25f);// add bias....test now
	//	m_Car[m_iPlayerNumber]->addBodyForce(gWheelContact0.pos, vforce);
	//}
 //	if(gWheelContact1.depth!=0)
	//{
	//	Vector3p vforce1(gWheelContact1.normal[0], gWheelContact1.normal[1], gWheelContact1.normal[2]);
	//	vforce1.mult(gWheelContact1.depth*0 + m_Car[m_iPlayerNumber]->mCarbody.mass*9.81f*0.25f);// add bias....test now
	//	m_Car[m_iPlayerNumber]->addBodyForce(gWheelContact1.pos, vforce1);
	//}
	//if(gWheelContact2.depth!=0)
	//{
	//	Vector3p vforce2(gWheelContact2.normal[0], gWheelContact2.normal[1], gWheelContact2.normal[2]);
	//	vforce2.mult(gWheelContact2.depth*0 + m_Car[m_iPlayerNumber]->mCarbody.mass*9.81f*0.25f);// add bias....test now
	//	m_Car[m_iPlayerNumber]->addBodyForce(gWheelContact2.pos, vforce2);
	//}
	//if(gWheelContact3.depth!=0)
	//{
	//	Vector3p vforce3(gWheelContact3.normal[0], gWheelContact3.normal[1], gWheelContact3.normal[2]);
	//	vforce3.mult(gWheelContact3.depth*0 + m_Car[m_iPlayerNumber]->mCarbody.mass*9.81f*0.25f);// add bias....test now
 //		m_Car[m_iPlayerNumber]->addBodyForce(gWheelContact3.pos, vforce3);
	//}
	//*/
	//
	//for(int i = 0 ; i < 4; i++)
	//{
	//	m_Car[m_iPlayerNumber]->mSusp[i]->Update();//update before m_Car[m_iPlayerNumber]->RKInit since, linear position...
	//	m_Car[m_iPlayerNumber]->mWheels[i]->Update();
	//}

	//
	//m_Car[m_iPlayerNumber]->mCarbody.setStepSize(elapsed);
 //	//m_Car[m_iPlayerNumber]->mCarbody.Update();//test updated elsewhere
	////m_Car[m_iPlayerNumber]->forces();


 //	m_Car[m_iPlayerNumber]->RKInit();
	//for(i = 0 ; i<4; i++)
	//	m_Car[m_iPlayerNumber]->mWheels[i]->RKInit();
	//
	//m_Car[m_iPlayerNumber]->forces();	
	//m_Car[m_iPlayerNumber]->RKStep1();
	//for(i = 0 ; i<4; i++)
	//	m_Car[m_iPlayerNumber]->mWheels[i]->RKStep1();//really tire slip vars
	//m_Car[m_iPlayerNumber]->mEngine->RKStep1();

	//m_Car[m_iPlayerNumber]->forces();	
	//m_Car[m_iPlayerNumber]->RKStep2();
	//for(i = 0 ; i<4; i++)
	//	m_Car[m_iPlayerNumber]->mWheels[i]->RKStep2();
	//m_Car[m_iPlayerNumber]->mEngine->RKStep2();

	//m_Car[m_iPlayerNumber]->forces();	
	//m_Car[m_iPlayerNumber]->RKStep3();
	//for(i = 0 ; i<4; i++)
	//	m_Car[m_iPlayerNumber]->mWheels[i]->RKStep3();
	//m_Car[m_iPlayerNumber]->mEngine->RKStep3();

	//m_Car[m_iPlayerNumber]->forces();	
	//m_Car[m_iPlayerNumber]->RKStep4();
	//for(i = 0 ; i<4; i++)
	//	m_Car[m_iPlayerNumber]->mWheels[i]->RKStep4();
	//m_Car[m_iPlayerNumber]->mEngine->RKStep4();

	////dSpaceCollide(m_odeSpace,0,nearCallback2);	
 ////	dWorldStep(m_odeWorld, elapsed );
	////dJointGroupEmpty(m_odeContactGroup);


	///*
	//m_TestCar->m_rSteeringWheelAngle = steeringwheelPos;
	//m_TestCar->m_Engine.m_rBrakePos = brakePos;
	//m_TestCar->m_Engine.m_rThrottlePos = throttlePos;
	//*/
 }
void CSim::EvolveToNow()
{//Run the simulation until simTim = realTime or maxSimTimePerFrame
	int    lastTime;
	int    curTime,diffTime,maxTime;
 
	// Calculate time from last to current situation
	simTimer->Update();
	curTime=simTimer->GetRealTime();
	lastTime=simTimer->GetLastSimTime();// the current time of simulation
	diffTime=curTime-lastTime;//how far are we behind
	
	maxTime=lastTime+m_iMaxSimTimePerFrame;//how far can we proceed

	//if(diffTime>m_iMaxSimTimePerFrame)//too far behind
	//{
	//	curTime=maxTime;//don't want to do too many steps per frame
	//}
	  
	// Calculate physics until we reach the current time(or maxTime)
	m_uiIterations=0;
	//while(simTimer->GetSimTime()<curTime  ) //the object frame isn't updated fast enough
	

 	while( m_uiIterations<10)//fixed for now
	{
		m_uiIterations++;			
		Update(m_fTimePerStepSec); //evolve the simulation
		
 		simTimer->SetSpan(m_iTimePerStepMS);
		simTimer->AddSimTime(simTimer->GetTimeSpan());
		 	
	}	
	//while(simTimer->GetSimTime()<curTime  )
	//{
	//	m_uiIterations++;			
	//	Update(m_fTimePerStepSec); //evolve the simulation
	//	
 //		simTimer->SetSpan(m_iTimePerStepMS);
	//	simTimer->AddSimTime(simTimer->GetTimeSpan());
	//	 	
	//}	
//	else // caught up to real time
//	{
////		simTimer->AddSimTime(-maxSimTimePerFrame);	//slow sim time so we can run at limit
//		while(simTimer->GetSimTime()<curTime-(maxSimTimePerFrame) )
//		{
//			m_uiIterations++;
//			
//			Update(fTimePerStepSec);//some step size...hard code for now i guess
// 			simTimer->SetSpan(iTimePerStepMS);
//			simTimer->AddSimTime(simTimer->GetTimeSpan());
//		 	
//		}	
//	}

	if(m_uiIterationsOld == m_uiIterations)
		m_uiNumFrameSameRate++;
	else
		m_uiNumFrameSameRate = 0;

	m_uiIterationsOld = m_uiIterations;
	
	simTimer->SetLastSimTime();
}


void CSim::AddCar(CMxCarAttribute _CarAttrib, Vector3p position, Matrix3x3 orientation, Vector3p velocity)
{
	m_Car.push_back(new CMxCar( _CarAttrib,m_odeWorld, m_odeSpace, position.x, position.y, position.z, m_Car.size() ));

}
void CSim::AddBike(CMxCarAttribute _BikeAttrib, Vector3p position, Matrix3x3 orientation, Vector3p velocity)
{}
void CSim::AddBox(Vector3p size, float mass,Vector3p position, Matrix3x3 orientation, Vector3p velocity)
{
	m_Boxes.push_back(new CMxPhysObject(m_odeWorld, m_odeSpace, mass, size.x, size.y, size.z, position.x, position.y,position.z)); 
}
void CSim::AddParticleSystem()
{}
void CSim::RemoveCar(int carNumber)
{}
void CSim::RemoveBike(int bikeNumber)
{}
void CSim::RemoveBox(int boxNumber)
{}
void CSim::RemoveParticleSystem()
{}


void CSim::Update(float stepsize)
{
	RKh = stepsize;
	RKh2 = stepsize / 2.0f;
	RKh4 = stepsize / 4.0f;
	RKh6 = stepsize / 6.0f;

	//update tire contact
	for(int i = 0; i<m_Boxes.size(); i++)
		m_Boxes[i]->Update();
	for(unsigned int j = 0; j<m_Car.size(); j++)
	{
		Vector3p vDown;
		bool binvertDown;

		m_Car[j]->Update();//hardcode possible neighbors (might be looking at unwritten memory... be  careful
		for ( i = 0; i < 4; i++) //test current and neighbors 
		{
			vDown = m_Car[j]->mWheels[i]->m_vDown;
			binvertDown = false;
			findNeighborIntersect:

			if(m_Track->intersect_triangle_index(m_Car[j]->mWheels[i]->positionWld,
				vDown,
				m_Car[j]->mWheels[i]->m_tireContact.lastIndex,
				&m_Car[j]->mWheels[i]->m_tireContact.t,
				&m_Car[j]->mWheels[i]->m_tireContact.u, 
				&m_Car[j]->mWheels[i]->m_tireContact.v))//check if still on current triangle
			{
                goto findContact;				
			}
			else if(m_Track->intersect_triangle_index(m_Car[j]->mWheels[i]->positionWld,
				vDown,
				m_Car[j]->mWheels[i]->m_tireContact.lastIndex + 1,
				&m_Car[j]->mWheels[i]->m_tireContact.t,
				&m_Car[j]->mWheels[i]->m_tireContact.u, 
				&m_Car[j]->mWheels[i]->m_tireContact.v))//front
			{
				m_Car[j]->mWheels[i]->m_tireContact.triangleIndex = m_Car[j]->mWheels[i]->m_tireContact.lastIndex + 1;
				goto findContact;				
			}
			else if(m_Track->intersect_triangle_index(m_Car[j]->mWheels[i]->positionWld,
				vDown,
				m_Car[j]->mWheels[i]->m_tireContact.lastIndex-1,
				&m_Car[j]->mWheels[i]->m_tireContact.t,
				&m_Car[j]->mWheels[i]->m_tireContact.u, 
				&m_Car[j]->mWheels[i]->m_tireContact.v))//back
			{
				m_Car[j]->mWheels[i]->m_tireContact.triangleIndex = m_Car[j]->mWheels[i]->m_tireContact.lastIndex - 1;
				goto findContact;				
			}
			else if(m_Track->intersect_triangle_index(m_Car[j]->mWheels[i]->positionWld,
				vDown,
				(m_Car[j]->mWheels[i]->m_tireContact.lastIndex-1)+(m_Track->m_TireSplineGeom->m_Length*2),
				&m_Car[j]->mWheels[i]->m_tireContact.t,
				&m_Car[j]->mWheels[i]->m_tireContact.u, 
				&m_Car[j]->mWheels[i]->m_tireContact.v))//right
			{
				m_Car[j]->mWheels[i]->m_tireContact.triangleIndex = (m_Car[j]->mWheels[i]->m_tireContact.lastIndex-1)+(m_Track->m_TireSplineGeom->m_Length*2);
				goto findContact;				
			}
			else if(m_Track->intersect_triangle_index(m_Car[j]->mWheels[i]->positionWld,
				vDown,
				(m_Car[j]->mWheels[i]->m_tireContact.lastIndex+1)-(m_Track->m_TireSplineGeom->m_Length*2),
				&m_Car[j]->mWheels[i]->m_tireContact.t,
				&m_Car[j]->mWheels[i]->m_tireContact.u, 
				&m_Car[j]->mWheels[i]->m_tireContact.v))//left
			{
				m_Car[j]->mWheels[i]->m_tireContact.triangleIndex = (m_Car[j]->mWheels[i]->m_tireContact.lastIndex+1)-(m_Track->m_TireSplineGeom->m_Length*2);
				goto findContact;				
			}

			else if(m_Track->intersect_triangle_index(m_Car[j]->mWheels[i]->positionWld,
				vDown,
				(m_Car[j]->mWheels[i]->m_tireContact.lastIndex+2),
				&m_Car[j]->mWheels[i]->m_tireContact.t,
				&m_Car[j]->mWheels[i]->m_tireContact.u, 
				&m_Car[j]->mWheels[i]->m_tireContact.v))//next quad
			{
				m_Car[j]->mWheels[i]->m_tireContact.triangleIndex = (m_Car[j]->mWheels[i]->m_tireContact.lastIndex+2);
				goto findContact;				
			}
			else if(m_Track->intersect_triangle_index(m_Car[j]->mWheels[i]->positionWld,
				vDown,
				(m_Car[j]->mWheels[i]->m_tireContact.lastIndex-2),
				&m_Car[j]->mWheels[i]->m_tireContact.t,
				&m_Car[j]->mWheels[i]->m_tireContact.u, 
				&m_Car[j]->mWheels[i]->m_tireContact.v))//prev quad
			{
				m_Car[j]->mWheels[i]->m_tireContact.triangleIndex = (m_Car[j]->mWheels[i]->m_tireContact.lastIndex-2);
				goto findContact;				
			}
			else if(m_Track->intersect_triangle_index(m_Car[j]->mWheels[i]->positionWld,
				vDown,
				(m_Car[j]->mWheels[i]->m_tireContact.lastIndex+2)+(m_Track->m_TireSplineGeom->m_Length*2),
				&m_Car[j]->mWheels[i]->m_tireContact.t,
				&m_Car[j]->mWheels[i]->m_tireContact.u, 
				&m_Car[j]->mWheels[i]->m_tireContact.v))//right quad
			{
				m_Car[j]->mWheels[i]->m_tireContact.triangleIndex = (m_Car[j]->mWheels[i]->m_tireContact.lastIndex+2)+(m_Track->m_TireSplineGeom->m_Length*2);
				goto findContact;				
			}

			else if(m_Track->intersect_triangle_index(m_Car[j]->mWheels[i]->positionWld,
				vDown,
				(m_Car[j]->mWheels[i]->m_tireContact.lastIndex+2)-(m_Track->m_TireSplineGeom->m_Length*2),
				&m_Car[j]->mWheels[i]->m_tireContact.t,
				&m_Car[j]->mWheels[i]->m_tireContact.u, 
				&m_Car[j]->mWheels[i]->m_tireContact.v))//left quad
			{//ok
				m_Car[j]->mWheels[i]->m_tireContact.triangleIndex = (m_Car[j]->mWheels[i]->m_tireContact.lastIndex+2)-(m_Track->m_TireSplineGeom->m_Length*2);
				goto findContact;				
			}

			else if(m_Track->intersect_triangle_index(m_Car[j]->mWheels[i]->positionWld,
				vDown,
				(m_Car[j]->mWheels[i]->m_tireContact.lastIndex-2)+(m_Track->m_TireSplineGeom->m_Length*2),
				&m_Car[j]->mWheels[i]->m_tireContact.t,
				&m_Car[j]->mWheels[i]->m_tireContact.u, 
				&m_Car[j]->mWheels[i]->m_tireContact.v))//right corner
			{//ok
				m_Car[j]->mWheels[i]->m_tireContact.triangleIndex = (m_Car[j]->mWheels[i]->m_tireContact.lastIndex-2)+(m_Track->m_TireSplineGeom->m_Length*2);
				goto findContact;				
			}

			else if(m_Track->intersect_triangle_index(m_Car[j]->mWheels[i]->positionWld,
				vDown,
				(m_Car[j]->mWheels[i]->m_tireContact.lastIndex-2)-(m_Track->m_TireSplineGeom->m_Length*2),
				&m_Car[j]->mWheels[i]->m_tireContact.t,
				&m_Car[j]->mWheels[i]->m_tireContact.u, 
				&m_Car[j]->mWheels[i]->m_tireContact.v))//left corner
			{
				m_Car[j]->mWheels[i]->m_tireContact.triangleIndex = (m_Car[j]->mWheels[i]->m_tireContact.lastIndex-2)-(m_Track->m_TireSplineGeom->m_Length*2);
				goto findContact;				
			}
		else if(m_Track->intersect_triangle_index(m_Car[j]->mWheels[i]->positionWld,
				vDown,
				(m_Car[j]->mWheels[i]->m_tireContact.lastIndex-1)-(m_Track->m_TireSplineGeom->m_Length*2),
				&m_Car[j]->mWheels[i]->m_tireContact.t,
				&m_Car[j]->mWheels[i]->m_tireContact.u, 
				&m_Car[j]->mWheels[i]->m_tireContact.v))//left corner
			{
				m_Car[j]->mWheels[i]->m_tireContact.triangleIndex = (m_Car[j]->mWheels[i]->m_tireContact.lastIndex-1)-(m_Track->m_TireSplineGeom->m_Length*2);
				goto findContact;				
			}
			else if(m_Track->intersect_triangle_index(m_Car[j]->mWheels[i]->positionWld,
				vDown,
				(m_Car[j]->mWheels[i]->m_tireContact.lastIndex-3)-(m_Track->m_TireSplineGeom->m_Length*2),
				&m_Car[j]->mWheels[i]->m_tireContact.t,
				&m_Car[j]->mWheels[i]->m_tireContact.u, 
				&m_Car[j]->mWheels[i]->m_tireContact.v))//left corner
			{
				m_Car[j]->mWheels[i]->m_tireContact.triangleIndex = (m_Car[j]->mWheels[i]->m_tireContact.lastIndex-3)-(m_Track->m_TireSplineGeom->m_Length*2);
				goto findContact;				
			}
			else if(m_Track->intersect_triangle_index(m_Car[j]->mWheels[i]->positionWld,
				vDown,
				(m_Car[j]->mWheels[i]->m_tireContact.lastIndex)-(m_Track->m_TireSplineGeom->m_Length*2),
				&m_Car[j]->mWheels[i]->m_tireContact.t,
				&m_Car[j]->mWheels[i]->m_tireContact.u, 
				&m_Car[j]->mWheels[i]->m_tireContact.v))//left corner
			{//ok
				m_Car[j]->mWheels[i]->m_tireContact.triangleIndex = (m_Car[j]->mWheels[i]->m_tireContact.lastIndex)-(m_Track->m_TireSplineGeom->m_Length*2);
				goto findContact;				
			}
			else if(m_Track->intersect_triangle_index(m_Car[j]->mWheels[i]->positionWld,
				vDown,
				(m_Car[j]->mWheels[i]->m_tireContact.lastIndex)+(m_Track->m_TireSplineGeom->m_Length*2),
				&m_Car[j]->mWheels[i]->m_tireContact.t,
				&m_Car[j]->mWheels[i]->m_tireContact.u, 
				&m_Car[j]->mWheels[i]->m_tireContact.v))//left corner
			{//ok
				m_Car[j]->mWheels[i]->m_tireContact.triangleIndex = (m_Car[j]->mWheels[i]->m_tireContact.lastIndex)+(m_Track->m_TireSplineGeom->m_Length*2);
				goto findContact;				
			}
			else if(m_Track->intersect_triangle_index(m_Car[j]->mWheels[i]->positionWld,
				vDown,
				(m_Car[j]->mWheels[i]->m_tireContact.lastIndex)+(m_Track->m_TireSplineGeom->m_Length*2)+1,
				&m_Car[j]->mWheels[i]->m_tireContact.t,
				&m_Car[j]->mWheels[i]->m_tireContact.u, 
				&m_Car[j]->mWheels[i]->m_tireContact.v))//left corner
			{//ok
				m_Car[j]->mWheels[i]->m_tireContact.triangleIndex = (m_Car[j]->mWheels[i]->m_tireContact.lastIndex)+(m_Track->m_TireSplineGeom->m_Length*2)+1;
				goto findContact;				
			}
			else if(m_Track->intersect_triangle_index(m_Car[j]->mWheels[i]->positionWld,
				vDown,
				(m_Car[j]->mWheels[i]->m_tireContact.lastIndex)-(m_Track->m_TireSplineGeom->m_Length*2)-1,
				&m_Car[j]->mWheels[i]->m_tireContact.t,
				&m_Car[j]->mWheels[i]->m_tireContact.u, 
				&m_Car[j]->mWheels[i]->m_tireContact.v))//left corner
			{
				m_Car[j]->mWheels[i]->m_tireContact.triangleIndex = (m_Car[j]->mWheels[i]->m_tireContact.lastIndex)-(m_Track->m_TireSplineGeom->m_Length*2)-1;
				goto findContact;				
			}
			else if(m_Track->intersect_triangle_index(m_Car[j]->mWheels[i]->positionWld,
				vDown,
				(m_Car[j]->mWheels[i]->m_tireContact.lastIndex)-(m_Track->m_TireSplineGeom->m_Length*2)+3,
				&m_Car[j]->mWheels[i]->m_tireContact.t,
				&m_Car[j]->mWheels[i]->m_tireContact.u, 
				&m_Car[j]->mWheels[i]->m_tireContact.v))//left corner
			{
				m_Car[j]->mWheels[i]->m_tireContact.triangleIndex = (m_Car[j]->mWheels[i]->m_tireContact.lastIndex)-(m_Track->m_TireSplineGeom->m_Length*2)+3;
				goto findContact;				
			}

			
			else if(!binvertDown)
			{
				binvertDown = true;
				vDown.neg();
				goto findNeighborIntersect;
			
			}

			else//find which triangle wheel is on
			{
				m_Track->FindTriangleIntersect(m_Car[j]->mWheels[i]->positionWld,
				//	m_Car[j]->mCarbody.m_vDown, //using car down 
				m_Car[j]->mWheels[i]->m_vDown,
					&m_Car[j]->mWheels[i]->m_tireContact.t,
					&m_Car[j]->mWheels[i]->m_tireContact.u, 
					&m_Car[j]->mWheels[i]->m_tireContact.v, 
					&m_Car[j]->mWheels[i]->m_tireContact.normal,
					&m_Car[j]->mWheels[i]->m_tireContact.triangleIndex);
			}
			if(m_Car[j]->mWheels[i]->m_tireContact.triangleIndex ==-1)
			{//look on the other side
				m_Track->FindTriangleIntersect(m_Car[j]->mWheels[i]->positionWld,
				//	m_Car[j]->mCarbody.m_vDown, //using car down 
				vDown,
					&m_Car[j]->mWheels[i]->m_tireContact.t,
					&m_Car[j]->mWheels[i]->m_tireContact.u, 
					&m_Car[j]->mWheels[i]->m_tireContact.v, 
					&m_Car[j]->mWheels[i]->m_tireContact.normal,
					&m_Car[j]->mWheels[i]->m_tireContact.triangleIndex);
			
			}
findContact:
			if(m_Car[j]->mWheels[i]->m_tireContact.triangleIndex !=-1)
			{
				//if odd- flip UV

				if(m_Car[j]->mWheels[i]->m_tireContact.triangleIndex&1)
				{  		
					m_Car[j]->mWheels[i]->m_tireContact.u =1- m_Car[j]->mWheels[i]->m_tireContact.u;
					m_Car[j]->mWheels[i]->m_tireContact.v = 1-m_Car[j]->mWheels[i]->m_tireContact.v;
				}
			
				m_Car[j]->mWheels[i]->m_tireContact.fRollingResistanceFactor = m_Track->GetSufaceRollingResistance(m_Car[j]->mWheels[i]->m_tireContact.triangleIndex);
				m_Car[j]->mWheels[i]->m_tireContact.fSurfaceFriction = m_Track->GetSufaceFriction(m_Car[j]->mWheels[i]->m_tireContact.triangleIndex);
				m_Car[j]->mWheels[i]->m_tireContact.fSurfaceType = m_Track->m_TireSplineGeom->m_TrisArray[m_Car[j]->mWheels[i]->m_tireContact.triangleIndex].iSurfaceType;

				//begin iterate to get closer
  				for(int qq = 0; qq <10; qq++)
				{
					m_Track->IterateUV(m_Car[j]->mWheels[i]->positionWld,
					m_Car[j]->mCarbody.m_vDown, //using car down 
					m_Car[j]->mWheels[i]->m_tireContact.t,
					&m_Car[j]->mWheels[i]->m_tireContact.u, 
					&m_Car[j]->mWheels[i]->m_tireContact.v, 
					m_Car[j]->mWheels[i]->m_tireContact.triangleIndex);
				}
				if(	m_Car[j]->mWheels[i]->m_tireContact.u >1.0f)
					m_Car[j]->mWheels[i]->m_tireContact.u =1.0f;
				if(	m_Car[j]->mWheels[i]->m_tireContact.v >1.0f)
					m_Car[j]->mWheels[i]->m_tireContact.v =1.0f;
				//end iterate to get closer

				//set the normal from the interpolated spline normals(instead of triangle)
				m_Car[j]->mWheels[i]->m_tireContact.normal = m_Track->getSplineNormalAt(
					m_Car[j]->mWheels[i]->m_tireContact.triangleIndex, 
					m_Car[j]->mWheels[i]->m_tireContact.u,
					m_Car[j]->mWheels[i]->m_tireContact.v);

//				set the tire distance from spline(instead of triangle)
				m_Car[j]->mWheels[i]->m_tireContact.t =  m_Track->getDistanceToSpline( 
					m_Car[j]->mWheels[i]->positionWld, 
					m_Car[j]->mWheels[i]->m_tireContact.u, 
					m_Car[j]->mWheels[i]->m_tireContact.v, m_Car[j]->mWheels[i]->m_tireContact.triangleIndex);

				//Set contact surface type
				m_Car[j]->mWheels[i]->m_tireContact.fSurfaceFriction = m_Track->GetSufaceFriction(m_Car[j]->mWheels[i]->m_tireContact.triangleIndex);//m_Track->m_TireSplineGeom->m_TrisArray[m_Car[j]->mWheels[i]->m_tireContact.triangleIndex].iSurfaceType;

				//if surface type not asphalt, modulate distance by noise function;
				m_Car[j]->mWheels[i]->m_tireContact.t += m_Track->GetBumpAt(m_Car[j]->mWheels[i]->m_tireContact.triangleIndex, m_Car[j]->mWheels[i]->m_tireContact.u, m_Car[j]->mWheels[i]->m_tireContact.v);
					

				//cache current triangle
				m_Car[j]->mWheels[i]->m_tireContact.lastIndex = m_Car[j]->mWheels[i]->m_tireContact.triangleIndex;

				//temp debug spline trace
				int triIndex = m_Car[j]->mWheels[i]->m_tireContact.triangleIndex;
				m_Car[j]->mWheels[i]->latSplineIndex = triIndex / ((m_Track->m_TireSplineGeom->m_Length-1)*2);
				m_Car[j]->mWheels[i]->longSplineIndex =
							((triIndex-	(m_Car[j]->mWheels[i]->latSplineIndex)*((m_Track->m_TireSplineGeom->m_Length-1)*2)	)/2)%	(m_Track->m_TireSplineGeom->m_Length-1)+1;
				m_Car[j]->mWheels[i]->tempLeftSplineTrace = m_Track->m_Splines[m_Car[j]->mWheels[i]->latSplineIndex].getPosAt(m_Car[j]->mWheels[i]->longSplineIndex , m_Car[j]->mWheels[i]->m_tireContact.v);
				m_Car[j]->mWheels[i]->tempRightSplineTrace = m_Track->m_Splines[m_Car[j]->mWheels[i]->latSplineIndex+1].getPosAt(m_Car[j]->mWheels[i]->longSplineIndex , m_Car[j]->mWheels[i]->m_tireContact.v);
				//m_Car[j]->mWheels[i]->tempLeftSplineTrace.mac(m_Car[j]->mWheels[i]->positionWld, m_Car[j]->mCarbody.m_vDown, m_Car[j]->mWheels[i]->m_tireContact.t) ;
				//m_Car[j]->mWheels[i]->tempRightSplineTrace.mac(m_Car[j]->mWheels[i]->positionWld, m_Car[j]->mCarbody.m_vDown, m_Car[j]->mWheels[i]->m_tireContact.t);
			}

		}
        //end tire contacts
  	
  		//initialize integrators
 		m_Car[j]->RKInit();
		//do forces
		m_Car[j]->forces();	
		//do integration        
		m_Car[j]->mDriveline->EulerStep();		
		m_Car[j]->EulerStep();		
		for(i = 0 ; i<4; i++)
			m_Car[j]->mWheels[i]->EulerStep();

		//do RK-4
		//m_Car[j]->RKStep1();
		//m_Car[j]->mEngine->RKStep1(m_Car[j]->mTransmission->m_fClutchTorque);
		//for(i = 0 ; i<4; i++)
		//	m_Car[j]->mWheels[i]->RKStep1();//really tire slip vars

		//m_Car[j]->forces();	
		//m_Car[j]->RKStep2();
		//m_Car[j]->mEngine->RKStep2(m_Car[j]->mTransmission->m_fClutchTorque);	
		//for(i = 0 ; i<4; i++)
		//	m_Car[j]->mWheels[i]->RKStep2();
	

		//m_Car[j]->forces();	
		//m_Car[j]->RKStep3();
		//m_Car[j]->mEngine->RKStep3(m_Car[j]->mTransmission->m_fClutchTorque);
		//for(i = 0 ; i<4; i++)
		//	m_Car[j]->mWheels[i]->RKStep3();
		//

		//m_Car[j]->forces();	
		//m_Car[j]->RKStep4();
		//m_Car[j]->mEngine->RKStep4(m_Car[j]->mTransmission->m_fClutchTorque);
		//for(i = 0 ; i<4; i++)
		//	m_Car[j]->mWheels[i]->RKStep4();

		
		/*m_Car[j]->RKStep1();
		m_Car[j]->mEngine->RKStep1(m_Car[j]->mTransmission->m_fClutchTorque);
		for(i = 0 ; i<4; i++)
			m_Car[j]->mWheels[i]->RKStep1();//really tire slip vars

		m_Car[j]->forces();	
		m_Car[j]->RKStep2();
		m_Car[j]->mEngine->RKStep2(m_Car[j]->mTransmission->m_fClutchTorque);	
		for(i = 0 ; i<4; i++)
			m_Car[j]->mWheels[i]->RKStep2();
	

		m_Car[j]->forces();	
		m_Car[j]->RKStep3();
		m_Car[j]->mEngine->RKStep3(m_Car[j]->mTransmission->m_fClutchTorque);
		for(i = 0 ; i<4; i++)
			m_Car[j]->mWheels[i]->RKStep3();
		

		m_Car[j]->forces();	
		m_Car[j]->RKStep4();
		m_Car[j]->mEngine->RKStep4(m_Car[j]->mTransmission->m_fClutchTorque);
		for(i = 0 ; i<4; i++)
			m_Car[j]->mWheels[i]->RKStep4();
		*/

	}

	//do collision detection responces
	dSpaceCollide(m_odeSpace,0,nearCallback2);		
	//m_Car[0]->mCarbody.Update();//update forces' debug informations

	//apply forces to bodies
	//dWorldQuickStep(m_odeWorld, stepsize );
	dWorldStep(m_odeWorld, stepsize );
	//empty collisions
	dJointGroupEmpty(m_odeContactGroup); 
	
}

void nearCallback2 (void *data, dGeomID o1, dGeomID o2)//ODE collision call back. ignore tires. will remove tire Geoms and bodies later
{
  int i,n;
  
  //	COgreFramework *inst = COgreFramework::GetInstance();
  CSim *simInst = CSim::GetInstance();
   // only collide things with the ground
  int g1 = (o1 == simInst->m_Track->m_trackgeomID || o1 ==simInst->m_Track->m_trackgeomID);
  int g2 = (o2 == simInst->m_Track->m_trackgeomID || o2 == simInst->m_Track->m_trackgeomID);
  //if (!(g1 ^ g2))
	 // return;

     const int N = 5;
  dContact contact[N];
  n = dCollide (o1,o2,N,&contact[0].geom,sizeof(dContact));
//  if(o1== simInst->m_Car->mCarbody.odeGeom || o2 == simInst->m_Car->mCarbody.odeGeom)
  //testing. do something more efficient later

  //for(int k = 0; k<4; k++)
  //for(int h = 0; h<4; h++)
  // CARBODY TO CARBODY!!!!
  for(int j = 0; j <(int)simInst->m_Car.size();  j++)
	for(int j2 = 0; j2 <(int)simInst->m_Car.size();  j2++)
	{
 		if(o1== simInst->m_Car[j]->mCarbody.odeGeom && o2== simInst->m_Car[j2]->mCarbody.odeGeom )
		
	//	 if(o1== simInst->m_Car[0]->mCarbody.odeGeom || o2 == simInst->m_Car[0]->mCarbody.odeGeom)
		{
 		if (n > 0) 
		{
			for (i=0; i<n; i++)
			{
				contact[i].surface.mode=0;
				//contact[i].surface.mode|=dContactSoftCFM;
				//contact[i].surface.mode|=dContactSoftERP;
				contact[i].surface.mode|=dContactBounce;
				contact[i].surface.mode|=dContactApprox1;

				//contact[i].surface.mu=.5f;
			//	contact[i].surface.bounce= 0.1f;//cCar->GetBody()->GetCoeffRestitution();
				//contact[i].surface.bounce_vel=0.2f;
				//contact[i].surface.soft_cfm = 0.2f;//RMGR->carCFM;
				//contact[i].surface.soft_erp = 0.8f;//RMGR->carERP;
		        
     			contact[i].surface.mu=0.7f;
	 			contact[i].surface.bounce= 0.4f;//cCar->GetBody()->GetCoeffRestitution();
 				contact[i].surface.bounce_vel=10.2f;
				//contact[i].surface.soft_cfm = 0.2f;//RMGR->carCFM;
 				//contact[i].surface.soft_erp = 0.8f;//RMGR->carERP;

 				dJointID c = dJointCreateContact (simInst->m_odeWorld,simInst->m_odeContactGroup,&contact[i]);
				dJointAttach (c, dGeomGetBody(contact[i].geom.g1), dGeomGetBody(contact[i].geom.g2));


		/*     contact[i].surface.mode = dContactSlip1 | dContactSlip2 |
 			dContactSoftERP | dContactSoftCFM | dContactApprox1;
			contact[i].surface.mu = 0.0f;//dInfinity;
			contact[i].surface.slip1 = 0.0;
			contact[i].surface.slip2 = 0.0;
    				contact[i].surface.soft_erp = 0.8;
				contact[i].surface.soft_cfm = 0.000001;

			dJointID c = dJointCreateContact (simInst->m_odeWorld,simInst->m_odeContactGroup,&contact[i]);

			dJointAttach (c, dGeomGetBody(contact[i].geom.g1), dGeomGetBody(contact[i].geom.g2));

			*/  

			}
		}  

	}
//carbody to some rigid body (like a cone/fence/dog or something)
	//for(int jj = 0; jj <64;  jj++)
	//	if((o1== simInst->m_Boxes[jj]->odeGeom&&o2==  simInst->m_Car[0]->mCarbody.odeGeom) 
	//		||(o2== simInst->m_Boxes[jj]->odeGeom &&o1== simInst->m_Car[0]->mCarbody.odeGeom) )
	//		if (n > 0) 
	//			for (i=0; i<n; i++)
	//			{
	//				contact[i].surface.mode=0;
	//				contact[i].surface.mode|=dContactApprox1;
 //  					contact[i].surface.mu=0.7f;

	//				dJointID c = dJointCreateContact (simInst->m_odeWorld,simInst->m_odeContactGroup,&contact[i]);
	//				dJointAttach (c, dGeomGetBody(contact[i].geom.g1), dGeomGetBody(contact[i].geom.g2));
	//			}
	//
	//
	for( int jj = 0; jj <simInst->m_Boxes.size();  jj++)
		for( int jj2 = 0; jj2 <simInst->m_Boxes.size();  jj2++)
			if((o1== simInst->m_Boxes[jj]->odeGeom&&o2==  simInst->m_Boxes[jj2]->odeGeom) 
			||(o2== simInst->m_Boxes[jj]->odeGeom &&o1== simInst->m_Boxes[jj2]->odeGeom))
			if (n > 0) 
				for (i=0; i<n; i++)
				{
					contact[i].surface.mode=0;
					contact[i].surface.mode|=dContactApprox1;
   					contact[i].surface.mu=0.7f;

					dJointID c = dJointCreateContact (simInst->m_odeWorld,simInst->m_odeContactGroup,&contact[i]);
					dJointAttach (c, dGeomGetBody(contact[i].geom.g1), dGeomGetBody(contact[i].geom.g2));
				}
			

		//box to track
	for( jj = 0; jj <simInst->m_Boxes.size();  jj++)
		if(((o1== simInst->m_Boxes[jj]->odeGeom&&o2== simInst->m_Track->m_trackgeomID) ||(o2== simInst->m_Boxes[jj]->odeGeom &&o1==simInst->m_Track->m_trackgeomID) )
			||((o1== simInst->m_Boxes[jj]->odeGeom&&o2==  simInst->m_Car[0]->mCarbody.odeGeom) 
			||(o2== simInst->m_Boxes[jj]->odeGeom &&o1== simInst->m_Car[0]->mCarbody.odeGeom) ))

			if (n > 0) 
				for (i=0; i<n; i++)
				{
					contact[i].surface.mode=0;
					contact[i].surface.mode|=dContactApprox1;
   					contact[i].surface.mu=0.7f;

					dJointID c = dJointCreateContact (simInst->m_odeWorld,simInst->m_odeContactGroup,&contact[i]);
					dJointAttach (c, dGeomGetBody(contact[i].geom.g1), dGeomGetBody(contact[i].geom.g2));
				}
	

	//carbody to track
	  for(int j = 0; j <(int)simInst->m_Car.size();  j++)
  		 if((o1== simInst->m_Car[j]->mCarbody.odeGeom&&o2== simInst->m_Track->m_trackgeomID)
	  ||(o2== simInst->m_Car[j]->mCarbody.odeGeom&&o1==simInst->m_Track->m_trackgeomID) )
	  {
		  if (n > 0) 
			{
				for (i=0; i<n; i++)
				{
					contact[i].surface.mode=0;
					contact[i].surface.mode|=dContactApprox1;
   					contact[i].surface.mu=0.7f;
					dJointID c = dJointCreateContact (simInst->m_odeWorld,simInst->m_odeContactGroup,&contact[i]);
					dJointAttach (c, dGeomGetBody(contact[i].geom.g1), dGeomGetBody(contact[i].geom.g2));

				}
			}
	  
	  }
	  else//wheel to track contact.
  {
  	  if (n > 0) //Sphere/ Trimesh only look at first for now
	  {
	   for (i=0; i<n; i++)
		{
			contact[i].surface.mode=0;
			contact[i].surface.mode|=dContactApprox1;
  			contact[i].surface.mu=0.7f;

			//dJointID c = dJointCreateContact (simInst->m_odeWorld,simInst->m_odeContactGroup,&contact[i]);
			//dJointAttach (c, dGeomGetBody(contact[i].geom.g1), dGeomGetBody(contact[i].geom.g2));
		
		  /*if(simInst->m_Car->mOdeWheelGeom[0]== o1)
			simInst->gWheelContact0 = contact[0].geom;
		  else if(simInst->m_Car->mOdeWheelGeom[1] == o1)
			simInst->gWheelContact1 = contact[0].geom;
		  else if(simInst->m_Car->mOdeWheelGeom[2] == o1)
			simInst->gWheelContact2 = contact[0].geom;
		  else if(simInst->m_Car->mOdeWheelGeom[3] == o1)
			simInst->gWheelContact3 = contact[0].geom;*/
		  	  
//			if(simInst->m_Car->mOdeWheelGeom[i] == o1)
//				  simInst->m_Car->mWheels[i]->m_WheelContact = contact[0].geom;  
		  
		}
	  }
 	  else if (n ==0)//reset contacts.
	  {
		 /* for(int i = 0; i<4; i++)
			  simInst->m_Car->mWheels[i]->m_WheelContact.depth = 0.0f; */
			//simInst->gWheelContact0.depth = 0.0f;
			//simInst->gWheelContact1.depth = 0.0f;
			//simInst->gWheelContact2.depth = 0.0f;
			//simInst->gWheelContact3.depth = 0.0f;
	  }

  }  

  simInst->debugContact = contact[0].geom;
  
  }

}


