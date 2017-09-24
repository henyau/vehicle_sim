#include "StdAfx.h"
#include "Sim.h"
#include "MxCar.h"
//#include "meshinformer.h"

//#include "MxCarEditorDoc.h"
//#include "MxCarAttrib.h"

CMxCar::CMxCar()//default
{
    
}

CMxCar::CMxCar( CMxCarAttribute _CarAttrib, dWorldID odeWorld, dSpaceID odeSpace, float posx, float posy, float posz, unsigned int number)
{
	dQuaternion q;
	dMass m;
	int i;

	m_CarAttrib = _CarAttrib;
	m_iNumber = number;//m_iWhich car
	bTireInitialized = false;
	
	mCarbody.odeBody = dBodyCreate (odeWorld);
    
    dQFromAxisAndAngle (q,1,0,0,M_PI*0.5);
	dQFromAxisAndAngle (q,0,1,0,0);
	dMassSetBox (&m,1,_CarAttrib.m_Body.rBodyWidth, _CarAttrib.m_Body.rBodyHeight,_CarAttrib.m_Body.rBodyLength);
	dMassAdjust (&m,_CarAttrib.m_Body.iBodyMass);
    dBodySetMass (mCarbody.odeBody,&m);

	mCarbody.odeGeom = dCreateBox(odeSpace,_CarAttrib.m_Body.rBodyWidth, _CarAttrib.m_Body.rBodyHeight,_CarAttrib.m_Body.rBodyLength);
	dBodySetPosition(mCarbody.odeBody, posx, posy, posz);
	dGeomSetBody(mCarbody.odeGeom, mCarbody.odeBody);
	
	for(i = 0; i < 4; i++)
	{
		mSusp[i] = new CMxSusp(&m_CarAttrib, this, i);
		mWheels[i] = new CMxWheel(&m_CarAttrib, this, mSusp[i], i);		
	}
	
	
	mEngine = new CMxEngine(&m_CarAttrib, this);
	mTransmission = new CMxTransmission(&m_CarAttrib,mEngine);

	mDriveline = new CMxDriveline(&m_CarAttrib, mEngine, mTransmission);
	//mDifferential = new CMxDifferential(&m_CarAttrib, this);


	mAerodynamics = new CMxAero(&m_CarAttrib, this);

	
	
//	for(i = 0; i <4; i++){
//		mOdeWheelGeom[i] = dCreateSphere(odeSpace, _CarAttrib.m_Wheel[i/2].rRadius);
//	}
//	//don't use ODE for wheels anymore

	mOdeSpace = odeSpace;
	mOdeWorld = odeWorld;
//	Reload(m_CarAttrib, posx,posy,posz);
	vInitalPos= Vector3p(posx,posy,posz);

	mVT_tireGroup = 0;
	
}

CMxCar::~CMxCar(void)
{
	dBodyDestroy(mCarbody.odeBody);
	dGeomDestroy(mCarbody.odeGeom);

	for (int i = 0; i < 4; i++) 
	{
		delete mSusp[i];
		delete mWheels[i];

    }
	delete mEngine;
//	delete mDifferential;
}

void CMxCar::Initialize(Vector3p & linPos, Quaternionp & qRotPos)
{
    for (int i = 0; i < 4; i++) 
	{
		mSusp[i]->Initialize();
		mWheels[i]->Initialize();		
    }
	mEngine->Initialize();
	//mDifferential->Initialize();

}
void CMxCar::Reload(CMxCarAttribute _CarAttrib, float posx, float posy, float posz)
{//destroy, reload.  This is for the vehicle dynamics editor, not game use. 

	dQuaternion q;
	dMass bodyMass;
	dMass engineMass;
	
	m_CarAttrib  = _CarAttrib;
	dBodyDestroy(mCarbody.odeBody);
	dGeomDestroy(mCarbody.odeGeom);
	//for(int i = 0; i< 4; i++)
	//{
	//	dGeomDestroy(mOdeWheelGeom[i]);
	//}//don't use ode for wheels

	mCarbody.odeBody = dBodyCreate (mOdeWorld);
  	
	dQFromAxisAndAngle(q,0,1,0,0);
    dBodySetQuaternion (mCarbody.odeBody,q);
	dMassSetBoxTotal (&bodyMass,m_CarAttrib.m_Body.iBodyMass,m_CarAttrib.m_Body.rBodyWidth, m_CarAttrib.m_Body.rBodyHeight,m_CarAttrib.m_Body.rBodyLength);
	//dMassSetParameters(&bodyMass,m_CarAttrib.m_Body.iBodyMass,m_CarAttrib.m_Body.rBodyWidth, m_CarAttrib.m_Body.rBodyHeight,m_CarAttrib.m_Body.rBodyLength);

	//test...move attribs to engine
	dMassSetBoxTotal(&engineMass,m_CarAttrib.m_Engine.rMass,m_CarAttrib.m_Engine.rWidth, m_CarAttrib.m_Engine.rHeight, m_CarAttrib.m_Engine.rLength);
	dMassTranslate(&engineMass, m_CarAttrib.m_Body.rPosOfEngine[0], m_CarAttrib.m_Body.rPosOfEngine[1], m_CarAttrib.m_Body.rPosOfEngine[2]);

	//dMassAdjust (&m,m_CarAttrib.m_Body.iBodyMass);//test
	fCoMY = m_CarAttrib.m_Body.rCoM[0];
	fCoMZ = m_CarAttrib.m_Body.rCoM[1];

	dMassTranslate(&bodyMass, 0, fCoMY, fCoMZ);
	dMassAdd(&bodyMass, &engineMass);	
    dBodySetMass (mCarbody.odeBody,&bodyMass);


	mCarbody.odeGeom = dCreateBox(mOdeSpace, m_CarAttrib.m_Body.rBodyWidth, m_CarAttrib.m_Body.rBodyHeight,m_CarAttrib.m_Body.rBodyLength);
	mCarbody.m_rWidth = m_CarAttrib.m_Body.rBodyWidth;
	mCarbody.m_rHeight = m_CarAttrib.m_Body.rBodyHeight;
	mCarbody.m_rLength = m_CarAttrib.m_Body.rBodyLength;
	mCarbody.mass = m_CarAttrib.m_Body.iBodyMass;//test

	dBodySetPosition(mCarbody.odeBody, posx, posy, posz);
	dBodySetLinearVel(mCarbody.odeBody, 0, 0, 0);
	dBodySetAngularVel(mCarbody.odeBody, 0, 0, 0);
	dGeomSetBody(mCarbody.odeGeom, mCarbody.odeBody);




	wOver2L = m_CarAttrib.m_Body.rFrontWheelBase/ 2*((m_CarAttrib.m_Body.rDistToFront+ m_CarAttrib.m_Body.rDistToRear));
	//wOver2L = (m_CarAttrib.m_Body.rDistToFront+ m_CarAttrib.m_Body.rDistToRear)/(10);//m_CarAttrib.m_Body.rFrontWheelBase/ (2.0f*(m_CarAttrib.m_Body.rDistToFront+ m_CarAttrib.m_Body.rDistToRear));
	
	steeringLock = m_CarAttrib.m_rMaxSteeringAngle;
	
//NOTE : test for now since not in GUI, this is just the axis m_iWhich the wheel is rotated about steering-wise
	//so for now just use an up vector, then transform to body coords
    Vector3p upVector(0.0f, 0.0f, 1.0f);
	const float *bodyRot= dBodyGetRotation((mCarbody.odeBody));
	Matrix3x3 mBodyMat;
	mBodyMat.m[0]= bodyRot[0];mBodyMat.m[1]= bodyRot[3];mBodyMat.m[2]= bodyRot[6];
	mBodyMat.m[3]= bodyRot[1];mBodyMat.m[4]= bodyRot[4];mBodyMat.m[5]= bodyRot[7];
	mBodyMat.m[6]= bodyRot[2];mBodyMat.m[7]= bodyRot[5];mBodyMat.m[8]= bodyRot[8];
	

	for (int i = 0; i < 2; i++) {
        Vector3p diagSteerAxis;
 		Vector3p tempSteerAxis(0,1,0);
		frontWheelAxis0[i].mult(mBodyMat, tempSteerAxis);

        diagSteerAxis.mult(mBodyMat, tempSteerAxis);
        dWheelAxis[i].cross(diagSteerAxis, frontWheelAxis0[i]);
        dWheelAxis[i].normalise();
    }


	driveType = m_CarAttrib.m_GearBox.iDriveType;
	
	//for(i = 0; i<2; i++){
		backBrakeFriction = m_CarAttrib.m_Tire[1].rPeakBrakingCoef;
		frontBrakeFriction= m_CarAttrib.m_Tire[0].rPeakBrakingCoef;

		maxBackBrakeTorque = m_CarAttrib.m_Tire[1].rBrakeForce;
		maxFrontBrakeTorque = m_CarAttrib.m_Tire[0].rBrakeForce;
		backBrakeTorque = 0.0f;
		frontBrakeTorque = 0.0f;

		maxHandBrakeTorque =  m_CarAttrib.m_rHandBrakeForce;
//	}


	bTireInitialized = false;//force contact point reset


	Update();//need positions for susp&wheel updates
	for( i= 0 ; i<4; i++)
	{
		//ODE Wheels
		//mOdeWheelGeom[i] = dCreateSphere(mOdeSpace, m_CarAttrib.m_Wheel[i/2].rRadius);
		//reload suspension
		mSusp[i]->Reload(&m_CarAttrib);
		//reload wheels
 		mWheels[i]->Reload(&m_CarAttrib);
	}
	//reload engine
	mTransmission->Reload(&m_CarAttrib);
	mEngine->Reload(&m_CarAttrib);
	mDriveline->Reload(&m_CarAttrib);
//	mDifferential->Reload(&m_CarAttrib);
	mAerodynamics->Reload(&m_CarAttrib);
	
}
void CMxCar::Update()
{
	//add some damping to the chassis body... we'll just call it "air resistance" or something...	//Prevent some explosions
	//(angular)
 	//const dReal *angVel = dBodyGetAngularVel(mCarbody.odeBody);
 	//dBodySetAngularVel(mCarbody.odeBody, angVel[0]*0.99f, angVel[1]*.99f, angVel[2]*.99f);

	//update graphics only (and set ODE position for wheel GeomID)
	mCarbody.Update();//Use this, delete below
	
	//test
	//Vector3p upVector(0.0f, 1.0f, 0.0f);
	/*const float *bodyRot= dBodyGetRotation((mCarbody.odeBody)); //4x3
	Matrix3x3 mBodyMat; 
 	mBodyMat.m[0]= bodyRot[0];mBodyMat.m[1]= bodyRot[1];mBodyMat.m[2]= bodyRot[2];
	mBodyMat.m[3]= bodyRot[4];mBodyMat.m[4]= bodyRot[5];mBodyMat.m[5]= bodyRot[6];
	mBodyMat.m[6]= bodyRot[8];mBodyMat.m[7]= bodyRot[9];mBodyMat.m[8]= bodyRot[10];*/
	/*mBodyMat.m[0]= bodyRot[0];mBodyMat.m[1]= bodyRot[3];mBodyMat.m[2]= bodyRot[6];
	mBodyMat.m[3]= bodyRot[1];mBodyMat.m[4]= bodyRot[4];mBodyMat.m[5]= bodyRot[7];
	mBodyMat.m[6]= bodyRot[2];mBodyMat.m[7]= bodyRot[5];mBodyMat.m[8]= bodyRot[8];*/

	frontWheelAxis0[0].mult(mCarbody.rotPos, Vector3p(0, 0, 1.0f));
    frontWheelAxis0[1].mult(mCarbody.rotPos, Vector3p(0, 0, 1.0f));
 		
///////test mWheels.getPosition
	Vector3p worldLinPos;
	Matrix3x3 rotationalPos;
    
	for(int i = 0; i<4; i++)
	{		
		//test... all this should be done in CMxSusp
		//suspAxis[i].mult(mCarbody.rotPos, mSusp[i]->axis);
		//mSusp[i]->suspPos0.transform( mSusp[i]->position,mCarbody.rotPos, mSusp[i]->position.ZERO);	
		//mSusp[i]->axisCC.add(suspAxis[i], suspAxis[i].ZERO);
	
		mSusp[i]->Update();
		//update wheel	
		mWheels[i]->Update();
				
		mWheels[i]->getPosition(worldLinPos, rotationalPos);//not world LinPos not use right now... writes to wheel.positionWld
		//dGeomSetPosition(mOdeWheelGeom[i],  mWheels[i]->positionWld.x, mWheels[i]->positionWld.y,mWheels[i]->positionWld.z);
		Quaternionp quat;
		quat.fromMatrix(rotationalPos);

/*		Matrix3x3 wheelRotWorld;
		wheelRotWorld.mult(mBodyMat, mWheels[i]->noRotOrient);
		//save positionWld//taken care of in wheel.getPosition
		//mWheels[i]->positionWld.x =  (p[0]+mWheels[i]->position.x);
		//mWheels[i]->positionWld.y =  (p[1]+mWheels[i]->position.y);
		//mWheels[i]->positionWld.z =  (p[2]+mWheels[i]->position.z);
		dGeomSetPosition(mOdeWheelGeom[i],  mWheels[i]->positionWld.x, mWheels[i]->positionWld.y,mWheels[i]->positionWld.z);
		m_wheelSceneNode[i]->setPosition(mWheels[i]->positionWld.x,mWheels[i]->positionWld.y,mWheels[i]->positionWld.z);
		Quaternionp quat;
		quat.fromMatrix(wheelRotWorld);
		m_wheelSceneNode[i]->setOrientation(quat.w, quat.x, quat.y, quat.z);
*/
		/////test mWheels.getPosition
		
 
	
		//m_wheelSceneNode[i]->setPosition(mWheels[i]->positionWld.x,mWheels[i]->positionWld.y, mWheels[i]->positionWld.z);
		//m_wheelSceneNode[i]->setOrientation(quat.w, quat.x, quat.y, quat.z);
				
	}

	//#ifdef USE_COLLISIONGEOMETRY
	//initialize the tire contact
	//if(!bTireInitialized)
	//{	
	//	for(int i = 0; i<4; i++)
	//		mWheels[i]->trackContact.init(mWheels[i]->positionWld.x, mWheels[i]->positionWld.z);
	//	bTireInitialized= true;
	//}
//#endif

	for (i = 0; i < 2; i++) {
        Vector3p diagSteerAxis;
   		Vector3p tempSteerAxis(0,1,0); //should be spindle orientation... change later.
 		frontWheelAxis0[i] = Vector3p(1,tan ((mWheels[i]->staticCamber*M_PI)/180),tan((mWheels[i]->toe*M_PI)/(180)));//mWheels[i]->axis;//Vector3p(1,0,0);
		frontWheelAxis0[i].normalise();
 		diagSteerAxis.mult(mBodyMat, mSusp[i]->axis);
		
        dWheelAxis[i].cross(mSusp[i]->axis, frontWheelAxis0[i]);
        dWheelAxis[i].normalise();
    }

	mCarbody.Update(); 	
}

void CMxCar::addBodyForce(Vector3p worldPos, Vector3p & worldForce)
{//add "addBodyImpulse" too

//ODE, might be relforceatpos
	//dBodyAddForceAtPos(mCarbody.odeBody, worldForce.x, worldForce.y, worldForce.z, 
	//	worldPos.x, worldPos.y, worldPos.z);
	dBodyAddForceAtPos(mCarbody.odeBody, worldForce.x, worldForce.y, worldForce.z,
		worldPos.x, worldPos.y, worldPos.z);

}
void CMxCar::addBodyTorque(Vector3p bodyAxis, float torqueScale)
{
	bodyAxis.normalise();
	bodyAxis.mult(torqueScale);
	
	dBodyAddRelTorque(mCarbody.odeBody, bodyAxis.x, bodyAxis.y, bodyAxis.z);
		

}

void CMxCar::addTireForce(int i, Vector3p & worldPos, Vector3p & worldForce)
{
	{
	Vector3p ccPos, ccForce;
	ccPos.invTransform(worldPos, mCarbody.rotPos, mCarbody.linPos);
	ccForce.transMult(mCarbody.rotPos, worldForce);//rotPos is ortho, so transpose(rotPos) = inverse(rotPos)
	
// find torque on wheel
    Vector3p CP;
	Vector3p ccWheelPos;
	Vector3p tcWheelPos;

	tcWheelPos = mWheels[i]->position;// tire coords.  assume on same axis so ignore xz 
	CP.sub(ccPos, tcWheelPos);// patch to wheel

	//Ground forces acting on wheel's rotation + rolling resistance
    Vector3p torque;                                 // torque vector on wheel
    torque.cross(CP, ccForce);
 	mWheels[i]->InputOut.brakingTorque += torque.dot(mWheels[i]->axis) + mWheels[i]->torqueRollingResist;

	// Forces acting along suspension
 	float suspMag = ccForce.dot(mSusp[i]->axis);
	mSusp[i]->linearForce += suspMag;

	Vector3p ContactPatch = mWheels[i]->position; //in CC
	ContactPatch.mac(mWheels[i]->m_vDown, mWheels[i]->radius);

	//Project worldForce onto ground plane.
	
	dBodyAddRelForceAtRelPos(mCarbody.odeBody,ccForce.x, 0, ccForce.z,ContactPatch.x, ContactPatch.y, ContactPatch.z);
//	dBodyAddForceAtRelPos(mCarbody.odeBody,worldForce.x, 0, worldForce.z,ContactPatch.x, ContactPatch.y, ContactPatch.z);

	////test bike balance.
	//addBodyTorque(Vector3p(0,0,1.0f), tempSteerPos*1000);
	//dBodyAddRelForce(mCarbody.odeBody,	-tempSteerPos*100, 0,0 );
	//dMatrix3 bodyOrient;
	//const float *bodyRot= dBodyGetRotation((mCarbody.odeBody)); //4x3
	//bodyOrient = 
	//dBodySetRotation(mCarbody.odeBody, );
	//const dReal *OdeQuat = dBodyGetQuaternion (mCarbody.odeBody);
	//float w = OdeQuat[0];
	//float x = OdeQuat[1];
	//float y = OdeQuat[2];
	//float z = OdeQuat[3];

	//real rQuatAngle = acos (OdeQuat[0]) * 2;

	//Quaternionp quat(w,x,y,z);
	//quat.rotation(

	
 }

	
 }

void CMxCar::addSuspForce(int i, float suspForce)
{
	//mSusp[i]->forceSpring +=suspForce;
	mSusp[i]->linearForce +=suspForce;
	
}

void CMxCar::addApplySuspForce(int i, Vector3p suspForce)
{
	//mSusp[i]->forceSpring +=suspForce;
	mSusp[i]->linearForce +=mSusp[i]->vInstantaneousAxisCC.dot(suspForce);

	/*
	Vector3p vRollCenterToCG;
	//vRollCenterToCG.sub( mSusp[i]->suspPos0 , Vector3p(0.0f, 0.0f , 0.0f));


	//if(i<2)
		//vRollCenterToCG.sub( mSusp[i]->position , vRollCenterFront_CC);
	//else
		//vRollCenterToCG.sub( mSusp[i]->position , vRollCenterRear_CC);
	vRollCenterToCG.sub(mSusp[i]->position, Vector3p(0.0f, -0.5f , 0.0f));
		
	Vector3p down = mCarbody.m_vDown;
    down.mult(0);

	//dBodyAddForceAtRelPos(mCarbody.odeBody,0, suspForce, 0,vRollCenterToCG.x, vRollCenterToCG.y, vRollCenterToCG.z);
//	dBodyAddForceAtRelPos(mCarbody.odeBody,0, suspForce, 0,mSusp[i]->position.x+down.x,mSusp[i]->position.y+down.y, mSusp[i]->position.z+down.z);
 	dBodyAddRelForceAtRelPos(mCarbody.odeBody,0, suspForce, 0,mSusp[i]->position.x+down.x,mSusp[i]->position.y+down.y, mSusp[i]->position.z+down.z);
	*/

	Vector3p vSuspToRC; //can precompute...not anymore, using dynamic roll centers
	/*if(i<2)
		vSuspToRC.sub( mSusp[i]->vMountTopCC , vRollCenterFront_CC);
	else
		vSuspToRC.sub( mSusp[i]->vMountTopCC , vRollCenterRear_CC);*/
	if(i<2)
		vSuspToRC.sub( mWheels[i]->position , vRollCenterFront_CC);
	else
		vSuspToRC.sub( mWheels[i]->position , vRollCenterRear_CC);
		
	
	//if(i<2)
	//	vSuspToRC.sub( mSusp[i]->vMountTopWC , vRollCenterFront_WC);
	//else
	//	vSuspToRC.sub( mSusp[i]->vMountTopWC , vRollCenterRear_WC);
	
	dBodyAddForceAtRelPos(mCarbody.odeBody,suspForce.x, suspForce.y, suspForce.z,vSuspToRC.x, vSuspToRC.y,vSuspToRC.z);

	//dBodyAddForceAtPos(mCarbody.odeBody,suspForce.x, suspForce.y, suspForce.z,vSuspToRC.x, vSuspToRC.y,vSuspToRC.z);
}


void CMxCar::addWheelBrakingTorque(int i, float wheelTorque)
{
	//mWheels[i]->rotForce += wheelTorque;
	mWheels[i]->InputOut.brakingTorque += wheelTorque;
}

float CMxCar::getDriveAccel()
{

	switch(driveType)
	{
	case 0:
		return (mWheels[0]->rotAccel +mWheels[1]->rotAccel)*0.5f;
	case 1:
		return (mWheels[2]->rotAccel +mWheels[3]->rotAccel)*0.5f;
	case 2:
		return (mWheels[0]->rotAccel +mWheels[1]->rotAccel + mWheels[2]->rotAccel +mWheels[3]->rotAccel)*0.25f;
	}

	return 0;
}


void CMxCar::addDriveTorque(float driveshaftTorque)
 {

//	 mDifferential->Forces(driveshaftTorque, mEngine->m_mOfI);


	for(int i = 0; i< 4; i++)
	{
		//mWheels[i]->rotForce += (mDifferential->m_rTorqueOut[i] )*mTransmission->GetFinalDriveRatio();
		
		//mWheels[i]->rotForce +=mWheels[i]->InputIn.torque;
		mWheels[i]->rotForce =mWheels[i]->InputIn.torque;
				//damp if rot velocity changes directions
		//if(mWheels[i]->rotForce *mWheels[i]->oldRotForce<0)
		//{
		//	mWheels[i]->rotVel = 0.00f;
		//	mWheels[i]->rotForce *=0.4f;
		//}
		//mWheels[i]->oldRotForce = mWheels[i]->rotForce;
//			mWheels[i]->oldRotVel = mWheels[i]->rotVel;


		//mWheels[i]->rotForce += driveshaftTorque*0.25f;

	}
			
	//mWheels[i]->rotForce += driveTorque*0.25f;
		
	//mWheels[2]->rotForce += driveTorque;//*5.0f;
	//mWheels[3]->rotForce += driveTorque;//*5.0f;
	//mWheels[i]->rotForce += mDifferential->m_rTorqueOut[i];
			
	
	///NOTE: Replace this with driveline Differentials
	//extern CMxCarAttribute _CarAttrib;

		//mWheels[1]->rotForce += driveTorque*0.50f;
		//mWheels[0]->rotForce += driveTorque*0.50f;
		//mWheels[2]->rotForce += driveTorque*0.50f;
		//mWheels[3]->rotForce += driveTorque*0.50f;


/*	if(_CarAttrib.m_GearBox.iDriveType == 0){ //fwd
		mWheels[0]->rotForce += driveTorque*0.5f;
		mWheels[1]->rotForce += driveTorque*0.5f;
	}
	if(_CarAttrib.m_GearBox.iDriveType == 1){ //rwd
		mWheels[2]->rotForce += driveTorque*0.5f;
		mWheels[3]->rotForce += driveTorque*0.5f;
	}
	if(_CarAttrib.m_GearBox.iDriveType == 2){ //awd
		mWheels[0]->rotForce += driveTorque*0.5f;//*_CarAttrib.m_GearBox.rAWDRearPowerRatio;
		mWheels[1]->rotForce += driveTorque*0.5f;//*_CarAttrib.m_GearBox.rAWDRearPowerRatio;

		mWheels[2]->rotForce += driveTorque*0.5f;//*(1.0f-_CarAttrib.m_GearBox.rAWDRearPowerRatio);
		mWheels[3]->rotForce += driveTorque*0.5f;//*(1.0f-_CarAttrib.m_GearBox.rAWDRearPowerRatio);
	}
*/	
}

void CMxCar::setSteering(float steer)
{ 
	tempSteerPos = steer;//bike
	
	float tanS = float(tan(-steer * steeringLock*M_PI/180));        // convert to radians
    float w2LTanS = wOver2L * tanS;
     float alphaL = tanS / (1.0f + w2LTanS);
    float alphaR = tanS / (1.0f - w2LTanS);
	
	mWheels[0]->axis.mac(frontWheelAxis0[0], dWheelAxis[0], alphaL);
    mWheels[0]->axis.normalise(); 
    mWheels[1]->axis.mac(frontWheelAxis0[1], dWheelAxis[1], alphaR);
    mWheels[1]->axis.normalise();

// set up front wheels rotation matrix. x axis of matrix is wheel axis. y axis is horizontal.
    float k = 1.0f / float(sqrt(mWheels[0]->axis.x * mWheels[0]->axis.x + mWheels[0]->axis.y * mWheels[0]->axis.y));
    mWheels[0]->noRotOrient = Matrix3x3(
       mWheels[0]->axis.x, -k * mWheels[0]->axis.y, -k * mWheels[0]->axis.x * mWheels[0]->axis.z,
       mWheels[0]->axis.y, k * mWheels[0]->axis.x, -k * mWheels[0]->axis.y *mWheels[0]->axis.z,
        mWheels[0]->axis.z, 0.0, 1.0f / k
    );
// set up wheel on other side as if axis was negated
    k = 1.0f / float(sqrt(mWheels[1]->axis.x * mWheels[1]->axis.x + mWheels[1]->axis.y * mWheels[1]->axis.y));
    mWheels[1]->noRotOrient = Matrix3x3(
        -mWheels[1]->axis.x, k *mWheels[1]->axis.y, -k * mWheels[1]->axis.x * mWheels[1]->axis.z,
        -mWheels[1]->axis.y, -k * mWheels[1]->axis.x, -k * mWheels[1]->axis.y * mWheels[1]->axis.z,
        -mWheels[1]->axis.z, 0.0, 1.0f / k
    );
}
///

void CMxCar::getBodyPos(Vector3p & linPos, Matrix3x3 & rotPos)
{
	const float *pos = dBodyGetPosition(mCarbody.odeBody);
	const float *rot = dBodyGetQuaternion(mCarbody.odeBody);
	
	linPos.x = pos[0];
	linPos.y = pos[1];
	linPos.z = pos[2];

	
	Quaternionp quat(rot[0], rot[1], rot[2], rot[3]);
	rotPos.fromQuat(quat);
}
void CMxCar::getWheelPos(Vector3p linPos[4], Matrix3x3 rotPos[4])
{

}
float CMxCar::getDriveVel(void)
{
	switch(driveType)
	{
	case 0:
		return (mWheels[0]->rotVel +mWheels[1]->rotVel)*0.5f;
	case 1:
		return (mWheels[2]->rotVel +mWheels[3]->rotVel)*0.5f;
	case 2:
		return (mWheels[0]->rotVel +mWheels[1]->rotVel + mWheels[2]->rotVel +mWheels[3]->rotVel)*0.25f;
	}

	return 0;
}
float CMxCar::getWheelVel(int i)
{
	return (mWheels[i]->rotVel);
	
}
float CMxCar::getWheelOldVel(int i)
{
	return (mWheels[i]->oldRotVel);
}
void CMxCar::getSuspPosVel(int i, float & suspPos, float & suspVel)
{
	suspPos = mSusp[i]->linearPosition;
	suspVel = mSusp[i]->linearVelocity;
}
void CMxCar::getTirePosVel(int i, Vector3p & linPos, Vector3p & linVel, Vector3p & axis, float & rotVel)
{

}

void CMxCar::Evolve(float h, float steer, float accel, float brake, float handBrake, float reverse)
{


}
void CMxCar::RKInit()
{
	//evolve body too.//test ODE first
	 for (int i = 0; i < 4; i++) 
	 {
		mWheels[i]->rotPos0	 = mWheels[i]->rotPos;
		mSusp[i]->linearPosition0 = mSusp[i]->linearPosition;
		mWheels[i]->rotVel0 = mWheels[i]->rotVel;
		mSusp[i]->linearVelocity0 = mSusp[i]->linearVelocity;
		mWheels[i]->rotForce = 0.0f;
		mSusp[i]->linearForce = 0.0f;
		
		mWheels[i]->InputOut.brakingTorque = 0.0f;
		mWheels[i]->RKInit();
    }
}

	//mWheels
	//mSusp
void CMxCar::RKStep1()
{	
	//mainBody.RKStep1();
    for (int i = 0; i < 4; i++) {
		mSusp[i]->linearAcceleration1 = mSusp[i]->linearForce * mWheels[i]->oneOverMass * CSim::RKh2;
		mWheels[i]->rotAccel1= mWheels[i]->rotForce* mWheels[i]->oneOverMOfI * CSim::RKh2;
		mSusp[i]->linearVelocity1 = mSusp[i]->linearVelocity;
		mWheels[i]->rotVel1 = mWheels[i]->rotVel;

		mSusp[i]->linearPosition += CSim::RKh2 * mSusp[i]->linearVelocity1;
		mWheels[i]->rotPos += CSim::RKh2 * mWheels[i]->rotVel1;

		mSusp[i]->linearVelocity += mSusp[i]->linearAcceleration1;
		mWheels[i]->rotVel += mWheels[i]->rotAccel1;

        mWheels[i]->rotForce = 0.0f;
		mSusp[i]->linearForce = 0.0f;
    }
}

void CMxCar::RKStep2()
{	
	//mainBody.RKStep2();
    for (int i = 0; i < 4; i++) {
		mSusp[i]->linearAcceleration2 = mSusp[i]->linearForce * mWheels[i]->oneOverMass * CSim::RKh2;
		mWheels[i]->rotAccel2= mWheels[i]->rotForce* mWheels[i]->oneOverMOfI * CSim::RKh2;
		mSusp[i]->linearVelocity2 = mSusp[i]->linearVelocity;
		mWheels[i]->rotVel2 = mWheels[i]->rotVel;
		mSusp[i]->linearPosition += CSim::RKh2 * mSusp[i]->linearVelocity2;
		mWheels[i]->rotPos += CSim::RKh2 * mWheels[i]->rotVel2;
		mSusp[i]->linearVelocity += mSusp[i]->linearAcceleration2;
		mWheels[i]->rotVel += mWheels[i]->rotAccel2;
        mWheels[i]->rotForce = 0.0f;
		mSusp[i]->linearForce = 0.0f;
    }
}

void CMxCar::RKStep3()
{	
	//mainBody.RKStep3();
    for (int i = 0; i < 4; i++) {
		mSusp[i]->linearAcceleration3 = mSusp[i]->linearForce * mWheels[i]->oneOverMass * CSim::RKh;
		mWheels[i]->rotAccel3= mWheels[i]->rotForce* mWheels[i]->oneOverMOfI * CSim::RKh;
		mSusp[i]->linearVelocity3 = mSusp[i]->linearVelocity;
		mWheels[i]->rotVel3 = mWheels[i]->rotVel;
		mSusp[i]->linearPosition += CSim::RKh * mSusp[i]->linearVelocity3;
		mWheels[i]->rotPos += CSim::RKh * mWheels[i]->rotVel3;
		mSusp[i]->linearVelocity += mSusp[i]->linearAcceleration3;
		mWheels[i]->rotVel += mWheels[i]->rotAccel3;
        mWheels[i]->rotForce = 0.0f;
		mSusp[i]->linearForce = 0.0f;
    }
}
void CMxCar::RKStep4()
{	
	//mainBody.RKStep4();
    for (int i = 0; i < 4; i++) {
		mSusp[i]->linearPosition = mSusp[i]->linearPosition0 + (mSusp[i]->linearVelocity1 + 2.0f *(mSusp[i]->linearVelocity2 + mSusp[i]->linearVelocity3) )* CSim::RKh6;
		mWheels[i]->rotPos = mWheels[i]->rotPos0 + (mWheels[i]->rotVel1 + 2.0f *(mWheels[i]->rotVel2 + mWheels[i]->rotVel3))* CSim::RKh6;

		mSusp[i]->linearVelocity = mSusp[i]->linearVelocity0 + (mSusp[i]->linearAcceleration1+  2.0f *(mSusp[i]->linearVelocity2 + mSusp[i]->linearVelocity))*CSim::RKh6;
		mWheels[i]->rotVel = mWheels[i]->rotVel0  + (mWheels[i]->rotAccel1 + 2.0f *(mWheels[i]->rotAccel2 + mWheels[i]->rotAccel3))* CSim::RKh6;
        
		//mWheels[i]->rotForce = 0.0f;
		//mSusp[i]->linearForce = 0.0f;
		mWheels[i]->rotPos = float (fmod(mWheels[i]->rotPos, 2.0f*M_PI));
		        
		while (mWheels[i]->rotPos >= 2.0 * M_PI)
            mWheels[i]->rotPos -= 2.0 * M_PI;
        while (mWheels[i]->rotPos < 0.0)
            mWheels[i]->rotPos += 2.0 * M_PI;

		if(mWheels[i]->bLocked == true)
		{
			//mWheels[i]->rotPos = 0.0f;//test	
			mWheels[i]->rotVel = 0.0f;
		}

			//if suspension at limits restrict
		if(mSusp[i]->linearPosition >  mSusp[i]->maxLength - mSusp[i]->restLength)
		{
			mSusp[i]->linearPosition =  mSusp[i]->maxLength - mSusp[i]->restLength;
			mSusp[i]->linearVelocity = 0.0f;

		}
		else if(mSusp[i]->linearPosition <  mSusp[i]->minLength - mSusp[i]->restLength)
		{
			mSusp[i]->linearPosition =  mSusp[i]->minLength - mSusp[i]->restLength;
			mSusp[i]->linearVelocity = 0.0f;
		}
  }

	
}

void CMxCar::EulerStep()
{	

    for (int i = 0; i < 4; i++) {
		mSusp[i]->linearAcceleration = mSusp[i]->linearForce * mWheels[i]->oneOverMass ;//* CSim::RKh;
		mSusp[i]->linearVelocity += mSusp[i]->linearAcceleration*CSim::RKh;
		mSusp[i]->linearPosition += mSusp[i]->linearVelocity* CSim::RKh;

		mWheels[i]->rotAccel= mWheels[i]->rotForce* mWheels[i]->oneOverMOfI;// * CSim::RKh;
		mWheels[i]->rotVel += mWheels[i]->rotAccel* CSim::RKh;
		
		mWheels[i]->oldRotVel = mWheels[i]->rotVel;



		mWheels[i]->rotPos += mWheels[i]->rotVel* CSim::RKh;

		//mWheels[i]->rotPos = float (fmod(mWheels[i]->rotPos, 2.0f*M_PI));//
		        
		if(mWheels[i]->rotPos>100000.0
			||mWheels[i]->rotPos<-100000.0)//don't explode
			mWheels[i]->rotPos = 0;

		while (mWheels[i]->rotPos >= 2.0 * M_PI)
            mWheels[i]->rotPos -= 2.0 * M_PI;
        while (mWheels[i]->rotPos < 0.0)
            mWheels[i]->rotPos += 2.0 * M_PI;



		//if suspension at limits restrict
		if(mSusp[i]->linearPosition >  mSusp[i]->maxLength)
		{
			mSusp[i]->linearPosition =  mSusp[i]->maxLength;
			mSusp[i]->linearVelocity = 0.0f;
			mSusp[i]->linearAcceleration *= -0.7f;//-mSusp[i]->linearAcceleration;
			mSusp[i]->linearForce *=-0.0f;
		}
		else if(mSusp[i]->linearPosition < mSusp[i]->minLength)
		{
			mSusp[i]->linearPosition =  mSusp[i]->minLength;
			mSusp[i]->linearVelocity = 0.0f;
			mSusp[i]->linearAcceleration *= -0.7f;//-mSusp[i]->linearAcceleration;
			mSusp[i]->linearForce *=-0.0f;
		}
  }
}

void CMxCar::forces()
{
	CalcRollCenters();//find the roll centers of  front and back suspensions 	
	//do engine forces, find clutch force and drive torque (after gear)
	/////////////driveline test
	mDriveline->DoForces(getDriveVel());// do all driveline forces
	addDriveTorque(mDriveline->rDriveshaftTorque);
	/////////////testing driveline end

    // do brake forces/torques
	// brake torque = rotVel.x * kv, up to an absolute maximum of current brake force
	if(mTransmission->m_rBrakePos >0.0f )
	{
		float bTorque;
		for(int i = 0; i<2; i++)
		{
			if(getWheelVel(i)>0)
				bTorque = frontBrakeTorque* frontBrakeFriction;
			else
				bTorque = -frontBrakeTorque* frontBrakeFriction;

			if(fabs(getWheelVel(i))<4.0f)//&& bTorque > 400)//rotvel too low, lock the wheel //should have if (bTorque > lockingTorque)...
			{
				mWheels[i]->rotForce = 0.0f;
				mWheels[i]->rotVel = 0.0f;
				mWheels[i]->bLocked = true;		
			}
			else//otherwise apply brake torque
			{
				addWheelBrakingTorque(i, -bTorque);
				//mWheels[i]->bLocked = false;
			}
		}
		for(i = 2; i<4; i++)
		{
			if(getWheelVel(i)>0)
				bTorque = backBrakeTorque* backBrakeFriction;
			else
				bTorque = -backBrakeTorque* backBrakeFriction;

			if(fabs(getWheelVel(i))<4.0f)//rotvel too low, lock the wheel
			{
				mWheels[i]->rotForce = 0.0f;
				mWheels[i]->rotVel = 0.0f;
				mWheels[i]->bLocked = true;		
			}
			else//otherwise apply brake torque
			{
				addWheelBrakingTorque(i, -bTorque);
				//mWheels[i]->bLocked = false;
			}
		}
}

	else if(mTransmission->m_rHandBrakePos > 0.0f)
	{
		float bTorque;
		for(int i = 2; i<4; i++)
		{
			if(getWheelVel(i)>0)
				bTorque = backBrakeTorque* backBrakeFriction;
			else
				bTorque = -backBrakeTorque* backBrakeFriction;
			//addWheelBrakingTorque(3, -bTorque);
			if(fabs(getWheelVel(i))<4.0f)//rotvel too low, lock the wheel
			{
				mWheels[i]->rotForce = 0.0f;
				mWheels[i]->rotVel = 0.0f;
				mWheels[i]->bLocked = true;		
			}
			else//otherwise apply brake torque
			{
				addWheelBrakingTorque(i, -bTorque);
				//mWheels[i]->bLocked = false;
			}
		}
	}

	else//unlock the wheels
	{
		mWheels[0]->bLocked = false;
		mWheels[1]->bLocked = false;
		mWheels[2]->bLocked = false;
		mWheels[3]->bLocked = false;
	
	}
   
	
    float suspPos[4], suspVel[4];
    for (int i = 0; i < 4; i++)
        getSuspPosVel(i, suspPos[i], suspVel[i]);

	float frontRollBar = (suspPos[0] - suspPos[1]) * mSusp[0]->arbKs;
	float rearRollBar = (suspPos[2] - suspPos[3]) * mSusp[3]->arbKs;



	for (i = 0; i < 4; i++)
	{
		float IR2 = mSusp[i]->fInstallationRatio*mSusp[i]->fInstallationRatio;//TEst IR2
		if(mSusp[i]->linearVelocity>0)//bump
		{
 			addSuspForce(i, -mSusp[i]->ks *IR2* suspPos[i] - mSusp[i]->bumpRate * suspVel[i] );
			//adds the wheel rate force
		}
		else// rebound
		{
			addSuspForce(i, -mSusp[i]->ks *IR2* suspPos[i] - mSusp[i]->reboundRate * suspVel[i] );
		}
		//if at top limit

		//if at lower limit



	}
	addSuspForce(0, - frontRollBar);
	addSuspForce(1,  frontRollBar);
	addSuspForce(2,  - rearRollBar);
	addSuspForce(3, rearRollBar);


	//runs slow on my laptop (and LG's too)
	//for (i = 0; i < 4; i++)
	//{
	//	Vector3p tireForce;	
	//	if(mWheels[i]->forces(tireForce))
	//	{
	//		//mBodyMat
	//		//dBodyAddRelForce(mCarbody.odeBody, fnormal.x, fnormal.y, fnormal.z);//fnormal in local coords.
	//		//dBodyAddForce(mCarbody.odeBody, fnormal.x, fnormal.y, fnormal.z);
	//		//mSusp[i]->linearForce+=mSusp[i]->axis.dot(fnormal);//TEST, should be own function

	//		addTireForce(i, mWheels[i]->forcePosWld, tireForce);
	//		addApplySuspForce(i, tireForce);
	//		
	//		//addBodyForce(mWheels[i]->positionWld, tireForce);//TEsT

	//		//dBodyAddRelForceAtRelPos(mCarbody.odeBody,
	//		//	tireForce.x, tireForce.y, tireForce.z,
	//		//	mSusp[i]->position.x, mSusp[i]->position.y, mSusp[i]->position.z
 // 	//			);
	//	}

	//}
	

	for (i = 0; i < 4; i++)
	{
		Vector3p tireForce;	
		if(	mWheels[i]->forces(tireForce))
		{
			//tireForce = Vector3p (0, 8000,0);//test virtools
	
			addTireForce(i, mWheels[i]->forcePosWld, tireForce);
			addApplySuspForce(i, tireForce);
		}
	}

	//const dReal *bodyODELinForce = dBodyGetForce(mCarbody.odeBody);
	//mCarbody.linForce.x = bodyODELinForce[0];
	//mCarbody.linForce.y = bodyODELinForce[1];
	//mCarbody.linForce.z = bodyODELinForce[2];

	///engine is updated, send torque to differential

	
	///Aero Forces
	mAerodynamics->DoWingForces();
	mAerodynamics->DoBodyForces();
//	mAerodynamics->m_vDragForce.
	//apply drag forces
	dBodyAddForce(mCarbody.odeBody, 	mAerodynamics->m_vDragForce.x, 	mAerodynamics->m_vDragForce.y, 	mAerodynamics->m_vDragForce.z);

	//add lift/down forces 
	 dBodyAddForceAtRelPos(mCarbody.odeBody, mAerodynamics->m_vRearLiftForce.x, mAerodynamics->m_vRearLiftForce.y, mAerodynamics->m_vRearLiftForce.z,
		 0.0f, mCarbody.m_rHeight, mSusp[2]->position.z);

	 dBodyAddForceAtRelPos(mCarbody.odeBody, mAerodynamics->m_vFrontLiftForce.x, mAerodynamics->m_vFrontLiftForce.y, mAerodynamics->m_vFrontLiftForce.z,
		 0.0f, mCarbody.m_rHeight, mSusp[0]->position.z);

	 //do balance forces

	 const dReal *torque = dBodyGetTorque(mCarbody.odeBody);
	torque[0];
	torque[1];
	//torque[2];
	 //dBodySetTorque(mCarbody.odeBody, torque[0], torque[1], -0.55*torque[2]);
	// const dReal *quat =  dBodyGetQuaternion (mCarbody.odeBody);
//
//    float bodyRollAngle = atan2(mCarbody.linForce.y, mCarbody.linForce.x);
//	dQuaternion quatFinal, quatRoll;
//	const dReal *quaternion = dBodyGetQuaternion(mCarbody.odeBody );
//	//dMatrix3 = dBodyGetRotation(mCarbody.odeBody );
//	Quaternionp quatpRoll;
//	quatpRoll.rotation(Vector3p(0,0,1), - 2*bodyRollAngle);
//	quatRoll[0] = quatpRoll.w;
//	quatRoll[1] = quatpRoll.x;
//	quatRoll[2] = quatpRoll.y;
//	quatRoll[3] = quatpRoll.z;
//
//
//	dQMultiply0 (quatFinal,  quaternion, quatRoll);
//
//	//quatFinal = Quaternionp(quaternion[0], quaternion[1],quaternion[2],quaternion[3]);
//    
////	quatFinal.
//
////	dQFromAxisAndAngle(quaternion, 0, 0, 1, bodyRollAngle);
//	dBodySetQuaternion (mCarbody.odeBody, quatFinal);


}

Vector3p testFindIntersect(Vector3p PA1,Vector3p PB1, Vector3p PA2, Vector3p PB2)
{

	//Vector3p VA, VB, VC;
	//VA.sub(PA1, PA2);
	//VB.sub(PB1, PB2);

	//VA.normalise();
	//VC.sub(PA1, PB1);

	//Vector3p temp1, temp2;
	//
	//temp1.cross(VA,VB);
	//temp2.cross(VA, VC);


	//float mag = temp1.mod()/temp2.mod();

	//Vector3p vIntersect;
	//vIntersect.mac(PA1, VA, mag);


	Vector3p V1, V2, V3;
	V1.sub(PA1, PB1);
	V2.sub(PA2, PB2);

	V1.normalise();
	V2.normalise();

	V3.sub(PA2, PA1);

	Vector3p temp1, temp2;
	
	temp1.cross(V3,V2);
	temp2.cross(V1, V2);


	float mag = -temp1.mod()/temp2.mod();

	Vector3p vIntersect;
	vIntersect.mac(PA1, V1, mag);

	return vIntersect;
    
}

void CMxCar::CalcRollCenters()
{
	//float k = c*(c+sigma)/(sin(alpha+ beta));
	//float p = k * sin(beta) + d;
	//float hr = bf/2.0f * p/ (k*cos(beta) + d*tan(sigma) +roSigma)
	//test static (always inbetween wheels, but "up" is scaled)
#define USE_STATICROLLCENTERS
#ifdef USE_STATICROLLCENTERS
	/*vRollCenterFront_WC.add(mWheels[0]->positionWld, mWheels[1]->positionWld);
    vRollCenterRear_WC.add(mWheels[2]->positionWld, mWheels[3]->positionWld);
	vRollCenterFront_WC.mult(0.5f);
	vRollCenterRear_WC.mult(0.5f);

	vRollCenterRear_WC.add(Vector3p(mCarbody.rotPos.m[1], mCarbody.rotPos.m[4], mCarbody.rotPos.m[7]));
	vRollCenterFront_WC.add(Vector3p(mCarbody.rotPos.m[1], mCarbody.rotPos.m[4], mCarbody.rotPos.m[7]));*/

	vRollCenterFront_CC.add(mSusp[0]->position, mSusp[1]->position);
    vRollCenterRear_CC.add(mSusp[2]->position, mSusp[3]->position);

	vRollCenterFront_CC.mult(0.5f);
 	vRollCenterRear_CC.mult(0.5f);

	Vector3p down = mCarbody.m_vDown;
    down.mult(mSusp[0]->fRCYPos);
	vRollCenterFront_CC.add(down);

	down = mCarbody.m_vDown;
	down.mult(mSusp[2]->fRCYPos);
	vRollCenterRear_CC.add(down);

	vRollCenterFront_CC.z *=mSusp[0]->fRCZPos;
	vRollCenterRear_CC.z *= mSusp[2]->fRCZPos;



    vRollCenterRear_WC.mult(mCarbody.rotPos, vRollCenterFront_CC);
	vRollCenterRear_WC.add(mCarbody.linPos);

	vRollCenterFront_WC.mult(mCarbody.rotPos, vRollCenterRear_CC);
	vRollCenterFront_WC.add(mCarbody.linPos);




	
#else 
	//Calc roll centers from svIC and fvIC
	//Intersections of the Instant Axis
	vRollCenterFront_WC = testFindIntersect(mSusp[0]->vSideViewIC_WC, mSusp[0]->vFrontViewIC_WC,mSusp[1]->vSideViewIC_WC, mSusp[1]->vFrontViewIC_WC );
	vRollCenterRear_WC = testFindIntersect(mSusp[2]->vSideViewIC_WC, mSusp[2]->vFrontViewIC_WC,mSusp[3]->vSideViewIC_WC, mSusp[3]->vFrontViewIC_WC );

	vRollCenterFront_CC = testFindIntersect(mSusp[0]->vSideViewIC_CC, mSusp[0]->vFrontViewIC_CC,mSusp[1]->vSideViewIC_CC, mSusp[1]->vFrontViewIC_CC );
	vRollCenterRear_CC = testFindIntersect(mSusp[2]->vSideViewIC_CC, mSusp[2]->vFrontViewIC_CC,mSusp[3]->vSideViewIC_CC, mSusp[3]->vFrontViewIC_CC );

	//Intersection of TireContacts to ICs

	Vector3p vSVRollAxisFront;
	Vector3p vSVRollAxisRear;
	vSVRollAxisFront = testFindIntersect(mWheels[0]->m_vWorldTireContact, mSusp[0]->vSideViewIC_WC,  mWheels[1]->m_vWorldTireContact, mSusp[1]->vSideViewIC_WC);
	vSVRollAxisRear = testFindIntersect(mWheels[2]->m_vWorldTireContact, mSusp[2]->vSideViewIC_WC,  mWheels[3]->m_vWorldTireContact, mSusp[3]->vSideViewIC_WC);


	Vector3p vFVRollAxisFront;
	Vector3p vFVRollAxisRear;
	vRollCenterFront_WC = testFindIntersect(mWheels[0]->m_vWorldTireContact, mSusp[0]->vFrontViewIC_WC,  mWheels[1]->m_vWorldTireContact, mSusp[1]->vFrontViewIC_WC);
	vRollCenterRear_WC = testFindIntersect(mWheels[2]->m_vWorldTireContact, mSusp[2]->vFrontViewIC_WC,  mWheels[3]->m_vWorldTireContact, mSusp[3]->vFrontViewIC_WC);

	vRollCenterFront_CC = testFindIntersect(mWheels[0]->m_vCCTireContact, mSusp[0]->vFrontViewIC_CC,  mWheels[1]->m_vCCTireContact, mSusp[1]->vFrontViewIC_CC);
	vRollCenterRear_CC = testFindIntersect(mWheels[2]->m_vCCTireContact, mSusp[2]->vFrontViewIC_CC,  mWheels[3]->m_vCCTireContact, mSusp[3]->vFrontViewIC_CC);


	
	vRollCenterFront_CC.z *=mSusp[0]->fRCZPos;
	vRollCenterRear_CC.z *= mSusp[2]->fRCZPos;

	//vRollCenterFront_WC.y = vFVRollAxisFront.y;
	//vRollCenterRear_WC.y = vFVRollAxisRear.y;

	//vRollCenterFront_WC = testFindIntersect(mWheels[0]->m_vWorldTireContact, mSusp[0]->vFrontViewIC_WC,  mWheels[1]->m_vWorldTireContact, mSusp[1]->vFrontViewIC_WC);
	//vRollCenterRear_WC = testFindIntersect(mWheels[2]->m_vWorldTireContact, mSusp[2]->vFrontViewIC_WC,  mWheels[3]->m_vWorldTireContact, mSusp[3]->vFrontViewIC_WC);




	/*vSVRollAxisFront = testFindIntersect(mWheels[0]->m_vWorldTireContact, mSusp[0]->vSideViewIC_CC,  mWheels[1]->m_vWorldTireContact, mSusp[1]->vSideViewIC_CC);
	vSVRollAxisRear = testFindIntersect(mWheels[2]->m_vWorldTireContact, mSusp[2]->vSideViewIC_CC,  mWheels[3]->m_vWorldTireContact, mSusp[3]->vSideViewIC_CC);
	vRollCenterFront_CC = testFindIntersect(mWheels[0]->m_vCCTireContact, mSusp[0]->vFrontViewIC_CC,  mWheels[1]->m_vCCTireContact, mSusp[1]->vFrontViewIC_CC);
	vRollCenterRear_CC = testFindIntersect(mWheels[2]->m_vCCTireContact, mSusp[2]->vFrontViewIC_CC,  mWheels[3]->m_vCCTireContact, mSusp[3]->vFrontViewIC_CC);

	vRollCenterFront_CC.z = vSVRollAxisFront.z;
	vRollCenterRear_CC.z = vSVRollAxisRear.z;*/
#endif 
	// now we can calculate jacking forces

}