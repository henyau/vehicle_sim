
#include "StdAfx.h"
#include "mxwheel.h"
#include "MxCarAttrib.h"
#include "Sim.h"

//dContactGeom CMxWheel::m_WheelContact;
CMxWheel::CMxWheel(CMxCarAttribute *_pCarAttr, CMxCar *car , CMxSusp *susp, int i)
{
	m_iWhich = i;
	pSusp = susp;
	pCar = car;
	Initialize();
	Reload(_pCarAttr);
}

CMxWheel::~CMxWheel(void)
{
}

void CMxWheel::Initialize()
{
	rotPos = 0.0f;
	rotVel = 0.0f;
	rotForce = 0.0f;

	torqueRollingResist = 0.0f;
}

void CMxWheel::Reload(CMxCarAttribute *_pCarAttr)
{
	//extern CMxCarAttribute _CarAttr;
	mass = _pCarAttr->m_Wheel[m_iWhich/2].rMass;
	oneOverMass = 1.0f/ mass;
	radius = _pCarAttr->m_Wheel[m_iWhich/2].rRadius;
	width =  _pCarAttr->m_Wheel[m_iWhich/2].rWidth;
	if(m_iWhich ==0|| m_iWhich ==2){
		toe =  -_pCarAttr->m_Wheel[m_iWhich/2].rToe;
		staticCamber =  -_pCarAttr->m_Wheel[m_iWhich/2].rCamber;
	}
	//right side camber and toe is reversed
	else if(m_iWhich ==1 || m_iWhich ==3){
		toe =  _pCarAttr->m_Wheel[m_iWhich/2].rToe;
		staticCamber =  _pCarAttr->m_Wheel[m_iWhich/2].rCamber;
	}
	momentOfInertia = mass * radius *radius * 0.5f;
	oneOverMOfI = 1.0f/momentOfInertia;
	 
	//only need one AckermanConstant, so moved to CMxCar.
	//still need rolling resistance const

	m_rRollingResistanceBasic = _pCarAttr->m_rRollingCoefBasic;//0.06f;
	m_rRollingResistanceSpeed = _pCarAttr->m_rRollingCoefSpeed;
	
 	axis = Vector3p(1,tan ((staticCamber*M_PI)/180),tan((toe*M_PI)/(180)));
	//axis = Vector3p(1,		tan ((staticCamber*M_PI)/180) +,		tan((toe*M_PI)/(180))) + ;
	// all suspension geometry is done in real time now
	axis.normalise();

	//setup the orientation of the wheel, no spin
	// set up back wheels rotation matrix. x axis of matrix is wheel axis. y axis is horizontal.
    float k = 1.0f / float(sqrt(axis.x * axis.x + axis.y * axis.y));
			
	if(m_iWhich ==2||m_iWhich ==0){
    noRotOrient = Matrix3x3(
        axis.x,	  -k * axis.y	,	-k * axis.x * axis.z,
        axis.y,		k * axis.x		,	-k * axis.y * axis.z,
        axis.z,		0.0				,	1.0f / k   );
}
// set up wheel on other side as if axis was negated
 	if(m_iWhich ==3||m_iWhich ==1){
     noRotOrient = Matrix3x3(
        -axis.x, k * axis.y, -k * axis.x * axis.z,
        -axis.y, -k * axis.x, -k * axis.y * axis.z,
        -axis.z, 0.0, 1.0f / k  );
	}

	tireSpringRate = _pCarAttr->m_Tire[0].rKp; //test in wheels
	
	m_rCornerStiffness = _pCarAttr->m_Tire[m_iWhich/2].rCorneringStiffness;
	m_rLongitudinalStiffness = _pCarAttr->m_Tire[m_iWhich/2].rLateralStiffness;

	tanSlipAngleVel1	=	tanSlipAngleVel2	=	tanSlipAngleVelT	=	tanSlipAngleVel	=
	tanSlipAnglePosT	=	tanSlipAnglePos	= 		slipRatioVel1	=	slipRatioVel2	=	
	slipRatioVelT	=	slipRatioVel	=	slipRatioPosT	=	slipRatioPos	=0.0f;

	rotForce = 0.0f;
	rotAccel = 0.0f;
	rotVel = 0.0f;
	
	torqueRollingResist = oldRotVel = rotPos0 =  rotVel0= rotVel1= rotVel2= rotVel3=rotAccel0= rotAccel1= rotAccel2= rotAccel3 =0;
	m_rRollingResistance = 0.0f;

	rotPos=  0.0f;
	forcePosWld = forcePosWld.ZERO;


	bLocked = false;
	oldLatForce = 0.0f;
	oldLongForce = 0.0f;
	Mz = 0.0f;

	load =0.0f;

	/////////
	//connections to diffs

	InputIn.brakingTorque = 0.0f; 
	InputIn.inertia =momentOfInertia;
	InputIn.rotVel = 0.0f;
	InputIn.torque = 0.0f;
	
	InputOut.brakingTorque = 0.0f; 
	InputOut.inertia =momentOfInertia;
	InputOut.rotVel = 0.0f;
	InputOut.torque = 0.0f;

	//
	m_tireContact.lastIndex = -1 ;//reset tri cache

	///pacejka tire params
	magicC = _pCarAttr->m_Tire[m_iWhich/2].magicC;
	magicE = -_pCarAttr->m_Tire[m_iWhich/2].magicE;
	kMuZ = _pCarAttr->m_Tire[m_iWhich/2].kMuZ;//note :: put into CMxWheel or tire
	kMuY = _pCarAttr->m_Tire[m_iWhich/2].kMuY;//note :: put into CMxWheel or tire
	mu0 = _pCarAttr->m_Tire[m_iWhich/2].rPrimaryFriction;//note :: put into CMxWheel or tire

	//relaxations
	vxMin =_pCarAttr->m_Tire[m_iWhich/2].vxMin;
	Lx = _pCarAttr->m_Tire[m_iWhich/2].Lx;
	Ly = _pCarAttr->m_Tire[m_iWhich/2].Ly;
	
	//CSim::m_Track->FindTriangleIntersect(positionWld,m_vDown,&m_tireContact.t,	&>m_tireContact.u, &m_tireContact.v, &m_tireContact.normal,&m_tireContact.triangleIndex);
	dynamicCamber = 0.0f;
	Update();
	getPosVel();

	m_CamberStiffness = 120.0f;
}


void CMxWheel::Update()
{//update position of wheel in CC.

	//position.mac(pSusp->position, pSusp->axis , -pSusp->restLength+pSusp->linearPosition); //in CC

	//position = pSusp->position;
//	position.mac(pSusp->position, pSusp->axis , -pSusp->restLength+pSusp->linearPosition); //in CC

	//if(m_iWhich&1)
	//	position.x +=0.7 * cos( -pSusp->restLength+pSusp->linearPosition);
	//else
	//	position.x -=0.7 * cos( -pSusp->restLength+pSusp->linearPosition);
	//position.y +=0.7 * sin( -pSusp->restLength+pSusp->linearPosition);

	position = pSusp->vKnucklePosCC;

 	//position.mac(pSusp->suspPos0 ,pSusp->axisCC , pSusp->linearPosition);
	//Vector3p suspRest;
	//suspRest.mac(pSusp->position, pSusp->axisCC, -pSusp->restLength);
	//position.mac(suspRest ,pSusp->axisCC , pSusp->linearPosition);

	//Vector3p suspPosCC;
	//suspPosCC.mult(pCar->mCarbody.rotPos, pSusp->position);
	//position.mac(pSusp->position,pSusp->axisCC , 3);
	//position.mac(suspPosCC,pSusp->axisCC , -pSusp->restLength+pSusp->linearPosition);


	//position.mac(pSusp->position ,pSusp->axisCC , pSusp->linearPosition);//3.10.05,
	//i seem to have mixed up suspPos0 and position. maybe elsewhere too.
	//suspPos0 is already transformed to carbody coords.

	oldRotVel = rotVel;
	getPosVel();

	//InputIn.brakingTorque = 0.0f; 
	//InputIn.inertia =momentOfInertia;
	//InputIn.rotVel = 0.0f;
	//InputIn.torque = 0.0f;
	
	//update connections
	InputOut.rotVel = rotVel;//reaction 
	InputOut.torque = -InputOut.brakingTorque;//torque from road/tire to wheel
	m_vWorldTireContact.mac( positionWld, m_vDown, m_tireContact.t);
	m_vCCTireContact.mac( position, m_vDown, m_tireContact.t);

}
void CMxWheel::getPosVel()
{
	//Vector3p diagWheelPos;                           // position in diag coords
	//diagWheelPos.mac(pSusp->suspPos0, pSusp->axis,pSusp->linearPosition);

	Vector3p diagWheelVel;   // velocity in diag coords
	diagWheelVel.cross(pCar->mCarbody.rotVel, position);
	diagWheelVel.mac(pSusp->axisWC, pSusp->linearVelocity);
	positionWld.transform(position, pCar->mCarbody.rotPos, pCar->mCarbody.linPos);
	velocityWld.transform(diagWheelVel, pCar->mCarbody.rotPos, pCar->mCarbody.linVel);
	
	//change axis to use suspension changes
	axisWorld.mult(pCar->mCarbody.rotPos, axis);
	
	
//
//	//hmm.. camber is only concerned about the FV. so should project to xy plane first... oh well for now
//	dynamicCamber = m_vDown.dot( m_tireContact.normal);
//	dynamicCamber = acos(dynamicCamber);
//	dynamicCamber = dynamicCamber - M_PI;
//
//	//adjust according to side
//	//transform to carspace
//	//if(m_vDown.dot(pCar->mCarbody.m_vDown)<0)
////	Vector3p downCarCoord;
//	//downCarCoord.transMult( noRotOrient, m_vDown);
////	downCarCoord.transMult(pCar->mCarbody.rotPos, m_vDown);
//
//	//project to x-z plane. subtract left from right. get line on axis. if positive, then positive camber. else negative
//	 
//	
//
//	if(m_iWhich&1)//odd
//	{
//		dynamicCamber*=-1;	
//	}
//
//	Vector3p vDownXZ = Vector3p(m_vDown.x , 0.0f, m_vDown.z);
//	//Vector3p vNormalXZ = Vector3p(m_tireContact.normal.x, 0.0f,m_tireContact.normal.z);
//	Vector3p vForwardXZ = Vector3p(finalNoSpin.m[2] ,0.0f, finalNoSpin.m[8]);
//	vDownXZ.normalise();
//	vForwardXZ.normalise();
//
//	if(vDownXZ.dot(vForwardXZ)<0)
//		dynamicCamber*=-1.0f;
//
//	//vNormalXZ.normalise();
//	//float upXZHeading = vDownXZ.dot(vNormalXZ);
//	//if(upXZHeading>0)
//	//		dynamicCamber*=-1.0f;
//	//if(!(m_iWhich&1)) //pos camber-->top moves away from car body
//	//	dynamicCamber*=-1.0f;
//
//
//
//
//	//RCVD tire model uses a "normalized" camber angle
//	if(load!=0)
//	{
//		normDynamicCamber = m_CamberStiffness*sin(dynamicCamber)/(kMuY*load);
//		normDynamicCamber = (normDynamicCamber)*180/M_PI;
//	}
//	else
//		normDynamicCamber = (dynamicCamber)*180/M_PI;

	//for GUI out
	//dynamicCamber = (dynamicCamber)*180/M_PI;

	CalcSlip();
}
//get the wheel in WORLD coords
void CMxWheel::getPosition(Vector3p &linearPos, Matrix3x3 &rotationalPos)
{
	//pCar->mCarbody///should do from pCar since uses same matrix for each wheel....
	//const float *bodyRot= dBodyGetRotation((pCar->mCarbody.odeBody)); //4x3
	//const float *bodyODELinPos= dBodyGetPosition(pCar->mCarbody.odeBody); //4x3
	//Matrix3x3 bodyMat; 
	//Vector3p bodyLinearPos;

	Vector3p diagWheelPos;
	Matrix3x3 rotPosSpin;
	float c, s;

	//move to Car class and update once per rk step
 //	bodyMat.m[0]= bodyRot[0];bodyMat.m[1]= bodyRot[1];bodyMat.m[2]= bodyRot[2];
	//bodyMat.m[3]= bodyRot[4];bodyMat.m[4]= bodyRot[5];bodyMat.m[5]= bodyRot[6];
	//bodyMat.m[6]= bodyRot[8];bodyMat.m[7]= bodyRot[9];bodyMat.m[8]= bodyRot[10];
 //   bodyLinearPos.x = bodyODELinPos[0];
	//bodyLinearPos.y = bodyODELinPos[1];
	//bodyLinearPos.z = bodyODELinPos[2];

//	diagWheelPos.mac(pSusp->suspPos0, pSusp->axis, pSusp->linearPosition);
	//position.transform(diagWheelPos, pCar->mCarbody.rotPos, pCar->mCarbody.linPos);
	//positionWld.transform(diagWheelPos, bodyMat,bodyLinearPos);


	//linearPos.transform(position, pCar->mCarbody.rotPos, pCar->mCarbody.linPos);//position is car coord
	linearPos.transform(position, pCar->mCarbody.rotPos, pCar->mCarbody.linPos);//position is car coord
	c = float(cos(rotPos) * radius);
	s = float(sin(rotPos) * radius);

	Matrix3x3 finalNoSpin;
		
	//if (m_iWhich & 1)                                  // reverse rotation on right side
	//{
	//	s = -s;
	//}

	finalNoSpin.mult(  noRotOrient, pSusp->mKnuckleRotPosWC);

	rotPosSpin.m[0] = finalNoSpin.m[0] * width;
    rotPosSpin.m[1] = finalNoSpin.m[1] * c + finalNoSpin.m[2] * s;
    rotPosSpin.m[2] = finalNoSpin.m[2] * c - finalNoSpin.m[1] * s;
    rotPosSpin.m[3] = finalNoSpin.m[3] * width;
    rotPosSpin.m[4] = finalNoSpin.m[4] * c + finalNoSpin.m[5] * s;
    rotPosSpin.m[5] = finalNoSpin.m[5] * c - finalNoSpin.m[4] * s;
    rotPosSpin.m[6] = finalNoSpin.m[6] * width;
    rotPosSpin.m[7] = finalNoSpin.m[7] * c + finalNoSpin.m[8] * s;
	rotPosSpin.m[8] = finalNoSpin.m[8] * c - finalNoSpin.m[7] * s;
	
	
	rotationalPos.mult( pCar->mCarbody.rotPos, rotPosSpin);
	//rotationalPos.mult(rotationalPos, pSusp->mKnuckleRotPosWC);
	//	positionWld.add( pCar->mCarbody.linPos, position);
	positionWld.transform(position, pCar->mCarbody.rotPos, pCar->mCarbody.linPos);

	rotPosWorld = rotationalPos;//test for graphics

	m_vDown = Vector3p(-noRotOrient.m[3], -noRotOrient.m[4],-noRotOrient.m[5]);
	m_vDown.mult(pSusp->mKnuckleRotPosWC, m_vDown); 
	m_vDown.mult( pCar->mCarbody.rotPos, m_vDown); 

	//m_vDown = Vector3p(-pSusp->mKnuckleRotPosWC.m[1], -pSusp->mKnuckleRotPosWC.m[4],-pSusp->mKnuckleRotPosWC.m[7]);
	//m_vDown.mult(noRotOrient, m_vDown); 
	//m_vDown.mult( pCar->mCarbody.rotPos, m_vDown); 

	axis = Vector3p(-noRotOrient.m[0], -noRotOrient.m[3],-noRotOrient.m[6]);
	//axis.mult(pSusp->mKnuckleRotPosWC, axis); 
	//axis.mult( pCar->mCarbody.rotPos, axis); 



	if(!(m_iWhich&1))//odd
	//{
	//	m_vDown.mult(-1.0f);
		axis.mult(-1.0f);
	//}

	/// FIX!!! Need to include the knucle transformation!!!!




	//	if(m_iWhich &1)
//	m_vDown = Vector3p(-finalNoSpin.m[1], -finalNoSpin.m[4],-finalNoSpin.m[7]);
	//m_vDown.mult(pCar->mCarbody.rotPos,m_vDown );
	//else
	//	m_vDown = Vector3p(finalNoSpin.m[1], finalNoSpin.m[4],finalNoSpin.m[7]);
	
	
	
	//axis = Vector3p(-finalNoSpin.m[0], -finalNoSpin.m[3],-finalNoSpin.m[6]);
	//account for suspension changes in toe.

 // 	positionWld.x =	(bodyODELinPos[0]+position.x);
	//positionWld.y =	(bodyODELinPos[1]+position.y);
	//positionWld.z =	(bodyODELinPos[2]+position.z);
//	positionWld.x = linearPos.x = 

	m_vDown.normalise();

	dynamicCamber = m_vDown.dot( m_tireContact.normal);
	//if(dynamicCamber<0)
	//	dynamicCamber = acos(dynamicCamber);

	dynamicCamber = acos(dynamicCamber);

	dynamicCamber = dynamicCamber - M_PI;
	
	if(m_iWhich&1)//odd
		dynamicCamber*=-1.0f;	
	if(m_vDown.x<0)//direction of camber
		dynamicCamber*=-1.0f;

	//Vector3p vDownXZ = Vector3p(m_vDown.x , 0.0f, m_vDown.z);
	//Vector3p vForwardXZ = Vector3p(-finalNoSpin.m[2] ,0.0f,-finalNoSpin.m[8]);
	////Vector3p vForwardXZ = Vector3p(0.0f ,0.0f, 1.0f);
	//vDownXZ.normalise();
	//vForwardXZ.normalise();

	////if(vDownXZ.dot(vForwardXZ)<0)

	//RCVD tire model uses a "normalized" camber angle
	if(fabs(load)<0.001f)
	{
 		normDynamicCamber = m_CamberStiffness*sin(dynamicCamber)/(kMuY*load);
		normDynamicCamber = (normDynamicCamber)*180/M_PI;
	}
	else
		normDynamicCamber = (dynamicCamber)*180/M_PI;



}

void CMxWheel::CalcSlip()
{//finds slip ratio and tan slip angle VELOCITY for use in integration

	if(m_tireContact.triangleIndex ==-1)
		return;
	float fabsVx;

	Vector3p fAxis, yAxis, sAxis;

	fAxis.cross(axisWorld, m_tireContact.normal);
    yAxis.cross(axisWorld, fAxis);
    sAxis.cross(fAxis, m_tireContact.normal);


	float	forwardVel = velocityWld.dot(fAxis);
	float sideVel = velocityWld.dot(sAxis);

    if (forwardVel > vxMin)                         // quadratic for fabsVx when small
        fabsVx = forwardVel;
    else if (forwardVel < -vxMin)
        fabsVx = -forwardVel;
    else
        fabsVx = (forwardVel * forwardVel / vxMin + vxMin) / 2.0f;


    tanSlipAngleVelT = (sideVel - fabsVx * tanSlipAnglePosT) / Ly;
	slipRatioVelT = (rotVel * radius - forwardVel- fabsVx * slipRatioPosT) / Lx;

	/////////

	//fAxis.y = 0.0f;
	//slipAnglePos2 = velocityWld.dot(fAxis);	
	//slipRatioPos2 = (rotVel *radius)/(forwardVel*slipRatioPos2) - 1.0f;//loaded radius ..m_tireContact.t
	//slipAnglePos2 = acos(slipAnglePos2);
}

/* //commented out August 22, 2008 use your brain numbnuts!
int CMxWheel::forces( Vector3p &fz)
{
	float Sx,Sy;
	float S;
	float F,Fx,Fy;
	float SxFactor,SyFactor;

	Vector3p forceDampingTC;
	Vector3p forceRoadTC;

	Sx=slipRatioPos;
	Sy=atan(tanSlipAnglePos);
	// Calculate total slip
	S=sqrt(Sx*Sx+Sy*Sy);

	
//	float planeDist = positionWld.dot(trackTirePlane.n) + trackTirePlane.k;
//	float planeDist = positionWld.y;
	float planeDist = m_tireContact.t;
	float depth =radius-planeDist; 
	load = depth*tireSpringRate;


	
	
	
	if(depth<0 || m_tireContact.triangleIndex ==-1) //need better rejection, this is just a sphere...
	{
		//tanSlipAngleVelT = 0.0f;
		//slipRatioVelT = 0.0f;
//		load = 0;
		load = 0;
		return 0;
	}

	//test virtools
	//fz = Vector3p(0,load,0);
	//return 1;

//	F = pTire->computeLongTireForce(load, Sx);//longi
// 	//float lateralForce = pTire->computeLatTireForce(planeReaction, staticCamber, 0.0f);//lat
//
////	if(fabs(S)>0.000005)
//	{
//		SxFactor=Sx/S;
//		SyFactor=Sy/S;
//		Fx=F*SxFactor;
//		Fy=F*SyFactor;
//	} 
	//else
	//{
	//	// Nearly no slip; no force
	//	Fx=Fy=0;
	//	SxFactor=0;
	//	SyFactor=0;
	//}

	//forceDampingTC.z*=fabs(SxFactor);
	//forceDampingTC.x*=fabs(SyFactor);



	//calc rolling resistance
	m_rRollingResistance = m_rRollingResistanceBasic +
			2.4248E-4 * m_rRollingResistanceSpeed *	velocityWld.mod2();
	if(rotVel>0)
		torqueRollingResist=-m_rRollingResistance*load;
	else
		torqueRollingResist=m_rRollingResistance*load;

	////surface specific rr

	{
		torqueRollingResist*=m_tireContact.fRollingResistanceFactor;
	}

//old static method
//	if(rotVel>0)
//		torqueRollingResist=-rollingCoeff*load*radius;
//	else
//		torqueRollingResist=rollingCoeff*load*radius;
//

	//set the tire contact position...ball wheel
	//Vector3p wheelCentToGround = trackTirePlane.n;
	Vector3p wheelCentToGround = m_tireContact.normal;
	wheelCentToGround.normalise();
	wheelCentToGround.mult(-radius);
	forcePosWld.add(positionWld,wheelCentToGround);
	//Vector3p wheelCentToGround(rotPosWorld.m[1], rotPosWorld.m[4],rotPosWorld.m[7]);
	//wheelCentToGround.normalise();
	//if(m_iWhich == 0 || m_iWhich == 2)
	//	wheelCentToGround.mult(radius);
	//else//(m_iWhich == 1 || m_iWhich == 3)
	//	wheelCentToGround.mult(-radius);
	//forcePosWld.add(positionWld,wheelCentToGround);




	////gravitational forces
	 //fz.y = load;

//	Vector3p fAxis, yAxis, sAxis; 
//	Vector3p tgiForce;
//
//	fAxis.cross(axisWorld, trackTirePlane.n);
//	yAxis.cross(axisWorld, fAxis);
//	sAxis.cross(fAxis, trackTirePlane.n);
//
//	float longForce2 = pTire->computeLongTireForce(load, Sx);//longi
//  	float latForce2 = pTire->computeLatTireForce(load, staticCamber, atan(tanSlipAnglePos));//lat
//	float Mz = pTire->computeMz(load, staticCamber, atan(tanSlipAnglePos));
//
//    tgiForce.mult(fAxis, longForce2);
//    tgiForce.mac(sAxis, latForce2);
//	fz.mac(tgiForce, trackTirePlane.n, load);
//
////	fz.x = fz.z = 0.0f;
//	debugForceLong.mult(fAxis, longForce2*0.01);
//	debugForceLat.mult(sAxis, latForce2*0.01);
//	debugForceLoad.mult(trackTirePlane.n, load*0.001 );
	
	Vector3p fAxis, yAxis, sAxis;
    
	fAxis.cross(axisWorld,m_tireContact.normal);
    yAxis.cross(axisWorld, fAxis);
    sAxis.cross(fAxis,m_tireContact.normal);

	float cosTheta = axisWorld.dot(m_tireContact.normal);             // theta = angle between wheel axis and plane normal
	cosTheta = fabs(cosTheta);

    Vector3p tgiForce;                      
    float forwardVel, sideVel;
    if (cosTheta > 0.9999 ) {
        forwardVel = 0.0;
        sideVel = 0.0;
    }
    else {
        float oneOverSinTheta = 1.0f / float(sqrt(1.0f - cosTheta * cosTheta));
        sAxis.mult(oneOverSinTheta);                // normalise these to length 1
        fAxis.mult(oneOverSinTheta);

        forwardVel = velocityWld.dot(fAxis); 
        sideVel = velocityWld.dot(sAxis);
    }

	//float fabsVx;
	//float vxMin = 10.00f;//add to CMxWheel or tire later
	//float Lx = 0.1f;////add to CMxWheel or tire later
	//float Ly = 0.1f;//add to CMxWheel or tire later
	float longForce  = 0.0f;
	float latForce = 0.0f;

    //if (forwardVel > vxMin)                         // quadratic for fabsVx when small
    //    fabsVx = forwardVel;
    //else if (forwardVel < -vxMin)
    //    fabsVx = -forwardVel;
    //else
    //    fabsVx = (forwardVel * forwardVel / vxMin + vxMin) / 2.0f;

 //   tanSlipAngleVelT = (sideVel - fabsVx * tanSlipAnglePosT) / Ly;
	//slipRatioVelT = (rotVel * radius - forwardVel- fabsVx * slipRatioPosT) / Lx;


	
	// find mu with load sensitivity
	
    float mux = mu0 * float(exp(-kMuZ * load));
    float muy = mux * kMuY;
	// test Surfaces
	mux *= (m_tireContact.fSurfaceFriction);
	muy *= (m_tireContact.fSurfaceFriction);

// find force from slip
	
 //     latStiffness=a3*sinf(2*atanf(Fz/a4))*(1.0f-a5*fabs(rCamber));
    //  B=latStiffness/(C*D);

	//m_rCornerStiffness=load*a3*sinf(2*atanf(load/a4))*(1.0f-a5*fabs(staticCamber));
////////

    float normSlipAngle; 
 	float normSlipRatio;
	normSlipAngle = m_rCornerStiffness * (tanSlipAnglePosT) / (muy * load);
	normSlipRatio = m_rLongitudinalStiffness * slipRatioPosT / (mux * load);//initial slope of long force *...

    float k2 = normSlipAngle * normSlipAngle + normSlipRatio * normSlipRatio;
    float k = float(sqrt(k2));
     const float kmin =0.0f;
 if (fabs(k) < 2.0f*M_PI) 
	//if(fabs(normSlipRatio)<0.35f)
	{// slip too small, use linear slip
     	longForce = m_rLongitudinalStiffness * slipRatioPosT*0.2f;

  	    //latForce = -m_rCornerStiffness * (tanSlipAnglePosT);

		float R = normSlipCurve(k, magicC, magicE);
        float phi = k2 / 4.0f;
        float ax = normSlipRatio * (m_rLongitudinalStiffness +  m_rCornerStiffness * phi);
        float ay = normSlipAngle * m_rLongitudinalStiffness * (1.0f + phi);
        float Rd = R / float(sqrt(ax * ax + ay * ay));
    
        float normFy = -Rd * ay;
		latForce = normFy * muy * load;
	}
	else 
	{
        float R = normSlipCurve(k, magicC, magicE);
        float phi = k2 / 4.0f;
        float ax = normSlipRatio * (m_rLongitudinalStiffness +  m_rCornerStiffness * phi);
        float ay = normSlipAngle * m_rLongitudinalStiffness * (1.0f + phi);
        float Rd = R / float(sqrt(ax * ax + ay * ay));
        float normFx = Rd * ax;
        float normFy = -Rd * ay;
        latForce = normFy * muy * load;
		//add camber thrust
		//if(slipRatioPosT>=0)
	
		//if(slipRatioPosT<0)
		//		latForce = (normFy - normDynamicCamber)/(1+ normDynamicCamber) * (muy * load);
		

	//	latForce *=0.5; //test
        longForce = normFx * mux * load;
  	}
		float camberStiffness = 0.2f;
		if(abs(dynamicCamber)<1.0f && fabs(k)>0.1f)
			latForce += camberStiffness*sin(dynamicCamber) * (muy * load);//approx

// find total force vector

   	 //add some damping to reduce those low speed occilations
	//if(oldLatForce*latForce<0.0f)
	//{
	//	latForce = 0.01f;//tire damping coeff	
	//}


	//if(slipRatioPosT<1.0f)
 //	if(normSlipRatio>10.0f)
	//{
	//	longForce = pTire->computeLongTireForce(load, slipRatioPosT);//longi
	//	latForce = 0.5*pTire->computeLatTireForce(load/1000.0f,  dynamicCamber, atan(tanSlipAnglePos));//lat
	//}
	//longForce = pTire->computeLongTireForce(load, slipRatioPosT);//longi
//	latForce = pTire->computeLatTireForce(load/1000.0f,  0.0f, atan(tanSlipAnglePos));//lat

	longForce = pTire->computeLongTireForce(load, slipRatioPosT);

	float longDamping = 2.0f*0.145f*forwardVel* sqrt((load*m_rLongitudinalStiffness) /(9.8f*10.1f));
    longDamping*=muy;
	//if (fabs(forwardVel)< 0.5f) 
	//{
	//	if(longForce<0)
	//		longForce += fabs(longDamping);
	//	else
	//		longForce -= fabs(longDamping);
	//}

	//if (k<0.5f )//lockup
	if(fabs(rotVel) < 0.001 )//|| fabs(atan(tanSlipAnglePosT)) > 3.14159/4.0f)//lockup
	 {//static friction
		// Vector3p diagWheelVel;   // velocity in diag coords
		//diagWheelVel.cross(pCar->mCarbody.rotVel, position);
		//diagWheelVel.mac(pSusp->axisWC, pSusp->linearVelocity);

		////car velocity vector
		//Vector3p *carVelocity = pCar->mCarbody.GetLinVel();
		////get wheel orientation in world space

		//float longVelocity = carVelocity->dot(fAxis); 
		//float latVelocity = carVelocity->dot(sAxis);

		//project force vector to long and lat directions of WHEEL
		 //float maxLongForce = -80;
		 //float maxLatForce = -50;
		 longForce = -load * mux * forwardVel;
		 latForce    = -load * muy * sideVel;
		 //scale to an ellipse or circle
		 float magnitude = sqrt(latForce*latForce + longForce*longForce);
		 longForce = (longForce/magnitude)*(longForce/magnitude)*(longForce/magnitude)*2000;
		 latForce = (latForce/magnitude)*(latForce/magnitude)*(latForce/magnitude)*1500;


	//	 float forceNorm = latForce*latForce + longForce*latForce;
	//	 if((forceNorm)>maxLongForce)
	//	 {
	//		 longForce = -maxLongForce*(longForce*longForce/(forceNorm));
	//		 latForce = -maxLongForce*(latForce*latForce/(forceNorm));
	//	 
	//	 }
	 }


	
  	oldLatForce = latForce;
	oldLongForce = longForce;//store forces

	

	tgiForce = tgiForce.ZERO;	 
	tgiForce.mult(fAxis, longForce);
	tgiForce.mac(sAxis, latForce);

	//fz.mac(tgiForce, trackTirePlane.n, load);
	fz.mac(tgiForce, m_tireContact.normal, load);

	///for debug display
	debugForceLong.mult(fAxis, longForce*0.01f);
	debugForceLat.mult(sAxis, latForce*0.01f);
	debugForceLoad.mult(m_tireContact.normal, load*0.001f);

	//aligning moment for ff
//	Mz =( pTire->computeMz(load, staticCamber, atan(tanSlipAnglePos)));
//	Mz =( pTire->computeMz(load/1000, dynamicCamber, atan(tanSlipAnglePos)));
	float pneumaticTrailLength = depth;//0.12f; //function of load?
	
	Mz = normLateralSlip(0.852, 2.3, 0.51, -2.75,atan(tanSlipAnglePos) )* mu0*pneumaticTrailLength*load;

	return 1;
}
*/
int CMxWheel::forces( Vector3p &fz)
//
{
	float Sx,Sy;
	float S;
	float F,Fx,Fy;
	float SxFactor,SyFactor;

	Vector3p forceDampingTC;
	Vector3p forceRoadTC;

	Sx=slipRatioPos;
	Sy=atan(tanSlipAnglePos);
	// Calculate total slip
	S=sqrt(Sx*Sx+Sy*Sy);

	float planeDist = m_tireContact.t;
	float depth =radius-planeDist; 
	load = depth*tireSpringRate;

	
	if(depth<0 || m_tireContact.triangleIndex ==-1) //need better rejection...
	{ //write a doughnut class later..
		load = 0;
		return 0;
	}

//	F = pTire->computeLongTireForce(load, Sx);//longi
// 	//float lateralForce = pTire->computeLatTireForce(planeReaction, staticCamber, 0.0f);//lat
//
////	if(fabs(S)>0.000005)
//	{
//		SxFactor=Sx/S;
//		SyFactor=Sy/S;
//		Fx=F*SxFactor;
//		Fy=F*SyFactor;
//	} 
	//else
	//{
	//	// Nearly no slip; no force
	//	Fx=Fy=0;
	//	SxFactor=0;
	//	SyFactor=0;
	//}

	//forceDampingTC.z*=fabs(SxFactor);
	//forceDampingTC.x*=fabs(SyFactor);

	//calc rolling resistance
	m_rRollingResistance = m_rRollingResistanceBasic +2.4248E-4 * m_rRollingResistanceSpeed *velocityWld.mod2();
	if(rotVel>0)
		torqueRollingResist=-m_rRollingResistance*load;
	else
		torqueRollingResist=m_rRollingResistance*load;
	////surface specific rr
	torqueRollingResist*=m_tireContact.fRollingResistanceFactor;

//old static method
//	if(rotVel>0)
//		torqueRollingResist=-rollingCoeff*load*radius;
//	else
//		torqueRollingResist=rollingCoeff*load*radius;
//

	//set the tire contact position...ball wheel
	//Vector3p wheelCentToGround = trackTirePlane.n;
	Vector3p wheelCentToGround = m_tireContact.normal;
	wheelCentToGround.normalise();
	wheelCentToGround.mult(-radius);
	forcePosWld.add(positionWld,wheelCentToGround);
	//Vector3p wheelCentToGround(rotPosWorld.m[1], rotPosWorld.m[4],rotPosWorld.m[7]);
	//wheelCentToGround.normalise();
	//if(m_iWhich == 0 || m_iWhich == 2)
	//	wheelCentToGround.mult(radius);
	//else//(m_iWhich == 1 || m_iWhich == 3)
	//	wheelCentToGround.mult(-radius);
	//forcePosWld.add(positionWld,wheelCentToGround);




	////gravitational forces
	 //fz.y = load;

//	Vector3p fAxis, yAxis, sAxis; 
//	Vector3p tgiForce;
//
//	fAxis.cross(axisWorld, trackTirePlane.n);
//	yAxis.cross(axisWorld, fAxis);
//	sAxis.cross(fAxis, trackTirePlane.n);
//
//	float longForce2 = pTire->computeLongTireForce(load, Sx);//longi
//  	float latForce2 = pTire->computeLatTireForce(load, staticCamber, atan(tanSlipAnglePos));//lat
//	float Mz = pTire->computeMz(load, staticCamber, atan(tanSlipAnglePos));
//
//    tgiForce.mult(fAxis, longForce2);
//    tgiForce.mac(sAxis, latForce2);
//	fz.mac(tgiForce, trackTirePlane.n, load);
//
////	fz.x = fz.z = 0.0f;
//	debugForceLong.mult(fAxis, longForce2*0.01);
//	debugForceLat.mult(sAxis, latForce2*0.01);
//	debugForceLoad.mult(trackTirePlane.n, load*0.001 );
	
	Vector3p fAxis, yAxis, sAxis;
    
	fAxis.cross(axisWorld,m_tireContact.normal);
    yAxis.cross(axisWorld, fAxis);
    sAxis.cross(fAxis,m_tireContact.normal);

	float cosTheta = axisWorld.dot(m_tireContact.normal);             // theta = angle between wheel axis and plane normal
	cosTheta = fabs(cosTheta);

    Vector3p tgiForce;                      
    float forwardVel, sideVel;
    if (cosTheta > 0.9999 ) {
        forwardVel = 0.0;
        sideVel = 0.0;
    }
    else {
        float oneOverSinTheta = 1.0f / float(sqrt(1.0f - cosTheta * cosTheta));
        sAxis.mult(oneOverSinTheta);                // normalise these to length 1
        fAxis.mult(oneOverSinTheta);

        forwardVel = velocityWld.dot(fAxis); 
        sideVel = velocityWld.dot(sAxis);
    }

	float longForce  = 0.0f;
	float latForce = 0.0f;

    //if (forwardVel > vxMin)                         // quadratic for fabsVx when small
    //    fabsVx = forwardVel;
    //else if (forwardVel < -vxMin)
    //    fabsVx = -forwardVel;
    //else
    //    fabsVx = (forwardVel * forwardVel / vxMin + vxMin) / 2.0f;

 //   tanSlipAngleVelT = (sideVel - fabsVx * tanSlipAnglePosT) / Ly;
	//slipRatioVelT = (rotVel * radius - forwardVel- fabsVx * slipRatioPosT) / Lx;


	
	// find mu with load sensitivity
	
    float mux = mu0 * float(exp(-kMuZ * load));
    float muy = mux * kMuY;
	// test Surfaces
	mux *= (m_tireContact.fSurfaceFriction);
	muy *= (m_tireContact.fSurfaceFriction);

// find force from slip
	
 //     latStiffness=a3*sinf(2*atanf(Fz/a4))*(1.0f-a5*fabs(rCamber));
    //  B=latStiffness/(C*D);

	//m_rCornerStiffness=load*a3*sinf(2*atanf(load/a4))*(1.0f-a5*fabs(staticCamber));
////////

    float normSlipAngle; 
 	float normSlipRatio;
	normSlipAngle = m_rCornerStiffness * (tanSlipAnglePosT) / (muy * load);
	normSlipRatio = m_rLongitudinalStiffness * slipRatioPosT / (mux * load);//initial slope of long force *...

    float k2 = normSlipAngle * normSlipAngle + normSlipRatio * normSlipRatio;
    float k = float(sqrt(k2));
     const float kmin =0.0f;

	if (fabs(k) < 2.0f*M_PI) 
	{// slip too small, use linear slip
     	longForce = m_rLongitudinalStiffness * slipRatioPosT*0.2f;

  	    //latForce = -m_rCornerStiffness * (tanSlipAnglePosT);

		float R = normSlipCurve(k, magicC, magicE);
        float phi = k2 / 4.0f;
        float ax = normSlipRatio * (m_rLongitudinalStiffness +  m_rCornerStiffness * phi);
        float ay = normSlipAngle * m_rLongitudinalStiffness * (1.0f + phi);
        float Rd = R / float(sqrt(ax * ax + ay * ay));
    
        float normFy = -Rd * ay;
		latForce = normFy * muy * load;
	}
	else 
	{
        float R = normSlipCurve(k, magicC, magicE);
        float phi = k2 / 4.0f;
        float ax = normSlipRatio * (m_rLongitudinalStiffness +  m_rCornerStiffness * phi);
        float ay = normSlipAngle * m_rLongitudinalStiffness * (1.0f + phi);
        float Rd = R / float(sqrt(ax * ax + ay * ay));
        float normFx = Rd * ax;
        float normFy = -Rd * ay;
        latForce = normFy * muy * load;
		//add camber thrust
		//if(slipRatioPosT>=0)
	
		//if(slipRatioPosT<0)
		//		latForce = (normFy - normDynamicCamber)/(1+ normDynamicCamber) * (muy * load);
		

	//	latForce *=0.5; //test
        longForce = normFx * mux * load;
  	}
		float camberStiffness = 0.2f;
		if(abs(dynamicCamber)<1.0f && fabs(k)>0.1f)
			latForce += camberStiffness*sin(dynamicCamber) * (muy * load);//approx

	longForce = pTire->computeLongTireForce(load, slipRatioPosT);

	float longDamping = 2.0f*0.145f*forwardVel* sqrt((load*m_rLongitudinalStiffness) /(9.8f*10.1f));
    longDamping*=muy;

	//aligning moment for ff
//	Mz =( pTire->computeMz(load, staticCamber, atan(tanSlipAnglePos)));
//	Mz =( pTire->computeMz(load/1000, dynamicCamber, atan(tanSlipAnglePos)));
	float pneumaticTrailLength = depth;//0.12f; //function of load?
	Mz = normLateralSlip(0.852, 2.3, 0.51, -2.75,atan(tanSlipAnglePos) )* mu0*pneumaticTrailLength*load;


	//if(fabs(rotVel) < 0.001 && fabs(velocityWld.mod())<1 )//|| fabs(atan(tanSlipAnglePosT)) > 3.14159/4.0f)//lockup
		if(fabs(rotVel) < 0.001 && fabs(velocityWld.mod())<1)
		 //need to put in condition of brake input, otherwise rolling action will be nullified.
	 {//static friction
		 //longForce = load * mux * forwardVel;
		 //latForce    = load * muy * sideVel;
		 ////scale to an ellipse or circle
		 //float magnitude = sqrt(latForce*latForce + longForce*longForce);
		 //longForce = -(longForce/magnitude)*longForce;
		 //latForce = -(latForce/magnitude)*latForce;

   //      //test just using coulomb friction
		 //float coulomblongForce = load*mux;
		 //float coulomblatForce = load*muy;
		
		 tgiForce = velocityWld;
		// tgiForce.normalise();
    	 //tgiForce.neg(); 
		 //tgiForce.mult(0.5*load);

		// float mag = tgiForce.mod();
		//Vector3p groundVect;
		//tgiForce.normalise();
		//wheelCentToGround.normalise();
		//groundVect.cross(tgiForce,wheelCentToGround);
		//groundVect.cross(wheelCentToGround,groundVect);

		////float projection = tgiForce.dot(groundVect);

		//tgiForce = groundVect;
		////tgiForce.mult(projection);
		//// 
		tgiForce.normalise();
    	tgiForce.neg(); 
		 tgiForce.mult(5*load*velocityWld.mod()); //damping coefficient * load * -sgn(velocity)*velocity

		

		//tgiForce.mult(load*muy*mag);

		 //tgiForce.normalise();
		//tgiForce.mac(sAxis, latForce);

		fz.mac(tgiForce, m_tireContact.normal, load);

		//Does a tire have more lateral traction during lock up than in the longitudinal direction?
		//May make sense since contact patch is "wider".

		//In any case, this class needs to be re written with the notion that tractive forces always exist when there is some opposing forces.
		// In the case of lock up and there is no forces acting on the car (in a direction which would generation motion)
		

		 return 1;
	 }

  	oldLatForce = latForce;
 	oldLongForce = longForce;//store forces
	
	tgiForce = tgiForce.ZERO;	 
	tgiForce.mult(fAxis, longForce);
	tgiForce.mac(sAxis, latForce);

	fz.mac(tgiForce, m_tireContact.normal, load);

	///for debug display
	debugForceLong.mult(fAxis, longForce*0.01f);
	debugForceLat.mult(sAxis, latForce*0.01f);
	debugForceLoad.mult(m_tireContact.normal, load*0.001f);

	
	return 1;
}

/*
int CMxWheel::forces( Vector3p &fz)
{
	float planeDist = m_tireContact.t;
	float depth =radius-planeDist; 

	
	if(depth<0 || m_tireContact.triangleIndex ==-1) //need better rejection, this is just a sphere...
	{
		load = 0;
		return 0;
	}
		
	load = depth*tireSpringRate;	
	float Sx,Sy;
	float S;
	float F,Fx,Fy;
	float SxFactor,SyFactor;

	Vector3p forceDampingTC;
	Vector3p forceRoadTC;
	Sx=slipRatioPos;
	Sy=atan(tanSlipAnglePos);
	S=sqrt(Sx*Sx+Sy*Sy);

	m_rRollingResistance = m_rRollingResistanceBasic +
			2.4248E-4 * m_rRollingResistanceSpeed *	velocityWld.mod2();
	if(rotVel>0)
		torqueRollingResist=-m_rRollingResistance*load;
	else
		torqueRollingResist=m_rRollingResistance*load;

	////surface specific rr
	torqueRollingResist*=m_tireContact.fRollingResistanceFactor;

	Vector3p wheelCentToGround = m_tireContact.normal;
	wheelCentToGround.normalise();
	wheelCentToGround.mult(-radius);
	forcePosWld.add(positionWld,wheelCentToGround);
	
	Vector3p fAxis, yAxis, sAxis;    
	fAxis.cross(axisWorld,m_tireContact.normal);
    yAxis.cross(axisWorld, fAxis);
    sAxis.cross(fAxis,m_tireContact.normal);

	float cosTheta = axisWorld.dot(m_tireContact.normal);  // theta = angle between wheel axis and plane normal
	cosTheta = fabs(cosTheta);

    Vector3p tgiForce;                      
    float forwardVel, sideVel;
    if (cosTheta > 0.9999 ) {
        forwardVel = 0.0;
        sideVel = 0.0;
    }
    else {
        float oneOverSinTheta = 1.0f / float(sqrt(1.0f - cosTheta * cosTheta));
        sAxis.mult(oneOverSinTheta);                // normalise these to length 1
        fAxis.mult(oneOverSinTheta);

        forwardVel = velocityWld.dot(fAxis); 
        sideVel = velocityWld.dot(sAxis);
    }


	float longForce  = 0.0f;
	float latForce = 0.0f;
	// find mu with load sensitivity
	
    float mux = mu0 * float(exp(-kMuZ * load));
    float muy = mux * kMuY;
	// test Surfaces
	mux *= (m_tireContact.fSurfaceFriction);
	muy *= (m_tireContact.fSurfaceFriction);

// find force from slip
	
 //     latStiffness=a3*sinf(2*atanf(Fz/a4))*(1.0f-a5*fabs(rCamber));
    //  B=latStiffness/(C*D);

	//m_rCornerStiffness=load*a3*sinf(2*atanf(load/a4))*(1.0f-a5*fabs(staticCamber));
////////

    float normSlipAngle; 
 	float normSlipRatio;
	normSlipAngle = m_rCornerStiffness * (tanSlipAnglePosT) / (muy * load);
	normSlipRatio = m_rLongitudinalStiffness * slipRatioPosT / (mux * load);//initial slope of long force *...

    float k2 = normSlipAngle * normSlipAngle + normSlipRatio * normSlipRatio;
    float k = float(sqrt(k2));
	const float kmin =0.0f;
	float Eta_0 = (m_rCornerStiffness*mux)/(muy*m_rLongitudinalStiffness);
	
	float Eta_k;
	if (fabs(k) <= 2.0f*M_PI) 
	{// slip too small, use linear slip
		Eta_k = 0.5 * (1+Eta_0 ) - 0.5 * (1- Eta_0) * cos(k/2);
	}
	else 
	{
		Eta_k =1.0f;
  	}

	//float camberStiffness = 0.2f;
	//if(abs(dynamicCamber)<1.0f && fabs(k)>0.1f)
	//	latForce += camberStiffness*sin(dynamicCamber) * (muy * load);//approx

	longForce = pTire->computeLongTireForce(load, slipRatioPosT);//load*tanSlipAnglePos * Eta_k *(1.0f/(normSlipRatio*normSlipRatio + Eta_k*Eta_k*(tan(normSlipAngle))*(tan(normSlipAngle))));
	latForce = pTire->computeLatTireForce(load, dynamicCamber*0.1f,  atan(tanSlipAnglePos) );//lat, lessen the influence on camber. 

// find total force vector

   	 //add some damping to reduce those low speed occilations
	//if(oldLatForce*latForce<0.0f)
	//{
	//	latForce = 0.01f;//tire damping coeff	
	//}


	//if(slipRatioPosT<1.0f)
 //	if(normSlipRatio>10.0f)
	//{
	//	longForce = pTire->computeLongTireForce(load, slipRatioPosT);//longi
	//	latForce = 0.5*pTire->computeLatTireForce(load/1000.0f,  dynamicCamber, atan(tanSlipAnglePos));//lat
	//}
	//longForce = pTire->computeLongTireForce(load, slipRatioPosT);//longi
//	latForce = pTire->computeLatTireForce(load/1000.0f,  0.0f, atan(tanSlipAnglePos));//lat

	float longDamping = 2.0f*0.145f*forwardVel* sqrt((load*m_rLongitudinalStiffness) /(9.8f*10.1f));
    longDamping*=muy;
	if (fabs(forwardVel)< 0.5f) 
	{
		if(longForce<0)
			longForce += fabs(longDamping);
		else
			longForce -= fabs(longDamping);
	}
	



  	oldLatForce = latForce;
	oldLongForce = longForce;//store forces

	tgiForce = tgiForce.ZERO;	 
	tgiForce.mult(fAxis, longForce);
	tgiForce.mac(sAxis, latForce);

	fz.mac(tgiForce, m_tireContact.normal, load);
	///for debug display
	debugForceLong.mult(fAxis, longForce*0.01f);
	debugForceLat.mult(sAxis, latForce*0.01f);
	debugForceLoad.mult(m_tireContact.normal, load*0.001f);

	//aligning moment for ff
//	Mz =( pTire->computeMz(load, staticCamber, atan(tanSlipAnglePos)));
//	Mz =( pTire->computeMz(load/1000, dynamicCamber, atan(tanSlipAnglePos)));
	float pneumaticTrailLength = depth*0.9f;//0.12f; //function of load?
	
	Mz = normLateralSlip(0.852, 2.3, 0.51, -2.75,atan(tanSlipAnglePos) )* mu0*pneumaticTrailLength*load;

	return 1;
}

*/
/*
int CMxWheel::forces( Vector3p &fz)
{
	float Sx,Sy;
	float S;
	float F,Fx,Fy;
	float SxFactor,SyFactor;

	Vector3p forceDampingTC;
	Vector3p forceRoadTC;

	Sx=slipRatioPos;
	Sy=atan(tanSlipAnglePos)*180.0f/M_PI;
	S=sqrt(Sx*Sx+Sy*Sy);//combined slipratio/angle

	float planeDist = m_tireContact.t;
	float depth =radius-planeDist; 
	load = depth*tireSpringRate;
	
	
	if(depth<0 || m_tireContact.triangleIndex ==-1) //need better rejection, this is just a sphere...
	{
		load = 0;
		return 0;
	}

	m_rRollingResistance = m_rRollingResistanceBasic +
			2.4248E-4 * m_rRollingResistanceSpeed *	velocityWld.mod2();
	if(rotVel>0)
		torqueRollingResist=-m_rRollingResistance*load;
	else
		torqueRollingResist=m_rRollingResistance*load;
	torqueRollingResist*=m_tireContact.fRollingResistanceFactor;
	

	Vector3p wheelCentToGround = m_tireContact.normal;
	wheelCentToGround.normalise();
	wheelCentToGround.mult(-radius);
	forcePosWld.add(positionWld,wheelCentToGround);
	
	Vector3p fAxis, yAxis, sAxis;    
	fAxis.cross(axisWorld,m_tireContact.normal);
    yAxis.cross(axisWorld, fAxis);
    sAxis.cross(fAxis,m_tireContact.normal);

	float cosTheta = axisWorld.dot(m_tireContact.normal);             // theta = angle between wheel axis and plane normal
	cosTheta = fabs(cosTheta);

    Vector3p tgiForce;                      

	float oneOverSinTheta = 1.0f / float(sqrt(1.0f - cosTheta * cosTheta));
	sAxis.mult(oneOverSinTheta);                // normalise these to length 1
	fAxis.mult(oneOverSinTheta);
	float forwardVel = velocityWld.dot(fAxis);    
	float sideVel = velocityWld.dot(sAxis);

	float longForce  =  pTire->computeLongTireForce(load, slipRatioPosT);//longi
	float latForce = pTire->computeLatTireForce(load/1000, 0, atan(tanSlipAnglePosT));
///do damping
	float dampingSpeed = 1.6;
	float dampingCoefficientLong = 0.4f;
	float dampingCoefficientLat = 0.8f;

	//b=relaxationLengthLat;
	//B=relaxationLengthLong;

  if(fabs(forwardVel)<dampingSpeed)
	{
		latForce -=dampingCoefficientLat*pTire->computeLatTireForce(load/1000, 0, atan(tanSlipAnglePosT));
	}

  // Longitudinal damping
  if(fabs(forwardVel)<dampingSpeed)
  {
   //longForce -=2.0f*dampingCoefficientLong*forwardVel* sqrt((load*m_rLongitudinalStiffness)/(9.8f*Ly))*mu0;
	  longForce -=dampingCoefficientLong*pTire->computeLongTireForce(load, slipRatioPosT);

  }


	///


  	oldLatForce = latForce;
	oldLongForce = longForce;//store forces

	tgiForce = tgiForce.ZERO;	 
	tgiForce.mult(fAxis, longForce);
	tgiForce.mac(sAxis, latForce);

	//fz.mac(tgiForce, trackTirePlane.n, load);
	fz.mac(tgiForce, m_tireContact.normal, load);

	///for debug display
	debugForceLong.mult(fAxis, longForce*0.01f);
	debugForceLat.mult(sAxis, latForce*0.01f);
	debugForceLoad.mult(m_tireContact.normal, load*0.001f);



	//aligning moment for ff
	float pneumaticTrailLength = 1.2f; //function of load?
	Mz = normLateralSlip(0.852, 2.3, 0.51, -2.75,atan(tanSlipAnglePos) )* mu0*pneumaticTrailLength*load;
	return 1;
}
*/


//Pacejka's magic formula normalized (stiffnesses multiplied later)
float CMxWheel::normSlipCurve(float k, float C, float E)
{
    float Bx = k / C;
    return float(sin(C * float(atan(Bx - E * (Bx - float(atan(Bx)))))));	
}
float CMxWheel::normLateralSlip(float B, float C, float D, float E,float alpha)
{
	float phi = (1-E)*alpha + (E/B)*atan(B*alpha);
	float theta = C*atan(B*phi);
	return D*sin(theta);
}

void CMxWheel::RKInit()
{
	tanSlipAnglePosT = tanSlipAnglePos;
	slipRatioPosT = slipRatioPos;
}

void CMxWheel::EulerStep()
{//test for now
	RKStep1();
	RKStep2();
	RKStep3();
	RKStep4();

}
void CMxWheel::RKStep1()
{
	tanSlipAngleVel = tanSlipAngleVelT;
	tanSlipAnglePosT= tanSlipAnglePos + CSim::RKh2 * tanSlipAngleVel;
	slipRatioVel = slipRatioVelT;
	slipRatioPosT = slipRatioPos + CSim::RKh2 * slipRatioVel;
}

void CMxWheel::RKStep2()
{
	tanSlipAngleVel1 = tanSlipAngleVelT;
	tanSlipAnglePosT= tanSlipAnglePos + CSim::RKh2 * tanSlipAngleVel1;
	slipRatioVel1 = slipRatioVelT;
	slipRatioPosT = slipRatioPos + CSim::RKh2 * slipRatioVel1;
}

void CMxWheel::RKStep3()
{
	tanSlipAngleVel2 = tanSlipAngleVelT;
	tanSlipAnglePosT = tanSlipAnglePos + CSim::RKh * tanSlipAngleVel2;
	tanSlipAngleVel2+= tanSlipAngleVel1;
	slipRatioVel2 = slipRatioVelT;
	slipRatioPosT = slipRatioPos + CSim::RKh * slipRatioVel2;
	slipRatioVel2+= slipRatioVel1;
}

void CMxWheel::RKStep4()
{
	float slipMax = 100.0;

    tanSlipAnglePos += CSim::RKh6 * (tanSlipAngleVel + tanSlipAngleVelT + 2.0f * tanSlipAngleVel2);
    if (tanSlipAnglePos > slipMax)
        tanSlipAnglePos = slipMax;
    else if (tanSlipAnglePos < -slipMax)
        tanSlipAnglePos = -slipMax;

    slipRatioPos += CSim::RKh6 * (slipRatioVel + slipRatioVelT + 2.0f * slipRatioVel2);
    if (slipRatioPos > slipMax)
        slipRatioPos = slipMax;
    else if (slipRatioPos < -slipMax)
        slipRatioPos = -slipMax;


	
}


float CMxWheel::calcWheelAccel()
{
	return 1.0f;
}




/*


//////////Rewrite
#include "StdAfx.h"
#include "mxwheel.h"
#include "MxCarAttrib.h"
#include "Sim.h"

//dContactGeom CMxWheel::m_WheelContact;
CMxWheel::CMxWheel(CMxCarAttribute *_pCarAttr, CMxCar *car , CMxSusp *susp, int i)
{
	m_iWhich = i;
	pSusp = susp;
	pCar = car;
	Initialize();
	Reload(_pCarAttr);
}

CMxWheel::~CMxWheel(void)
{
}

void CMxWheel::Initialize()
{
	rotPos = 0.0f;
	rotVel = 0.0f;
	rotForce = 0.0f;

	torqueRollingResist = 0.0f;
}

void CMxWheel::Reload(CMxCarAttribute *_pCarAttr)
{
	//extern CMxCarAttribute _CarAttr;
	mass = _pCarAttr->m_Wheel[m_iWhich/2].rMass;
	oneOverMass = 1.0f/ mass;
	radius = _pCarAttr->m_Wheel[m_iWhich/2].rRadius;
	width =  _pCarAttr->m_Wheel[m_iWhich/2].rWidth;
	if(m_iWhich ==0|| m_iWhich ==2){
		toe =  -_pCarAttr->m_Wheel[m_iWhich/2].rToe;
		staticCamber =  -_pCarAttr->m_Wheel[m_iWhich/2].rCamber;
	}
	//right side camber and toe is reversed
	else if(m_iWhich ==1 || m_iWhich ==3){
		toe =  _pCarAttr->m_Wheel[m_iWhich/2].rToe;
		staticCamber =  _pCarAttr->m_Wheel[m_iWhich/2].rCamber;
	}
	momentOfInertia = mass * radius *radius * 0.5f;
	oneOverMOfI = 1.0f/momentOfInertia;
	 
	//only need one AckermanConstant, so moved to CMxCar.
	//still need rolling resistance const

	m_rRollingResistanceBasic = _pCarAttr->m_rRollingCoefBasic;//0.06f;
	m_rRollingResistanceSpeed = _pCarAttr->m_rRollingCoefSpeed;
	
 	axis = Vector3p(1,tan ((staticCamber*M_PI)/180),tan((toe*M_PI)/(180)));
	//axis = Vector3p(1,		tan ((staticCamber*M_PI)/180) +,		tan((toe*M_PI)/(180))) + ;
	// all suspension geometry is done in real time now
	axis.normalise();

	//setup the orientation of the wheel, no spin
	// set up back wheels rotation matrix. x axis of matrix is wheel axis. y axis is horizontal.
    float k = 1.0f / float(sqrt(axis.x * axis.x + axis.y * axis.y));
			
	if(m_iWhich ==2||m_iWhich ==0){
    noRotOrient = Matrix3x3(
        axis.x,	  -k * axis.y	,	-k * axis.x * axis.z,
        axis.y,		k * axis.x		,	-k * axis.y * axis.z,
        axis.z,		0.0				,	1.0f / k   );
}
// set up wheel on other side as if axis was negated
 	if(m_iWhich ==3||m_iWhich ==1){
     noRotOrient = Matrix3x3(
        -axis.x, k * axis.y, -k * axis.x * axis.z,
        -axis.y, -k * axis.x, -k * axis.y * axis.z,
        -axis.z, 0.0, 1.0f / k  );
	}

	tireSpringRate = _pCarAttr->m_Tire[0].rKp; //test in wheels
	
	m_rCornerStiffness = _pCarAttr->m_Tire[m_iWhich/2].rCorneringStiffness;
	m_rLongitudinalStiffness = _pCarAttr->m_Tire[m_iWhich/2].rLateralStiffness;

	tanSlipAngleVel1	=	tanSlipAngleVel2	=	tanSlipAngleVelT	=	tanSlipAngleVel	=
	tanSlipAnglePosT	=	tanSlipAnglePos	= 		slipRatioVel1	=	slipRatioVel2	=	
	slipRatioVelT	=	slipRatioVel	=	slipRatioPosT	=	slipRatioPos	=0.0f;

	rotForce = 0.0f;
	rotAccel = 0.0f;
	rotVel = 0.0f;
	
	torqueRollingResist = oldRotVel = rotPos0 =  rotVel0= rotVel1= rotVel2= rotVel3=rotAccel0= rotAccel1= rotAccel2= rotAccel3 =0;
	m_rRollingResistance = 0.0f;

	rotPos=  0.0f;
	forcePosWld = forcePosWld.ZERO;


	bLocked = false;
	oldLatForce = 0.0f;
	oldLongForce = 0.0f;
	Mz = 0.0f;

	load =0.0f;

	/////////
	//connections to diffs

	InputIn.brakingTorque = 0.0f; 
	InputIn.inertia =momentOfInertia;
	InputIn.rotVel = 0.0f;
	InputIn.torque = 0.0f;
	
	InputOut.brakingTorque = 0.0f; 
	InputOut.inertia =momentOfInertia;
	InputOut.rotVel = 0.0f;
	InputOut.torque = 0.0f;

	//
	m_tireContact.lastIndex = -1 ;//reset tri cache

	///pacejka tire params
	magicC = _pCarAttr->m_Tire[m_iWhich/2].magicC;
	magicE = -_pCarAttr->m_Tire[m_iWhich/2].magicE;
	kMuZ = _pCarAttr->m_Tire[m_iWhich/2].kMuZ;//note :: put into CMxWheel or tire
	kMuY = _pCarAttr->m_Tire[m_iWhich/2].kMuY;//note :: put into CMxWheel or tire
	mu0 = _pCarAttr->m_Tire[m_iWhich/2].rPrimaryFriction;//note :: put into CMxWheel or tire

	//relaxations
	vxMin =_pCarAttr->m_Tire[m_iWhich/2].vxMin;
	Lx = _pCarAttr->m_Tire[m_iWhich/2].Lx;
	Ly = _pCarAttr->m_Tire[m_iWhich/2].Ly;
	
	//CSim::m_Track->FindTriangleIntersect(positionWld,m_vDown,&m_tireContact.t,	&>m_tireContact.u, &m_tireContact.v, &m_tireContact.normal,&m_tireContact.triangleIndex);
	dynamicCamber = 0.0f;
	Update();
	getPosVel();

	m_CamberStiffness = 120.0f;
}


void CMxWheel::Update()
{
	position = pSusp->vKnucklePosCC;
	oldRotVel = rotVel;
	getPosVel();
	InputOut.rotVel = rotVel;//reaction 
	InputOut.torque = -InputOut.brakingTorque;//torque from road/tire to wheel
	m_vWorldTireContact.mac( positionWld, m_vDown, m_tireContact.t);
	m_vCCTireContact.mac( position, m_vDown, m_tireContact.t);


}
void CMxWheel::getPosVel()
{
	Vector3p diagWheelVel;   // velocity in diag coords
	diagWheelVel.cross(pCar->mCarbody.rotVel, position);
	diagWheelVel.mac(pSusp->axisWC, pSusp->linearVelocity);
	positionWld.transform(position, pCar->mCarbody.rotPos, pCar->mCarbody.linPos);
	velocityWld.transform(diagWheelVel, pCar->mCarbody.rotPos, pCar->mCarbody.linVel);
	axisWorld.mult(pCar->mCarbody.rotPos, axis);

	CalcSlip();
}

void CMxWheel::getPosition(Vector3p &linearPos, Matrix3x3 &rotationalPos)
{
	Vector3p diagWheelPos;
	Matrix3x3 rotPosSpin;
	float c, s;

	linearPos.transform(position, pCar->mCarbody.rotPos, pCar->mCarbody.linPos);//position is car coord
	c = float(cos(rotPos) * radius);
	s = float(sin(rotPos) * radius);

	Matrix3x3 finalNoSpin;

	finalNoSpin.mult(  noRotOrient, pSusp->mKnuckleRotPosWC);

	rotPosSpin.m[0] = finalNoSpin.m[0] * width;
    rotPosSpin.m[1] = finalNoSpin.m[1] * c + finalNoSpin.m[2] * s;
    rotPosSpin.m[2] = finalNoSpin.m[2] * c - finalNoSpin.m[1] * s;
    rotPosSpin.m[3] = finalNoSpin.m[3] * width;
    rotPosSpin.m[4] = finalNoSpin.m[4] * c + finalNoSpin.m[5] * s;
    rotPosSpin.m[5] = finalNoSpin.m[5] * c - finalNoSpin.m[4] * s;
    rotPosSpin.m[6] = finalNoSpin.m[6] * width;
    rotPosSpin.m[7] = finalNoSpin.m[7] * c + finalNoSpin.m[8] * s;
	rotPosSpin.m[8] = finalNoSpin.m[8] * c - finalNoSpin.m[7] * s;
	
	
	rotationalPos.mult( pCar->mCarbody.rotPos, rotPosSpin);
	positionWld.transform(position, pCar->mCarbody.rotPos, pCar->mCarbody.linPos);
	rotPosWorld = rotationalPos;//test for graphics
	m_vDown = Vector3p(-noRotOrient.m[3], -noRotOrient.m[4],-noRotOrient.m[5]);
	m_vDown.mult(pSusp->mKnuckleRotPosWC, m_vDown); 
	m_vDown.mult( pCar->mCarbody.rotPos, m_vDown); 
	axis = Vector3p(-noRotOrient.m[0], -noRotOrient.m[3],-noRotOrient.m[6]);

	if(!(m_iWhich&1))//odd
		axis.mult(-1.0f);
	m_vDown.normalise();
	dynamicCamber = m_vDown.dot( m_tireContact.normal);
	dynamicCamber = acos(dynamicCamber);
	dynamicCamber = dynamicCamber - M_PI;	
	if(m_iWhich&1)//odd
		dynamicCamber*=-1.0f;	
	if(m_vDown.x<0)//direction of camber
		dynamicCamber*=-1.0f;
	if(load!=0)
	{
		normDynamicCamber = m_CamberStiffness*sin(dynamicCamber)/(kMuY*load);
		normDynamicCamber = (normDynamicCamber)*180/M_PI;
	}
	else
		normDynamicCamber = (dynamicCamber)*180/M_PI;
}

void CMxWheel::CalcSlip()
{	
}

int CMxWheel::forces( Vector3p &fz)
{
}

//Pacejka's magic formula normalized (stiffnesses multiplied later)
float CMxWheel::normSlipCurve(float k, float C, float E)
{
    float Bx = k / C;
    return float(sin(C * float(atan(Bx - E * (Bx - float(atan(Bx)))))));	
}
float CMxWheel::normLateralSlip(float B, float C, float D, float E,float alpha)
{
	float phi = (1-E)*alpha + (E/B)*atan(B*alpha);
	float theta = C*atan(B*phi);
	return D*sin(theta);
}

void CMxWheel::RKInit()
{
}

void CMxWheel::EulerStep()
{
}
void CMxWheel::RKStep1()
{
}

void CMxWheel::RKStep2()
{
}

void CMxWheel::RKStep3()
{
}

void CMxWheel::RKStep4()
{
}

float CMxWheel::calcWheelAccel()
{
}
*/