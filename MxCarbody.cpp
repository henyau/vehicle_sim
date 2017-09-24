#include "StdAfx.h"
#include "mxcarbody.h"



CMxCarbody::CMxCarbody(void)
{
	

}

CMxCarbody::~CMxCarbody(void)
{
}

void CMxCarbody::Load()
{
	odeBody;// = dBodyCreate(;
	odeMass;
  	odeGeom;
}

//since using ODE for the actual rigid body copy ODE data into our data.  later, just rewrite the Vector/Mat 
//functions to use ODE data types
void CMxCarbody::Update()
{
	const float *bodyRot= dBodyGetRotation(odeBody); //4x3
	const float *bodyODELinPos= dBodyGetPosition(odeBody); 
	const float *bodyODERotVel= dBodyGetAngularVel(odeBody); 
	const float *bodyODELinVel= dBodyGetLinearVel(odeBody); 
	const float *bodyODEQuat = dBodyGetQuaternion(odeBody);
 	const float *bodyODELinForce = dBodyGetForce(odeBody);

	//rotPos.m update rotational position
/*	rotPos.m[0]= bodyRot[0];rotPos.m[1]= bodyRot[4];rotPos.m[2]= bodyRot[8];
	rotPos.m[3]= bodyRot[1];rotPos.m[4]= bodyRot[5];rotPos.m[5]= bodyRot[9];
	rotPos.m[6]= bodyRot[2];rotPos.m[7]= bodyRot[6];rotPos.m[8]= bodyRot[10];
*/
	rotPos.m[0]= bodyRot[0];rotPos.m[1]= bodyRot[1];rotPos.m[2]= bodyRot[2];
	rotPos.m[3]= bodyRot[4];rotPos.m[4]= bodyRot[5];rotPos.m[5]= bodyRot[6];
	rotPos.m[6]= bodyRot[8];rotPos.m[7]= bodyRot[9];rotPos.m[8]= bodyRot[10];
	
	m_vDown.x = -rotPos.m[3];
	m_vDown.y = -rotPos.m[4];
	m_vDown.z = -rotPos.m[5];
	//update linear position
    linPos.x = bodyODELinPos[0];
	linPos.y = bodyODELinPos[1];
	linPos.z = bodyODELinPos[2];

	//rotational velocity
	rotVel.x = bodyODERotVel[0];
	rotVel.y = bodyODERotVel[1];
	rotVel.z = bodyODERotVel[2];

	linVel.x = bodyODELinVel[0];
	linVel.y = bodyODELinVel[1];
	linVel.z = bodyODELinVel[2];

	qRotPos.w = bodyODEQuat[0];
	qRotPos.x = bodyODEQuat[1]; 
	qRotPos.y = bodyODEQuat[2];
	qRotPos.z = bodyODEQuat[3];


	linForce.x = bodyODELinForce[0];
	linForce.y = bodyODELinForce[1];
	linForce.z = bodyODELinForce[2];

	 //do balance forces

	// const dReal *torque = dBodyGetTorque(odeBody);
	//torque[0];
	//torque[1];
	//torque[2];
	// dBodySetTorque(odeBody, torque[0], torque[1], 0/*mCarbody.linForce.x*//*-0.01*torque[2]*/);
	// const dReal *quat =  dBodyGetQuaternion (odeBody);

	// float bodyRollAngle;
	//if(fabs(linForce.x)>0.1)
	//	bodyRollAngle = -atan(linForce.y/ linForce.x) ;
	//else 
	//	bodyRollAngle = 0;
	//dQuaternion quaternion;
	//dQFromAxisAndAngle(quaternion, 0, 0, 1, bodyRollAngle);
	//dGeomSetQuaternion(odeGeom, quaternion);

}

void CMxCarbody::setStepSize(float h)
{

}

