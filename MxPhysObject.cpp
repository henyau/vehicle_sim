#include "StdAfx.h"
#include ".\mxphysobject.h"

CMxPhysObject::CMxPhysObject(void)
{
}

CMxPhysObject::CMxPhysObject( dWorldID _odeWorld, dSpaceID _odeSpace,float _mass, float _length, float _height, float _width,
							 float posx, float posy,float posz)
{
	odeBody = dBodyCreate (_odeWorld);
	dMassSetBox(&odeMass,1,_length, _height, _width);
	dMassAdjust(&odeMass,_mass);
	dBodySetMass(odeBody,&odeMass);
	odeGeom = dCreateBox(_odeSpace,_length, _height, _width);

	mass = _mass;
	oneOverMass = 1/mass;

	dBodySetPosition(odeBody, posx, posy, posz);
	dGeomSetBody(odeGeom, odeBody);

	dBodySetAutoDisableFlag(odeBody, 1);

	//dBodySetAutoDisableLinearThreshold(odeBody, 0.05);
	//dBodySetAutoDisableAngularThreshold (odeBody, 0.1);
	dBodySetAutoDisableSteps (odeBody, 15);

	dBodySetFiniteRotationMode (odeBody, 0);



	rLength = _length;
	rWidth = _width;
	rHeight = _height;

//	vInitialPos = Vector3p(posx, posy, posz);

}

CMxPhysObject::~CMxPhysObject(void)
{

	dBodyDestroy(odeBody);
	dGeomDestroy(odeGeom);
}

void CMxPhysObject::setPosition(float x, float y, float z)
{
	dMatrix3 R;
	R[0] = 1;R[1] = 0;R[2] = 0;R[3] = 0;
	R[4] = 0;R[5] = 1;R[6] = 0;R[7] = 0;
	R[8] = 0;R[9] = 0;R[10] = 1;R[11] = 0;


	dBodySetRotation(odeBody, R);
	dBodySetPosition(odeBody, x, y, z);
	dBodySetForce(odeBody, 0, -5,0);
	dBodySetTorque(odeBody, 0, 0, 0);
	dBodySetAngularVel(odeBody, 0,0,0);
	dBodySetLinearVel(odeBody, 0,0,0);
	
}
void CMxPhysObject::Update()
{

	const float *bodyRot= dBodyGetRotation(odeBody); //4x3
	const float *bodyODELinPos= dBodyGetPosition(odeBody); 
	const float *bodyODERotVel= dBodyGetAngularVel(odeBody); 
	const float *bodyODELinVel= dBodyGetLinearVel(odeBody); 
	const float *bodyODEQuat = dBodyGetQuaternion(odeBody);
 	const float *bodyODELinForce = dBodyGetForce(odeBody);

	//rotPos.m update rotational position

	rotPos.m[0]= bodyRot[0];rotPos.m[1]= bodyRot[1];rotPos.m[2]= bodyRot[2];
	rotPos.m[3]= bodyRot[4];rotPos.m[4]= bodyRot[5];rotPos.m[5]= bodyRot[6];
	rotPos.m[6]= bodyRot[8];rotPos.m[7]= bodyRot[9];rotPos.m[8]= bodyRot[10];
	

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

}
