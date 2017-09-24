#pragma once
#include "Matrix.h"
#include "MxCarAttrib.h"
#include "MxCar.h"


class CMxCar;



class CMxAero
{
	typedef struct
	{
		float rAngleAttack;
		float rArea;
		Vector3p vPosition;
		Vector3p vForce;

	}tdAeroWing;

public:
	CMxAero( CMxCarAttribute *_pCarAttr, CMxCar *_pCar);
	~CMxAero(void);
	
	void Reload( CMxCarAttribute *_pCarAttr);

	//
	CMxCar *pCar;//to get velocity///....or maybe we should just pass the vector to DoWingForces....
	//properties
	tdAeroWing m_AeroWing;
	float m_Rho;
	float m_Cd;//total drag (body + wing)
	
	
	float m_FrontCl;//body lift
	float m_RearCl;

	float m_AeroDrag;
	float m_FrontalArea;//in sq meters

	Vector3p m_vDragForce;
	Vector3p m_vRearLiftForce;
	Vector3p m_vFrontLiftForce;

	void DoWingForces();
	void DoBodyForces();
};
