#pragma once
#include "Matrix.h"
//#include "RigidBody.h"

class CMxTire//combine tire, wheel, suspension into one class
{
public:
	CMxTire(void);
	~CMxTire(void);
	void Init();//(const String& name,const String& mesh, Vector3p position,float mass);
	//(we turn off friction and compute forces on tires ourselves)
	float computeSlipAngle();
	float computeSlipVelocity();
//	const Vector3p& computeLateralForce();
//	const Vector3p& computeLongitudinalForce();	
	//should do this only once on startup (or reload) and put values into lookup table then interpolate
	//Pacejka
	float computeLongTireForce(float m_Fz, float m_rSlipRatio);
	float computeLatTireForce(float m_Fz, float m_rCamber, float m_rSlipAngle);
	float computeMz(float m_Fz, float m_rCamber, float m_rSlipAngle);
	float computeMaxForce(float m_Fz);


	//int m_iWheelIndex;//m_iWhich wheel	


 //   //input physics parameters (tire/brake)
	//float	m_rBrakeForce;//max brake force
	//float m_rFriction;
	////float m_rCornerStiffness;
	//float m_rReactionSpring;
	//float m_rPeakBrakingCoef;
	//float m_rAutoClutchBrakeForce;

 //   float m_rMass;
	//Vector3p m_vInertia;//diags of inertia matrix
	//float m_rRadius;
	//float m_rWidth;
	//float m_rRollingResistanceSpeed;
	//float m_rRollingResistanceBasic;
	//float m_rRollingResistance;
	//float m_rCamber;
	//float m_rToe;	
	//
	//
	////computed state parameters(for Pajecka)
	//float m_rSlipAngle;
	//float m_rSlipRatio;
	//float m_rPosOfEnginebinedSlipLen; //used to compute combined lat/long forces


	////state (tires)
	//float m_rMaxForce;
	//float m_rLatForce;
	//float m_rLongForce;


	//Vector3p m_vWheelAxis;
	//Vector3p m_vSuspAxis;
	//Vector3p m_vSuspPos;
	//float	m_rKp, m_rKd, m_rARB;
	//float m_rRestLength;
	//float m_rUpperLimit;
	//float m_rLowerLimit;	

 //   //state(wheel and suspension)
	//float m_rSuspLinearForce;
	//float m_rWheelRotForce;
	//float m_rSuspLinearPosition,  m_rSuspLinearVelocity;
	//float m_rWheelRotPosition,  m_rWheelRotVelocity;	

};

