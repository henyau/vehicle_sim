#include "Matrix.h"
#include "ode/ode.h"
#pragma once

//multilink connects 2 rigid bodies together.

class CMxMultiLink
{
public:
	CMxMultiLink(void);
	CMxMultiLink(dBodyID bodyStart, dBodyID bodyEnd);
	~CMxMultiLink(void);

	//RBody *prBody1;
	//RBody *prBody2;

	dBodyID m_BodyStart;//for example chassis
	dBodyID m_BodyEnd;//for example wheel
	

	Vector3p m_vHingeAxis;
	
	Vector3p m_vSteerAxis;

	Vector3p m_vStart0;
	Vector3p m_vStart1;
	Vector3p m_vStart2;
	Vector3p m_vStart3;
	Vector3p m_vStart4;

	Vector3p m_vEnd0;
	Vector3p m_vEnd1;
	Vector3p m_vEnd2;
	Vector3p m_vEnd3;
	Vector3p m_vEnd4;

	dJointID m_jointLinkStart0;
	dJointID m_jointLinkStart1;
	dJointID m_jointLinkStart2;
	dJointID m_jointLinkStart3;
	dJointID m_jointLinkStart4;
	
	/*dJointID m_jointLinkEnd0;
	dJointID m_jointLinkEnd1;
	dJointID m_jointLinkEnd2;
	dJointID m_jointLinkEnd3;
	dJointID m_jointLinkEnd4;
	*/
	dJointGroup m_jointGroup;

	/*dBodyID m_bodyLink0;
	dBodyID m_bodyLink1;
	dBodyID m_bodyLink2;
	dBodyID m_bodyLink3;
	dBodyID m_bodyLink4;*/

	float m_fSpring;
	float m_fDamper;


};
