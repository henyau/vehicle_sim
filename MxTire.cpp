#include "Stdafx.h"
#include ".\mxtire.h"
#define M_PI 3.1415926535897932384626433832795
///Note Used.  Using normalized pacejka instead since coeffs are easier to tweak


CMxTire::CMxTire(void)
{
}

CMxTire::~CMxTire(void)
{
}

void CMxTire::Init()//(const String& name,const String& mesh, Vector3p position,float mass)
{
	//m_vSuspPos = position;
	//m_name = name;
	//m_Entity = World::getSingleton().getSceneManager()->createEntity(name + "_Entity",mesh);
	//m_Entity->setCastShadows(true);
 //   m_rRadius = MxMeshInformer::getRadius(m_Entity->getMesh());//change.

	//m_Node = World::getSingleton().getSceneManager()->getRootSceneNode()->createChildSceneNode(name + "_Node");
	//m_Node->setPosition(position);
	////m_body = new Body(name + "_Body");
	//m_Node->attachObject(m_Entity);
	//m_Node->attachObject(m_body);

	//m_body->setMass(SphereMass(mass,m_rRadius));//change to disk or own tire class
	//m_geometry = new SphereGeometry(m_rRadius, World::getSingleton().getDefaultSpace());
	//m_geometry->setBody(m_body);
}

float CMxTire::computeLongTireForce(float rFz, float rSlipRatio)
{
//	from Genta's book, from a supposed 312
	rFz *= 0.001f;
	float b0= 	1.65f;//  	none  	   	 
	float b1 =	0.0f; 	//1/MegaNewton 	  	 
	float b2 =	1688.0f;// 	1/K 	  	 
	float b3 =	0.0f; 	//1/MegaNewton 	  	 
	float b4 =	229.0f; //	1/K 	  	 
	float b5 =	0.0f; 	//1/KN 	  	 
	float b6 =	0.0f; 	//1/(KN)^2 	  	 
	float b7 =	0.0f; //	1/KN 	  	 
	float b8 =	-10.0f;// 	none 	  	 
	float b9 =	0.0f;	//1/KN 	  	 
	float b10 =	0.0f; //	none 	  	 
	float b11 =	0.0f ;//	N/kN 	Load influence on vertical shift 	NYI in Racer; values between -106..142
	float b12 =	0.0f;// 	N
	/*float B;
	float C = b0;
	float D = (b1*m_Fz+b2)*m_Fz;
	if((C>-0.000001&&C<0.00001)|| (D>-0.000001&&D<0.00001))
		 B = 99999.0f;
	else
		 B = ((b3*m_Fz*m_Fz+b4*m_Fz)*expf(-b5*m_Fz))/(C*D);
	float E = b6*m_Fz*m_Fz+b7*m_Fz+b8;
	float Sh = b9*m_Fz+b10;
	float Sv = 0.0f;
	float m_Fx = D*sinf(C*atan(B*(1-E)*(m_rSlipRatio+Sh)+E*atan(B*(m_rSlipRatio+Sh))))+Sv;*/
	

	float muP = b1*rFz + b2;
	float D = muP*rFz; //N, normal frictional force.
	float E = b6*rFz*rFz + b7*rFz + b8;
	float B = (b3*rFz + b4)*exp(-b5*rFz)/(b0 * muP);
	float S = rSlipRatio + b9 * rFz +b10;
	//S = 10.0f;
	 //float sanity = sin (b0 * atan (S*B + E*(atan(S*B) -S*B)));
	float rFx = D * sin (b0 * atan (S*B + E*(atan(S*B) -S*B)));


	return rFx;
}

float CMxTire::computeLatTireForce(float rFz, float rCamber, float rSlipAngle)
{//BCD = Cornering Stiffness
 
	rFz *= 0.001f;
  	float  a0=1.799f;
 	float a1=-38.0f;
	float a2=1488.0f;
 	float   a3=2640.0f;
	float a4=8.026f;
	float a5=0.014f;
	float a6=-0.25890f; 
	float a7= 1.0f;
	float a8=-0.03f;
	float a9=-0.00131f;
	float a10=-0.15224f;
	float a111=-8.5f;
  	float   a112=-0.29f;
	float a12=-6.0f;
	float a13=-2.4f;
/////////
	//float B,C,D,E;
	float rFy;
	//float Sh,Sv;
	//float uP;
	  
//  // Calculate derived coefficients
//  C=a0;
//  uP=a1*m_Fz+a2;
//  D=uP*m_Fz;
//  E=a6*m_Fz+a7;
//  
//  // Avoid div by 0
//  if((C>-0.000001&&C<0.00001)|| (D>-0.000001&&D<0.00001))
//  {
//    B=99999.0f;
//  } 
//  else
//  {
//    if(a4>-0.00001&&a4<0.000001)
//		B=99999.0f;
//	else
//		B=(a3*sinf(2*atanf(m_Fz/a4))*(1-a5*fabs(m_rCamber)))/(C*D);
//    
//  }
//  //Sh=a8*m_rCamber+a9*m_Fz+a10;//camber should be in degrees
//  Sv=(a111*m_Fz+a112)*m_rCamber*m_Fz+a12*m_Fz+a13;// ply steer and conicity
//  
//  // Calculate result force
//  //m_Fy=D*sinf(C*atanf(B*(1.0f-E)*(m_rSlipAngle+Sh)+E*atanf(B*(m_rSlipAngle+Sh))))+Sv;
//
//  float SB = (m_rSlipAngle+a8*m_rCamber+a9*m_Fz+a10)*B;
//  m_Fy = D*sinf(a0 * atan(SB+E*(atan(SB)-SB) )) +Sv;
//float temp = sinf(a0 * atan(SB+E*(atan(SB)-SB) ));

	//rCamber*= (float)(180.0f/M_PI);
	//rSlipAngle*= (float)(180.0f/M_PI);

	float muYP = a1*rFz +a2; //peak lateral coefficient 1/Kilos
	float D = muYP *  rFz;//normal * friction
	float B;
	float D1 = a0*muYP*rFz;
	if(fabs(D1)>0.00001f)
		B =( a3 * sin(2*atan (rFz/a4) )*(1-a5*fabs(rCamber)))/(a0 * muYP * rFz);
	else
		B = 99999.0f;

	float S = (rSlipAngle) +(a8 * rCamber) + a9*rFz + a10;
	float E = a6*rFz + a7;
	float Sv = ((a111*rFz +a112)*rCamber +a12)*rFz + a13; 
	rFy = D * sin(a0 * atan (S*B + E*(atan(S*B) -S*B)))+Sv;
 	return (-rFy);
}

float CMxTire::computeMz(float m_Fz, float m_rCamber, float m_rSlipAngle)
// Calculates aligning moment
{
//	m_Fz*=0.001;
	float c0=2.068f;
	float c1=-6.49f;
	float c2=-21.85f;
	float c3=0.416f;
	float c4=-21.31f;
	float c5=0.02942f;
	float   c6=0.0f;
	float c7=-1.197f;
	float c8=5.228f;
	float c9=-14.84f;
	float c10=0.0f;
	float c11=0.0f;
	float c12=-0.003736f;
	float c13=0.03891f;
	float c14=0.0f;
	float c15=0.0f;
	float c16=0.639f;
	float c17=1.693f;

	float  m_Mz;
	float  B,C,D,E,Sh,Sv;
//	float  uP;
	float  m_FzSquared;
  
	//m_rCamber*= (float)(180.0f/M_PI) *0;
	//m_rSlipAngle*= (float)(180.0f/M_PI);

	// Calculate derived coefficients
	m_FzSquared=m_Fz*m_Fz;
	C=c0;
	D=c1*m_FzSquared+c2*m_Fz;
	E=(c7*m_FzSquared+c8*m_Fz+c9)*(1-c10*fabs(m_rCamber));
	if((C>-0.00001f&&C<0.00001f)||(D>-0.00001f&&D<0.00001f))
	{
		B=99999.0f;
	} 
	else
	{
		B=((c3*m_FzSquared+c4*m_Fz)*(1.0f-c6*fabs(m_rCamber))*expf(-c5*m_Fz))/(C*D);
	}
	Sh=c11*m_rCamber+c12*m_Fz+c13;
	Sv=(c14*m_FzSquared+c15*m_Fz)*m_rCamber+c16*m_Fz+c17;
	  
	m_Mz=D*sin(C*atanf(B*(1.0f-E)*(m_rSlipAngle+Sh)+
				E*atan(B*(m_rSlipAngle+Sh))))+Sv;
	return m_Mz;
}



float CMxTire::computeMaxForce(float m_Fz)
// Calculates maximum force that the tire can produce
// If the longitudinal and lateral force combined exceed this,
// a violation of the friction circle (max total tire force) is broken.
// In that case, reduce the lateral force, since the longitudinal force
// is more prominent (this simulates a locked braking wheel, m_iWhich
// generates no lateral force anymore but does maximum longitudinal force).
{
	float uP;
	float b1 =	0.0f; 	//1/MegaNewton 	
	float b2 =	1688.0f;// 	1/K 	  	

	// Calculate derived coefficients
	uP=b1*m_Fz+b2;

  return uP*m_Fz;
}