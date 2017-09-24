#include "StdAfx.h"
#include "MxSusp.h"
//simple linear spring. soon to be non-linear.  then to be moveable suspension geometry

CMxSusp::CMxSusp(CMxCarAttribute *_pCarAttr, CMxCar *_Car,int _which)
{
		pCar = _Car;
		m_iWhich = _which;
		Initialize();
		Reload(_pCarAttr);
}

CMxSusp::~CMxSusp(void)
{
}



void CMxSusp::Initialize()
{
	//reset state variables
	linearAcceleration = 0.0f;
	linearPosition = 0.0f;
    linearVelocity = 0.0f;
	linearForce = 0.0f;	
}	
void CMxSusp::Reload(CMxCarAttribute *_pCarAttr)
{
		//extern CMxCarAttribute m_CarAttr;
		ks = _pCarAttr->m_Suspension[m_iWhich/2].rKp;
		bumpRate = _pCarAttr->m_Suspension[m_iWhich/2].rKdBounce;
		reboundRate = _pCarAttr->m_Suspension[m_iWhich/2].rKdRebound;
		arbKs = _pCarAttr->m_Suspension[m_iWhich/2].rARB;//anti roll bar "spring"

		//origin of suspension/axis of travel
		// in body coords
		if(m_iWhich==0)//front
		{
			position.x = -(_pCarAttr->m_Body.rFrontWheelBase*0.5f);
			position.y = (_pCarAttr->m_Body.rFrontSuspHeight);
			position.z = (_pCarAttr->m_Body.rDistToFront);
			
			axis.x = -tan((_pCarAttr->m_Suspension[0].fOffx*M_PI )/180.0f);
			axis.z = tan((_pCarAttr->m_Suspension[0].fOffz*M_PI)/180.0f );
			axis.y = 1.0f;
			axis.normalise();			
		}
		else if(m_iWhich ==1)
		{
			position.x = (_pCarAttr->m_Body.rFrontWheelBase*0.5f);
			position.y = (_pCarAttr->m_Body.rFrontSuspHeight);
			position.z = (_pCarAttr->m_Body.rDistToFront);

			axis.x = tan((_pCarAttr->m_Suspension[0].fOffx*M_PI )/180);
			axis.z = tan((_pCarAttr->m_Suspension[0].fOffz*M_PI)/180.0f );
			axis.y = 1.0f;
			axis.normalise();

		}
		else if(m_iWhich ==2)
		{
			position.x = -(_pCarAttr->m_Body.rRearWheelBase*0.5f);
			position.y = (_pCarAttr->m_Body.rRearSuspHeight);
			position.z = -(_pCarAttr->m_Body.rDistToRear);

			axis.x = -tan((_pCarAttr->m_Suspension[1].fOffx*M_PI)/180.0f );
			axis.z = tan((_pCarAttr->m_Suspension[1].fOffz*M_PI)/180.0f );
			axis.y = 1.0f;
			axis.normalise();

		}
		else if(m_iWhich ==3)
		{
			position.x = (_pCarAttr->m_Body.rRearWheelBase*0.5f);
			position.y = (_pCarAttr->m_Body.rRearSuspHeight);
 			position.z = -(_pCarAttr->m_Body.rDistToRear);

			axis.x = tan((_pCarAttr->m_Suspension[1].fOffx*M_PI)/180.0f );
			axis.z = tan((_pCarAttr->m_Suspension[1].fOffz*M_PI)/180.0f );
			axis.y = 1.0f;
			axis.normalise();

		}
	
		//front
		if(m_iWhich <2)
		{
			restLength = _pCarAttr->m_Suspension[0].rLength;
			//linearPosition = restLength;
			minLength = _pCarAttr->m_Suspension[0].rLoLimit;
			maxLength = _pCarAttr->m_Suspension[0].rUpLimit; //note...these numbers are kind of backwards. 
																						//they denote the distance from the top of the suspension.

			iType = _pCarAttr->m_Suspension[0].iType;

		}
		//rear
		else
		{
			restLength = _pCarAttr->m_Suspension[1].rLength;
			minLength = _pCarAttr->m_Suspension[1].rLoLimit;
			maxLength = _pCarAttr->m_Suspension[1].rUpLimit;
			iType = _pCarAttr->m_Suspension[1].iType;
		
		}



		linearForce = 0.0f;
		linearPosition = 0.0f;
		linearVelocity = 0.0f;
		// now calculate suspension and wheels geometry in diagonalised coords
		//need to know the carbody, so do in mxcar
		//axis.mult(mCar->mCarbody.diagRotPos, axis);
		//suspPos0.transform(position, mCar->mCarbody.diagRotPos, mCar->mCarbody.diagLinPos);

   	/*	suspAxis.mult(mainBody.diagRotPos, defSuspAxis[i]);
        suspPos0.transform(position, carBody.diagRotPos, mainBody.diagLinPos);
    
		frontWheelAxis0[0].mult(mainBody.diagRotPos, defWheelAxis[0]);
		frontWheelAxis0[1].mult(mainBody.diagRotPos, defWheelAxis[1]);
		wheels[2].axis.mult(mainBody.diagRotPos, defWheelAxis[2]);
		wheels[3].axis.mult(mainBody.diagRotPos, defWheelAxis[3]);
*/

		///test STATIC ROLL CENTER

		//test a macpherson strut
		

		if(m_iWhich & 1)
		{
			vMountTopCC.add(position, Vector3p(_pCarAttr->m_Suspension[m_iWhich/2].fMountOffsetUpper[0], 
				_pCarAttr->m_Suspension[m_iWhich/2].fMountOffsetUpper[1], 0.0));
			vMountBottomCC.add(position, Vector3p(_pCarAttr->m_Suspension[m_iWhich/2].fMountOffsetLower[0],
				_pCarAttr->m_Suspension[m_iWhich/2].fMountOffsetLower[1], 0.0));
		}
		else
		{
			vMountTopCC.add(position, Vector3p(-_pCarAttr->m_Suspension[m_iWhich/2].fMountOffsetUpper[0], 
				_pCarAttr->m_Suspension[m_iWhich/2].fMountOffsetUpper[1], 0.0));
			vMountBottomCC.add(position, Vector3p(-_pCarAttr->m_Suspension[m_iWhich/2].fMountOffsetLower[0],
				_pCarAttr->m_Suspension[m_iWhich/2].fMountOffsetLower[1], 0.0));
		}


		Vector3p vTempMount = vMountTopCC;
		vTempMount.sub(vMountBottomCC);
		fMountLength =vTempMount.mod();


		fCasterAngle = _pCarAttr->m_Suspension[m_iWhich/2].fCasterAngle * M_PI /180.0f;
		
		Matrix3x3 mCasterRot;
		mCasterRot.xAxisRotation(fCasterAngle);
		Vector3p vMountTopAtOrig = Vector3p(0.0f,fMountLength,0.0f);
				
		vMountTopAtOrig.mult(mCasterRot, vMountTopAtOrig);
		vMountTopCC.add(vMountTopAtOrig);
		vMountTopCC.y = vMountTopAtOrig.y+vMountBottomCC.y;

		fLengthBottom = _pCarAttr->m_Suspension[m_iWhich/2].fLowerControlArmLength;
		fLengthTop = _pCarAttr->m_Suspension[m_iWhich/2].fUpperControlArmLength;

		fRCYPos =  _pCarAttr->m_Suspension[m_iWhich/2].fRollCenterY;
		fRCZPos = _pCarAttr->m_Suspension[m_iWhich/2].fRollCenterZ;
		
		fInstallationRatio = _pCarAttr->m_Suspension[m_iWhich/2].fInstallRatio;
		Update();



		fMountingAngle = vTempMount.dot(Vector3p(0.0f,1.0f,0.0f));
		fMountingAngle = acos(fMountingAngle);

		fKnuckleOffset[0] = _pCarAttr->m_Suspension[m_iWhich/2].fKnuckleOffset[0];//0.02f;
		fKnuckleOffset[1] = _pCarAttr->m_Suspension[m_iWhich/2].fKnuckleOffset[1];//0.06f;
		fKnuckleOffset[2] = _pCarAttr->m_Suspension[m_iWhich/2].fKnuckleOffset[2];//0.06f;//used in semi-trailing arm only.(so it is the arm to wheel offset in the z-direction
		
		 fSpindleLength = _pCarAttr->m_Suspension[m_iWhich/2].fSpindleLength;

		 fSweepAngle = _pCarAttr->m_Suspension[m_iWhich/2].fSweepAngle*M_PI/180.0f;

		fSTABackYOffset = -0.05f;// temp... later use inclination angle

}

void CMxSusp::Update()
{
	
	//iType = SUSP_TYPE_1DSTRUT;
	if(iType == SUSP_TYPE_MACPHERSON)
	{ //need to add limits to IC in case of parallel.
		//MacPherson
		//update the start position of suspension in WORLD C0ORDS!!!!
		suspPos0.mult(pCar->mCarbody.rotPos, position);
		suspPos0.add(pCar->mCarbody.linPos);
		axisWC.mult(pCar->mCarbody.rotPos, axis);
		    

		vLowerBallJointCC = vMountBottomCC;

		
// 		float sinBeta =  ((linearPosition)-restLength)*fInstallationRatio/fLengthBottom;
		float sinBeta =  (((linearPosition)-restLength)/fInstallationRatio)/fLengthBottom;
		//how beta varies with suspension length
		float beta = asin(sinBeta);
		float cosBeta = cos(beta);

		
		//float alpha3 = acos((fMountLength*fMountLength)+(fLengthBottom*fLengthBottom)-(linearPosition+restLength))/(2*fMountLength*fLengthBottom);
		//float beta = M_PI - alpha3 - fMountingAngle;
		//float cosBeta = cos(beta);
		//float sinBeta = sin(beta);


		if(m_iWhich&1)
			vLowerBallJointCC.x +=fLengthBottom * cosBeta;
		else
			vLowerBallJointCC.x -=fLengthBottom * cosBeta;

		vLowerBallJointCC.y +=fLengthBottom *sinBeta;

		vMountBottomWC.mult(pCar->mCarbody.rotPos, vMountBottomCC);
		vMountBottomWC.add(pCar->mCarbody.linPos);
		
		vMountTopWC.mult(pCar->mCarbody.rotPos, vMountTopCC);
		vMountTopWC.add(pCar->mCarbody.linPos);

		vLowerBallJointWC.mult(pCar->mCarbody.rotPos, vLowerBallJointCC);
		vLowerBallJointWC.add(pCar->mCarbody.linPos);
		

		sinBeta =  ((linearPosition*fInstallationRatio))/fLengthBottom;
		beta -=M_PI/2;

		if(!(m_iWhich&1))
		{
			//beta = -beta;
			vUpperBallJointCC.mult(-1);
		}	
		
		float fStrutLength;
		Vector3p vStrutCC = vMountTopCC;
		vStrutCC.sub(vLowerBallJointCC);
		fStrutLength = vStrutCC.mod();
		vInstantaneousAxisCC = vStrutCC;//for MacPherson Strut, the axis is same of strut.(for my case anyways ...
		/*
		float knuckInAngle = acos(((fStrutLength*fStrutLength)+(fLengthBottom*fLengthBottom)-(fMountLength*fMountLength))/(2*fStrutLength*fMountLength));
		float alpha = ( (beta + knuckInAngle)- M_PI*0.75f) ;
	*/

		float alpha =acos( vStrutCC.dot(Vector3p(0,1.0f,0))) - (0.75*M_PI);// knuckle is oriented with the strut
		if(!(m_iWhich&1))
			alpha = -alpha;

		float sinAlpha = sin(alpha);
		float cosAlpha = cos(alpha);

		//orientation of the knuckle. will by multiplied by noRotOrient to get final orientation
		mKnuckleRotPosWC = Matrix3x3(
			-sinAlpha,	cosAlpha,		0,
			cosAlpha,	sinAlpha	,0,		
			0,		0	,	1 );
	
		vKnucklePosCC = vLowerBallJointCC;
		vKnucklePosCC.y += fKnuckleOffset[1];
		if(m_iWhich&1)
			vKnucklePosCC.x += fKnuckleOffset[0];
		else
			vKnucklePosCC.x -= fKnuckleOffset[0];

		vKnucklePosCC.z += fKnuckleOffset[2];
		
		vKnucklePosWC.transform(vKnucklePosCC, pCar->mCarbody.rotPos, pCar->mCarbody.linPos);//i need to have some function to do these transformations...


	///calc ICs .. do in CC
		/// ADD parallel checks to ICs
	
	float magSideIC;
	Vector3p V1, V2, V3;
	
	V1 = Vector3p (0, -vStrutCC.z, vStrutCC.y);//side view 90 to strut axis
	V1.normalise();
	V3 = Vector3p (0, 0, -1.0f); //for now just keep LCA parallel to base;

	V2.cross(V3,V1);
    vStrutCC.cross(V1,vStrutCC );
	magSideIC = vStrutCC.mod()/V2.mod();
	if(fCasterAngle<0)
		magSideIC *=-1;

	vSideViewIC_CC.mac(vMountTopCC, V1, magSideIC);
	vSideViewIC_WC.mult(pCar->mCarbody.rotPos, vSideViewIC_CC);
	vSideViewIC_WC.add(pCar->mCarbody.linPos);

    
	
	vStrutCC = vMountTopCC;
	vStrutCC.sub(vLowerBallJointCC);

	
	V1 = Vector3p (-vStrutCC.y, vStrutCC.x, 0);
	V1.normalise();
	
	V3.sub( vLowerBallJointCC,vMountBottomCC);
	V3.z = 0;
	V3.normalise();	


	V2.cross(V3,V1);
    vStrutCC.cross(V1,vStrutCC );
	magSideIC = vStrutCC.mod()/V2.mod();
	if(!(m_iWhich&1))
		magSideIC *=-1;
	
	vFrontViewIC_CC.mac(vMountTopCC, V1, magSideIC);
	vFrontViewIC_WC.mult(pCar->mCarbody.rotPos, vFrontViewIC_CC);
	vFrontViewIC_WC.add(pCar->mCarbody.linPos);

	vInstantRollAxis.sub(vFrontViewIC_WC, vSideViewIC_WC);//instance axis that Wheel travels.
	}
	
	if(iType==SUSP_TYPE_SEMITRAIL)
	{//terminalogy is all messed up.... but all we're doing is rotating the knuckle about some fixed axis
		//Axis intersection with axel plane is FVIC, Axis intersect with Wheel plane is SVIC
	
		//Use the vMountBottom as the point on instantaneous axis where (vMountBOttom-knuckle) is perpendicular to that axis
		//camber = asin(sin (sweep) sin( gain))
		//toe = sweep - atan(sin (sweep) * sin(gain) /sin(sweep) )

		suspPos0.mult(pCar->mCarbody.rotPos, position);
		suspPos0.add(pCar->mCarbody.linPos);
		axisWC.mult(pCar->mCarbody.rotPos, axis);
	

		//divide into two rotations. Sweep rotate, Swing arm rotate
		float sinSweepAngle;
		float cosSweepAngle;
		if((m_iWhich&1))
		{
			sinSweepAngle = sin(fSweepAngle); // rotation about y-axis
			cosSweepAngle = cos(fSweepAngle);
		}
		else
		{
			sinSweepAngle = sin(-fSweepAngle); // rotation about y-axis
			cosSweepAngle = cos(-fSweepAngle);
		}
		

		vLowerBallJointCC = vMountBottomCC;
		
		Matrix3x3 swingArmRotate;
		//Vector3p instantaniousAxis2;
		//instantaniousAxis2 = vMountBottomCC;// this can also be the one used in the IC calcs

		float sinBeta =  (((linearPosition)-restLength)/fInstallationRatio)/fLengthBottom;
		//From  RCVD. if pivot is at P, and the spring acts at a distance 'a' along the pivot arm and the wheel center is at b,
		// when Wheel moves deltaX, the spring compresses deltaY. IR = deltaY/deltaX
		// IR changes with ride travel (but I don't take this into account as stiffly sprung springs can be approximated with IR^2

		//working backwards from the linear spring position, we find the wheel position by the geometry.
		//

		float beta = asin(sinBeta);
		if(!(m_iWhich&1))
			beta *=-1.0f;
		float cosBeta = cos(beta); // rotation about z-axis
//		sinBeta = sin(beta);// can do a easy optimize later


		//swingArmRotate.arbAxisRotation( Vector3p(cosSweepAngle , 0 , sinSweepAngle), beta);
		swingArmRotate.arbAxisRotation( Vector3p(-sinSweepAngle , fSTABackYOffset , cosSweepAngle), beta);
//		swingArmRotate.mult(Matrix3x3(cosSweepAngle, 0,-sinSweepAngle,
//			0,1,0,sinSweepAngle,0,cosSweepAngle	), swingArmRotate);

		//rotation matrix for knuck center
		//Matrix3x3 rotToWheelPos(cosBeta*cosSweepAngle, sinBeta,  sinSweepAngle*cosBeta,
		//										 -sinBeta*cosSweepAngle,cosBeta,  sinSweepAngle*(sinBeta),
		//										 -sinSweepAngle, 0 , cosSweepAngle);
		//Vector3p LBJLocal(fLengthBottom, 0.0f, 0.0f);// relative to lower mounting point.

		//if(!(m_iWhich&1))
		//	LBJLocal.x *=-1.0f;

		//LBJLocal.mult(rotToWheelPos, LBJLocal);

		Vector3p LBJLocal(fLengthBottom*cosSweepAngle, 0.0f,fLengthBottom*sinSweepAngle);// relative to lower mounting point.
		//Vector3p vPerpRodRotate(fLengthBottom, 0.0f, 0.0f);//for IC calcs
		if(!(m_iWhich&1))
		{
			LBJLocal.mult(-1.0f);
		//	vPerpRodRotate.mult(-1.0f);
		}
		

		LBJLocal.z += fKnuckleOffset[2];//for wheel position/orientation calcs
		LBJLocal.y += fKnuckleOffset[1];

		if(m_iWhich&1)
			LBJLocal.x += fKnuckleOffset[0];
		else
			LBJLocal.x -= fKnuckleOffset[0];

		LBJLocal.mult(swingArmRotate, LBJLocal);
		//vPerpRodRotate.mult(swingArmRotate, vPerpRodRotate);

		/////slightly optimized
		//Vector3p LBJLocal = Vector3p(cosBeta*cosSweepAngle*fLengthBottom,-sinBeta*cosSweepAngle*fLengthBottom,-sinSweepAngle*fLengthBottom);
		//if(!(m_iWhich&1))
		//	LBJLocal.mult(-1.0f);
		/////
		vLowerBallJointCC.add(LBJLocal);


		vMountBottomWC.mult(pCar->mCarbody.rotPos, vMountBottomCC);
		vMountBottomWC.add(pCar->mCarbody.linPos);		
		vMountTopWC.mult(pCar->mCarbody.rotPos, vMountTopCC);
		vMountTopWC.add(pCar->mCarbody.linPos);
		vLowerBallJointWC.mult(pCar->mCarbody.rotPos, vLowerBallJointCC);
		vLowerBallJointWC.add(pCar->mCarbody.linPos);


		float fStrutLength;
		Vector3p vStrutCC = vMountTopCC;
		vStrutCC.sub(vLowerBallJointCC);
		fStrutLength = vStrutCC.mod();
		vInstantaneousAxisCC = vStrutCC; //this is the axis of travel not rotation.  force is applied in this direction( to suspension spring)
		//----------
	
		/*if(m_iWhich&1)
		mKnuckleRotPosWC = Matrix3x3(
												cosBeta*cosSweepAngle, -sinBeta,  -sinSweepAngle*cosBeta,
												 -sinBeta*cosSweepAngle,-cosBeta,  -sinSweepAngle*(sinBeta),
												 sinSweepAngle, 0 ,cosSweepAngle);

		else
		mKnuckleRotPosWC = Matrix3x3(
												 -cosBeta*cosSweepAngle, sinBeta,  sinSweepAngle*cosBeta,
												 sinBeta*cosSweepAngle,cosBeta,  sinSweepAngle*(sinBeta),
												 sinSweepAngle, 0 , cosSweepAngle);*/

		if(m_iWhich&1)
			mKnuckleRotPosWC =  Matrix3x3(swingArmRotate.m[0], -swingArmRotate.m[1], swingArmRotate.m[2],
																	swingArmRotate.m[3],-swingArmRotate.m[4],swingArmRotate.m[5],
																	swingArmRotate.m[6],-swingArmRotate.m[7],swingArmRotate.m[8]		);
		else
			mKnuckleRotPosWC =  Matrix3x3(-swingArmRotate.m[0], swingArmRotate.m[1], swingArmRotate.m[2],
																-swingArmRotate.m[3],swingArmRotate.m[4],swingArmRotate.m[5],
																-swingArmRotate.m[6],swingArmRotate.m[7],swingArmRotate.m[8]		);
////////------------


		/* // made big mistake. rotation beta should be about an arbitrary vector defined by sweep angle not z-axis
 		float sinBeta =  ((linearPosition)-restLength)*fInstallationRatio/fLengthBottom;
		float beta = asin(sinBeta);
		if((m_iWhich&1))
			beta *=-1.0f;
		float cosBeta = cos(beta); // rotation about z-axis
		sinBeta = sin(beta);// can do a easy optimize later
		
		float sinSweepAngle;
		float cosSweepAngle;
		if((m_iWhich&1))
		{
			sinSweepAngle = sin(fSweepAngle); // rotation about y-axis
			cosSweepAngle = cos(fSweepAngle);
		}
		else
		{
			sinSweepAngle = sin(-fSweepAngle); // rotation about y-axis
			cosSweepAngle = cos(-fSweepAngle);
		}
		vLowerBallJointCC = vMountBottomCC;

		//rotation matrix for knuck center
		//Matrix3x3 rotToWheelPos(cosBeta*cosSweepAngle, sinBeta,  sinSweepAngle*cosBeta,
		//										 -sinBeta*cosSweepAngle,cosBeta,  sinSweepAngle*(sinBeta),
		//										 -sinSweepAngle, 0 , cosSweepAngle);
		//Vector3p LBJLocal(fLengthBottom, 0.0f, 0.0f);// relative to lower mounting point.

		//if(!(m_iWhich&1))
		//	LBJLocal.x *=-1.0f;

		//LBJLocal.mult(rotToWheelPos, LBJLocal);

		///slightly optimized
		Vector3p LBJLocal = Vector3p(cosBeta*cosSweepAngle*fLengthBottom,-sinBeta*cosSweepAngle*fLengthBottom,-sinSweepAngle*fLengthBottom);
		if(!(m_iWhich&1))
			LBJLocal.mult(-1.0f);
		///

		vLowerBallJointCC.add(LBJLocal);

		vMountBottomWC.mult(pCar->mCarbody.rotPos, vMountBottomCC);
		vMountBottomWC.add(pCar->mCarbody.linPos);
		
		vMountTopWC.mult(pCar->mCarbody.rotPos, vMountTopCC);
		vMountTopWC.add(pCar->mCarbody.linPos);

		vLowerBallJointWC.mult(pCar->mCarbody.rotPos, vLowerBallJointCC);
		vLowerBallJointWC.add(pCar->mCarbody.linPos);


		float fStrutLength;
		Vector3p vStrutCC = vMountTopCC;
		vStrutCC.sub(vLowerBallJointCC);
		fStrutLength = vStrutCC.mod();
		vInstantaneousAxisCC = vStrutCC; //this is the axis of travel not rotation.  force is applied in this direction( to suspension spring)
		//----------
	
		if(m_iWhich&1)
		mKnuckleRotPosWC = Matrix3x3(
												cosBeta*cosSweepAngle, -sinBeta,  -sinSweepAngle*cosBeta,
												 -sinBeta*cosSweepAngle,-cosBeta,  -sinSweepAngle*(sinBeta),
												 sinSweepAngle, 0 ,cosSweepAngle);

		else
		mKnuckleRotPosWC = Matrix3x3(
												 -cosBeta*cosSweepAngle, sinBeta,  sinSweepAngle*cosBeta,
												 sinBeta*cosSweepAngle,cosBeta,  sinSweepAngle*(sinBeta),
												 sinSweepAngle, 0 , cosSweepAngle);

		*/

	/* // don't remember what I was trying to do... starting over...*_*
		if(m_iWhich&1)
			vLowerBallJointCC.x +=fLengthBottom * cosBeta*0.5f;
		else
			vLowerBallJointCC.x -=fLengthBottom * cosBeta*0.5f;
		vLowerBallJointCC.y +=fLengthBottom *sinBeta*0.5f;

		//rotate by sweep angle
		if(m_iWhich&1)
			vLowerBallJointCC.x+=fLengthBottom* cosSweepAngle*0.5f;
		else
			vLowerBallJointCC.x -= fLengthBottom*cosSweepAngle*0.5f;
		vLowerBallJointCC.z -=fLengthBottom*sinSweepAngle*0.5f;


		vMountBottomWC.mult(pCar->mCarbody.rotPos, vMountBottomCC);
		vMountBottomWC.add(pCar->mCarbody.linPos);
		
		vMountTopWC.mult(pCar->mCarbody.rotPos, vMountTopCC);
		vMountTopWC.add(pCar->mCarbody.linPos);

		vLowerBallJointWC.mult(pCar->mCarbody.rotPos, vLowerBallJointCC);
		vLowerBallJointWC.add(pCar->mCarbody.linPos);

		sinBeta =  ((linearPosition*fInstallationRatio))/fLengthBottom;
		beta -=M_PI/2;

		
		float fStrutLength;
		Vector3p vStrutCC = vMountTopCC;
		vStrutCC.sub(vLowerBallJointCC);
		fStrutLength = vStrutCC.mod();
		vInstantaneousAxisCC = vStrutCC; //this is the axis of travel not rotation.  force is applied in this direction
	*/
	
		//float camberAngle = asin(sinBeta * sinSweepAngle) + M_PI/2.0f;
		//float toeAngle = -atan((sinBeta * sinSweepAngle)/cos(beta));
		//
		//if((m_iWhich&1))
		//{
		//	camberAngle *=-1;
		//	toeAngle -= M_PI/4.0f;	
		//}
		//else
		//	toeAngle += M_PI/4.0f;	

		//orientation of the knuckle. will by multiplied by noRotOrient to get final orientation
		/* // not sure what I was doing here either! &_&
		float camberAngle = asin(sinBeta * sinSweepAngle) + M_PI/2.0f;
		float toeAngle = -atan((sinBeta * sinSweepAngle)/cos(beta));
		
		if((m_iWhich&1))
		{
			camberAngle *=-1;
			toeAngle -= M_PI/4.0f;	
		}
		else
			toeAngle += M_PI/4.0f;	

		//orientation of the knuckle. will by multiplied by noRotOrient to get final orientation
		mKnuckleRotPosWC = Matrix3x3(
			-sin(camberAngle),	cos(camberAngle),		0,
			cos(camberAngle),	sin(camberAngle)	,0,		
			0,		0	,	1 );

        		////toe gain
		//Matrix3x3 mToeGain 
		mKnuckleRotPosWC	=  Matrix3x3(
			-sin(toeAngle),0,			cos(toeAngle),
			0,	1	,0,	
			cos(toeAngle),		0	,	sin(toeAngle) );

		//Matrix3x3 toeGain, camberGain;

		//toeGain.yAxisRotation(toeAngle);
		//camberGain.zAxisRotation(camberAngle);

		//mKnuckleRotPosWC.mult( camberGain, toeGain);
	*/

		vKnucklePosCC = vLowerBallJointCC;
		//vKnucklePosCC.y += fKnuckleOffset[1];
		//if(m_iWhich&1)
		//	vKnucklePosCC.x += fKnuckleOffset[0];
		//else
		//	vKnucklePosCC.x -= fKnuckleOffset[0];

	//vKnucklePosCC.z += fKnuckleOffset[2];
		
		vKnucklePosWC.transform(vKnucklePosCC, pCar->mCarbody.rotPos, pCar->mCarbody.linPos);



	///calc ICs .

	//float magSideIC;
	//Vector3p V1, V2, V3;
	//
	//V1 = Vector3p (0, -vStrutCC.z, vStrutCC.y);
	//V1.normalise();
	//V3 = Vector3p (0, 0, -1.0f); //for now just keep LCA parallel to base;

	//V2.cross(V3,V1);
 //   vStrutCC.cross(V1,vStrutCC );
	//magSideIC = vStrutCC.mod()/V2.mod();
	//if(fCasterAngle<0)
	//	magSideIC *=-1;


	//Vector3p V2;//(fLengthBottom*cosSweepAngle, 0.0f,fLengthBottom*sinSweepAngle);
	//Vector3p V1;
	////V1.sub(vLowerBallJointCC,vMountBottomCC );
	//float transZ;
	//V2 =  vLowerBallJointCC;
	////V2.x = 0.0f;
	//V2.normalise();
	////transZ = V1.z;
	////V2.z = 0.0f;
	////V1.mult();//scale to intersect with original mouting point
	//vSideViewIC_CC = vMountBottomCC;
	//vSideViewIC_WC.mult(pCar->mCarbody.rotPos, vSideViewIC_CC);
	//vSideViewIC_WC.add(pCar->mCarbody.linPos);
	//V1 =  vLowerBallJointCC;
	//V1.z = -V2.z;
	//V1.y -= V2.y;
	//V1.normalise();
	////This is just an approximation.. but it seems good enough
	////V1.mult();//scale to intersect with original mouting point
	//vFrontViewIC_CC.sub(vMountBottomCC,V1);
	//vFrontViewIC_WC.mult(pCar->mCarbody.rotPos, vFrontViewIC_CC);
	//vFrontViewIC_WC.add(pCar->mCarbody.linPos);
	////scale shouldn't matter as we are only concerned with the vector intersection later to find the roll axis

	//Vector3p V1;
	//Vector3p V2;	
	//V1.sub(vPerpRodRotate,vMountBottomCC );
	//V1.y = 0.0f;
	//V1.normalise();
	//float STABackYOffset = -0.05f;

	//vSideViewIC_CC = vMountBottomCC;
	//vSideViewIC_WC.mult(pCar->mCarbody.rotPos, vSideViewIC_CC);
	//vSideViewIC_WC.add(pCar->mCarbody.linPos);

	//
	////V1.mult();//scale to intersect with original mouting point
	//if(m_iWhich&1)
	//{
	//	vFrontViewIC_CC.x = vMountBottomCC.x - V1.z;
	//	vFrontViewIC_CC.y = vMountBottomCC.y + STABackYOffset;
	//	vFrontViewIC_CC.z = vMountBottomCC.z - V1.x;
	//}
	//else
	//{
	//	vFrontViewIC_CC.x = vMountBottomCC.x + V1.z;
	//	vFrontViewIC_CC.y = vMountBottomCC.y + STABackYOffset;
	//	vFrontViewIC_CC.z = vMountBottomCC.z + V1.x;
	//}

	//vFrontViewIC_WC.mult(pCar->mCarbody.rotPos, vFrontViewIC_CC);
	//vFrontViewIC_WC.add(pCar->mCarbody.linPos);
	////scale shouldn't matter as we are only concerned with the vector intersection later to find the roll axis

	Vector3p V1(-sinSweepAngle , fSTABackYOffset , cosSweepAngle);

	vSideViewIC_CC = vMountBottomCC;
	vSideViewIC_WC.mult(pCar->mCarbody.rotPos, vSideViewIC_CC);
	vSideViewIC_WC.add(pCar->mCarbody.linPos);

	
	//V1.mult();//scale to intersect with original mouting point
	vFrontViewIC_CC.sub(vMountBottomCC, V1);
	

	vFrontViewIC_WC.mult(pCar->mCarbody.rotPos, vFrontViewIC_CC);
	vFrontViewIC_WC.add(pCar->mCarbody.linPos);
	//scale shouldn't matter as we are only concerned with the vector intersection later to find the roll axis


	//this is a misnomer, it should be something like : instantWheelTravelAxis
	vInstantRollAxis.sub(vFrontViewIC_WC, vSideViewIC_WC);//instance axis that Wheel travels.


	}

	if(iType == SUSP_TYPE_SLA)
	{
		//SLA
		suspPos0.mult(pCar->mCarbody.rotPos, position);
		suspPos0.add(pCar->mCarbody.linPos);
		axisWC.mult(pCar->mCarbody.rotPos, axis);

		vLowerBallJointCC = vMountBottomCC;
		
		float sinBeta =  (((linearPosition)-restLength)/fInstallationRatio)/fLengthBottom;
		float beta = asin(sinBeta);
		float cosBeta = cos(beta);

		if(m_iWhich&1)
			vLowerBallJointCC.x +=fLengthBottom * cosBeta;
		else
			vLowerBallJointCC.x -=fLengthBottom * cosBeta;

		vLowerBallJointCC.y +=fLengthBottom *sinBeta;

		vMountBottomWC.mult(pCar->mCarbody.rotPos, vMountBottomCC);
		vMountBottomWC.add(pCar->mCarbody.linPos);
		
		vMountTopWC.mult(pCar->mCarbody.rotPos, vMountTopCC);
		vMountTopWC.add(pCar->mCarbody.linPos);

		vLowerBallJointWC.mult(pCar->mCarbody.rotPos, vLowerBallJointCC);
		vLowerBallJointWC.add(pCar->mCarbody.linPos);

		sinBeta =  ((linearPosition*fInstallationRatio))/fLengthBottom;
		beta -=M_PI/2;

		//calc upper ball joint (top of spindle)
        float topMountToChord; // from uppermount to cord
		Vector3p vTopMountToLBJ;
		vTopMountToLBJ = vLowerBallJointCC;
		vTopMountToLBJ.sub(vMountTopCC);
		float topMountToLBJ = vTopMountToLBJ.mod();

	    topMountToChord = ((fLengthTop*fLengthTop)  -(fSpindleLength*fSpindleLength) +(topMountToLBJ*topMountToLBJ))/(2.0f*topMountToLBJ);
//if negative (impossible configuration), then will be ...bad, so just fabs...
		float halfChordLength = sqrt(fabs((fLengthTop*fLengthTop)-(topMountToChord*topMountToChord)));
  //      if((m_iWhich&1))
		////{
		//	halfChordLength*=-1.0f;
		//
		////}
		//else
		//	topMountToChord *=-1.0f;
	
	if((m_iWhich&1))
			halfChordLength*=-1;

		vUpperBallJointCC = vMountTopCC;
		vTopMountToLBJ.normalise();
		//vUpperBallJointCC.mac(vTopMountToLBJ, halfChordLength);// to cord
		vUpperBallJointCC.mac(vTopMountToLBJ, topMountToChord);// to cord

		tempAltitudeBase.transform( vUpperBallJointCC, pCar->mCarbody.rotPos, pCar->mCarbody.linPos);

		float tempy = vTopMountToLBJ.y;
		vTopMountToLBJ.y = -vTopMountToLBJ.x;
		vTopMountToLBJ.x = tempy;
		vTopMountToLBJ.z= 0.0f;
		vTopMountToLBJ.normalise();

		//signs will be messed up for left/right now, but just test it.	

	
		vUpperBallJointCC.mac(vTopMountToLBJ, halfChordLength);// up to ball joint
		Vector3p UpperControlArmCC;
		UpperControlArmCC.sub(vUpperBallJointCC,vMountTopCC);
		upperarmLength = UpperControlArmCC.mod();

		Vector3p SpindleAxisCC;
		SpindleAxisCC.sub(vUpperBallJointCC,vLowerBallJointCC  );
		SpindleAxisCC.normalise();
		//vUpperBallJointCC.mac(vTopMountToLBJ, topMountToChord);// up to ball joint


		//I think the force should be applied to the vector constructed by the mounting points on the chassis.
		
		float fStrutLength;
		Vector3p vStrutCC = vMountTopCC;
		vStrutCC.sub(vLowerBallJointCC);
		fStrutLength = vStrutCC.mod();
		vInstantaneousAxisCC = vStrutCC;//for MacPherson Strut, the axis is same of strut.(for my case anyways ...
		/*
		float knuckInAngle = acos(((fStrutLength*fStrutLength)+(fLengthBottom*fLengthBottom)-(fMountLength*fMountLength))/(2*fStrutLength*fMountLength));
		float alpha = ( (beta + knuckInAngle)- M_PI*0.75f) ;
	*/

		float alpha =acos( SpindleAxisCC.dot(Vector3p(0,1.0f,0))) - (M_PI*0.5);// knuckle is oriented with the spindle
		if(!(m_iWhich&1))
			alpha = -alpha;

		float sinAlpha = sin(alpha);
		float cosAlpha = cos(alpha);

//		orientation of the knuckle. will by multiplied by noRotOrient to get final orientation
	//	calc from spindle orientation
		mKnuckleRotPosWC = Matrix3x3(
			-sinAlpha,	cosAlpha,		0,
			cosAlpha,	sinAlpha	,0,		
			0,		0	,	1 );

		///orient the knucle according to the spindle's orientation.  include toe changes later.

	/*	mKnuckleRotPosWC = Matrix3x3(
			-sinAlpha,	cosAlpha,		0,
			cosAlpha,	sinAlpha	,0,		
			0,		0	,	1 );*/


	
		/////////////////
		vKnucklePosCC = vLowerBallJointCC;
		vKnucklePosCC.y += fKnuckleOffset[1];
		if(m_iWhich&1)
			vKnucklePosCC.x += fKnuckleOffset[0];
		else
			vKnucklePosCC.x -= fKnuckleOffset[0];

		
		vKnucklePosWC.transform(vKnucklePosCC, pCar->mCarbody.rotPos, pCar->mCarbody.linPos);//i need to have some function to do these transformations...
		vUpperBallJointWC.transform(vUpperBallJointCC, pCar->mCarbody.rotPos, pCar->mCarbody.linPos);//i need to have some function to do these transformations...
	//vUpperBallJointCC.mult(mKnuckleRotPosWC, vUpperBallJointCC);





	///calc ICs .. do in CC
	//
	//float magSideIC;
	//Vector3p V1, V2, V3;
	//
	//V1 = Vector3p (0, -vStrutCC.z, vStrutCC.y);
	//V1.normalise();
	//V3 = Vector3p (0, 0, -1.0f); //for now just keep LCA parallel to base;

	//V2.cross(V3,V1);
 //   vStrutCC.cross(V1,vStrutCC );
	//magSideIC = vStrutCC.mod()/V2.mod();
	//if(fCasterAngle<0)
	//	magSideIC *=-1;

	//vSideViewIC_CC.mac(vMountTopCC, V1, magSideIC);
	//vSideViewIC_WC.mult(pCar->mCarbody.rotPos, vSideViewIC_CC);
	//vSideViewIC_WC.add(pCar->mCarbody.linPos);

 //   
	//
	//vStrutCC = vMountTopCC;
	//vStrutCC.sub(vLowerBallJointCC);

	//
	//V1 = Vector3p (-vStrutCC.y, vStrutCC.x, 0);
	//V1.normalise();
	//
	//V3.sub( vLowerBallJointCC,vMountBottomCC);
	//V3.z = 0;
	//V3.normalise();	


	//V2.cross(V3,V1);
 //   vStrutCC.cross(V1,vStrutCC );
	//magSideIC = vStrutCC.mod()/V2.mod();
	//if(!(m_iWhich&1))
	//	magSideIC *=-1;
	//
	//vFrontViewIC_CC.mac(vMountTopCC, V1, magSideIC);
	//vFrontViewIC_WC.mult(pCar->mCarbody.rotPos, vFrontViewIC_CC);
	//vFrontViewIC_WC.add(pCar->mCarbody.linPos);

	//vInstantRollAxis.sub(vFrontViewIC_WC, vSideViewIC_WC);//instance axis that Wheel travels.

		//
	float magSideIC;
	Vector3p V1, V2, V3;
	V1 = Vector3p (0, -vStrutCC.z, vStrutCC.y);
	V1.normalise();
	V3 = Vector3p (0, 0, -1.0f); //for now just keep LCA parallel to base;

	V2.cross(V3,V1);
    vStrutCC.cross(V1,vStrutCC );
	magSideIC = vStrutCC.mod()/V2.mod();
	if(fCasterAngle<0)
		magSideIC *=-1;

	vSideViewIC_CC.mac(vMountTopCC, V1, magSideIC);
	vSideViewIC_WC.mult(pCar->mCarbody.rotPos, vSideViewIC_CC);
	vSideViewIC_WC.add(pCar->mCarbody.linPos);



    /////////what the helll.
	//
	//vStrutCC = vMountTopCC;
	//vStrutCC.sub(vLowerBallJointCC);

	//
	//V1 = Vector3p (-vStrutCC.y, vStrutCC.x, 0);
	//V1.normalise();
	//
	//V3.sub( vLowerBallJointCC,vMountBottomCC);
	//V3.z = 0;
	//V3.normalise();	


	//V2.cross(V3,V1);
 //   vStrutCC.cross(V1,vStrutCC );
	//magSideIC = vStrutCC.mod()/V2.mod();
	//if(!(m_iWhich&1))
	//	magSideIC *=-1;

	//V1 = Vector3p (UpperControlArmCC.x, UpperControlArmCC.y,0.0f);
	//V1.normalise();

	//V2.sub(vMountBottomCC, vLowerBallJointCC);// = Vector3p (0, 0, -1.0f); //for now just keep LCA parallel to base;
	//V2.normalise();

	//V3.sub(vMountTopCC,vMountBottomCC);

	//Vector3p temp1, temp2;
	//temp1.cross(V3,V2);
	//temp2.cross(V1, V2);

	//magSideIC = temp1.mod()/temp2.mod();


	///
	V1 = Vector3p (UpperControlArmCC.x, UpperControlArmCC.y,0.0f);
	V1.normalise();

	V2.sub(vMountBottomCC, vLowerBallJointCC);// = Vector3p (0, 0, -1.0f); //for now just keep LCA parallel to base;
	V2.normalise();

	V3.sub(vMountTopCC,vMountBottomCC);

	Vector3p temp1, temp2;
	temp1.cross(V3,V2);
	temp2.cross(V1, V2);

	magSideIC = -temp1.mod()/temp2.mod();


	
	vFrontViewIC_CC.mac(vMountTopCC, V1, magSideIC);
	vFrontViewIC_WC.mult(pCar->mCarbody.rotPos, vFrontViewIC_CC);
	vFrontViewIC_WC.add(pCar->mCarbody.linPos);

	vInstantRollAxis.sub(vFrontViewIC_WC, vSideViewIC_WC);//instance axis that Wheel travels.


	}



	if(iType == SUSP_TYPE_1DSTRUT)
	{// motorcycle strut
		suspPos0.mult(pCar->mCarbody.rotPos, position);
		suspPos0.add(pCar->mCarbody.linPos);
		axisWC.mult(pCar->mCarbody.rotPos, axis);
		
	
	

		vMountBottomWC.mult(pCar->mCarbody.rotPos, vMountBottomCC);
		vMountBottomWC.add(pCar->mCarbody.linPos);
		
		vMountTopWC.mult(pCar->mCarbody.rotPos, vMountTopCC);
		vMountTopWC.add(pCar->mCarbody.linPos);

		if(m_iWhich&1)
		{
		mKnuckleRotPosWC = Matrix3x3(
			-1,	0,		0,
			0,-1	,0,		
			0,		0	,	-1 );
		}
		else

		mKnuckleRotPosWC = Matrix3x3(
			1,	0,		0,
			0,1	,0,		
			0,		0	,	1 );

		
		float fStrutLength;
		Vector3p vStrutCC = vMountTopCC;
		vStrutCC.sub(vMountBottomCC);
		fStrutLength = vStrutCC.mod();
		vInstantaneousAxisCC = vStrutCC;
		vLowerBallJointCC.mac(vMountBottomCC, vStrutCC, (linearPosition)-restLength);
		vLowerBallJointWC.mult(pCar->mCarbody.rotPos, vLowerBallJointCC);
		vLowerBallJointWC.add(pCar->mCarbody.linPos);

		vKnucklePosCC = vLowerBallJointCC;
		vKnucklePosCC.y += fKnuckleOffset[1];
		if(m_iWhich&1)
			vKnucklePosCC.x += fKnuckleOffset[0];
		else
			vKnucklePosCC.x -= fKnuckleOffset[0];

		vKnucklePosCC.z += fKnuckleOffset[2];
		
		vKnucklePosWC.transform(vKnucklePosCC, pCar->mCarbody.rotPos, pCar->mCarbody.linPos);//i need to have some function to do these transformations...


	///calc ICs .. do in CC
	


	vSideViewIC_CC = Vector3p(0, 0, -1.5f);//test
	vSideViewIC_WC.mult(pCar->mCarbody.rotPos, vSideViewIC_CC);
	vSideViewIC_WC.add(pCar->mCarbody.linPos);

    
	vFrontViewIC_CC = Vector3p(0, 0, 0);
	vFrontViewIC_WC.mult(pCar->mCarbody.rotPos, vFrontViewIC_CC);
	vFrontViewIC_WC.add(pCar->mCarbody.linPos);//same as macpherson but rotated 90 degrees aboug y


	vInstantRollAxis.sub(vFrontViewIC_WC, vSideViewIC_WC);//instance axis that Wheel travels.
	
	}
	
}


void CMxSusp::DoForces(float wheelForce)
{
	//done in MxCar for now.
	//Something like this.  Input is force VECTOR of wheel.  Dot with instantanious strut axis to get force applied at top mount (for MacPherson)
	//Dot with Side Vect to compute Jacking forces

}