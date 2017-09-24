#include "StdAfx.h"
#include "MxDriveline.h"
#include "MxDifferential.h"

CMxDriveline::CMxDriveline(CMxCarAttribute *_pCarAttr, CMxEngine *_pEngine, CMxTransmission *_pTransmission)
{
	pEngine = _pEngine;
	pTransmission = _pTransmission;
	pCar = pEngine->pCar;
	Reload( _pCarAttr);
}

CMxDriveline::~CMxDriveline(void)
{

}


void CMxDriveline::Reload(CMxCarAttribute *_pCarAttr)
{
    
    
//	pDifferential = _pDifferential;

	bAutoClutch = false;
	bClutchSlipping = false; // slip can mean "clutch fully depressed" or "cluch/engine not completely locked"
	bInGear = false;	 //need since shifting means out of gear
	bDiffLocked = false;
	rClutchVel = 0.0f;
	rTorqueClutch = 0.0f;
	rDriveshaftTorque = 0.0f;
	rDriveshaftAccel = 0.0f;
	rClutchNormalForce = 0.0f; // = m_rClutchMaxNormalForce * clutchPos^2;

	//these are properties and should be in the GUI
	m_rClutchMaxNormalForce =200.0f;
	m_iClutchInterfaces = 2;
	m_rClutchMeanRadius = 7.0f;
	m_rClutchFriction =  _pCarAttr->m_GearBox.rClutchFriction; //0.2f;//-0.4 for dry clutches

	//reload drive types/clutch etc 
	//m_fClutchTorque = 0.0f;
	m_fClutchKv= _pCarAttr->m_GearBox.rClutchFriction;


	m_iFrontRearAWD = _pCarAttr->m_GearBox.iDriveType; // 0:«eÅX  1:«áÅX  2:AWD
	m_fFRPowerSplit = _pCarAttr->m_GearBox.rAWDRearPowerRatio;



	///////reload the 3 diffs

	mDifferential[FRONT_DIFF].m_rFinalDrive = _pCarAttr->m_Differentials.rFrontGearRatio;
	mDifferential[REAR_DIFF].m_rFinalDrive = _pCarAttr->m_Differentials.rRearGearRatio; 
	mDifferential[CENTER_DIFF].m_rFinalDrive = _pCarAttr->m_Differentials.rCenterGearRatio; 

	mDifferential[REAR_DIFF].m_rTorqueBias= _pCarAttr->m_Differentials.rCenterTorqueBiasNum/_pCarAttr->m_Differentials.rCenterTorqueBiasDenom;//4.0f/6.0f;  //for center, this is the normal torque split (under no slip)
	mDifferential[FRONT_DIFF].m_rTorqueBias =  _pCarAttr->m_Differentials.rFrontTorqueBiasNum/_pCarAttr->m_Differentials.rFrontTorqueBiasDenom;// torqueBias = torqueGrippy/torqueSlippy
	mDifferential[CENTER_DIFF].m_rTorqueBias =  _pCarAttr->m_Differentials.rRearTorqueBiasNum/_pCarAttr->m_Differentials.rRearTorqueBiasDenom;

	mDifferential[FRONT_DIFF].type =  _pCarAttr->m_Differentials.iFrontType;
	mDifferential[REAR_DIFF].type =  _pCarAttr->m_Differentials.iRearType;
	mDifferential[CENTER_DIFF].type =  _pCarAttr->m_Differentials.iCenterType;


	mDifferential[FRONT_DIFF].m_rEfficiency= _pCarAttr->m_Differentials.rFrontEfficiency ;
	mDifferential[REAR_DIFF].m_rEfficiency=		_pCarAttr->m_Differentials.rRearEfficiency;
	mDifferential[CENTER_DIFF].m_rEfficiency=		_pCarAttr->m_Differentials.rCenterEfficiency;

	mDifferential[FRONT_DIFF].m_rSlipBiasMax=		_pCarAttr->m_Differentials.rFrontMaxSlipBias   ;
	mDifferential[REAR_DIFF].m_rSlipBiasMax=				_pCarAttr->m_Differentials.rRearMaxSlipBias   ;
	mDifferential[CENTER_DIFF].m_rSlipBiasMax=	 _pCarAttr->m_Differentials.rCenterMaxSlipBias  ;

	mDifferential[FRONT_DIFF].m_rLockingTorque=		_pCarAttr->m_Differentials.rFrontLockingTorque ;
	mDifferential[REAR_DIFF].m_rLockingTorque=	_pCarAttr->m_Differentials.rRearLockingTorque   ;
	mDifferential[CENTER_DIFF].m_rLockingTorque=_pCarAttr->m_Differentials.rCenterLockingTorque  ;

	 mDifferential[FRONT_DIFF].m_rViscosity = _pCarAttr->m_Differentials.rFrontViscosity;
	 mDifferential[REAR_DIFF].m_rViscosity= _pCarAttr->m_Differentials.rRearViscosity;
	 mDifferential[CENTER_DIFF].m_rViscosity= _pCarAttr->m_Differentials.rCenterViscosity  ;


	 mDifferential[FRONT_DIFF].m_rTorqueBiasMin=_pCarAttr->m_Differentials.rFrontMinTorqueBias;
	 mDifferential[REAR_DIFF].m_rTorqueBiasMin= _pCarAttr->m_Differentials.rRearMinTorqueBias;
	 mDifferential[CENTER_DIFF].m_rTorqueBiasMin=  _pCarAttr->m_Differentials.rCenterMinTorqueBias  ;
	 
	 
	 mDifferential[FRONT_DIFF].m_rTorqueBiasMax= _pCarAttr->m_Differentials.rFrontMaxTorqueBias ;
	 mDifferential[REAR_DIFF].m_rTorqueBiasMax= _pCarAttr->m_Differentials.rRearMaxTorqueBias ;
	 mDifferential[CENTER_DIFF].m_rTorqueBiasMax=  _pCarAttr->m_Differentials.rCenterMaxTorqueBias  ;

	 mDifferential[FRONT_DIFF].m_rInertia= _pCarAttr->m_Differentials.rFrontInertia   ;
	 mDifferential[REAR_DIFF].m_rInertia= _pCarAttr->m_Differentials.rRearInertia ;
	 mDifferential[CENTER_DIFF].m_rInertia=  _pCarAttr->m_Differentials.rCenterInertia  ;

	 //connect the diffs

	for (int j = 0; j < 2; j++) 
	{
		mDifferential[FRONT_DIFF].OutputIn[j]  = &(pCar->mWheels[j]->InputOut);
		mDifferential[FRONT_DIFF].OutputOut[j] = &(pCar->mWheels[j]->InputIn);
    }
    for (j = 0; j < 2; j++) 
	{
		mDifferential[REAR_DIFF].OutputIn[j]  = &(pCar->mWheels[2+j]->InputOut);
		mDifferential[REAR_DIFF].OutputOut[j] = &(pCar->mWheels[2+j]->InputIn);
	}
    mDifferential[CENTER_DIFF].OutputIn[0]  = &(mDifferential[FRONT_DIFF].InputOut);
    mDifferential[CENTER_DIFF].OutputOut[0] = &(mDifferential[FRONT_DIFF].InputIn);

    mDifferential[CENTER_DIFF].OutputIn[1]  = &(mDifferential[REAR_DIFF].InputOut);
    mDifferential[CENTER_DIFF].OutputOut[1] = &(mDifferential[REAR_DIFF].InputIn);


	 //set some properties depending on transmission type

	 if(m_iFrontRearAWD == DRIVETYPE_FWD)
	 {
		 m_rFinalGear = mDifferential[FRONT_DIFF].m_rFinalDrive;
		 m_rEfficiency = mDifferential[FRONT_DIFF].m_rEfficiency;
	 }
	 else if(m_iFrontRearAWD == DRIVETYPE_RWD)
	 {
		 m_rFinalGear = mDifferential[REAR_DIFF].m_rFinalDrive;
		 m_rEfficiency = mDifferential[REAR_DIFF].m_rEfficiency;
	 }
	 else// if(m_iFrontRearAWD == DRIVETYPE_AWD)
	 {
		 m_rFinalGear = mDifferential[CENTER_DIFF].m_rFinalDrive;
		 m_rEfficiency =  mDifferential[CENTER_DIFF].m_rEfficiency*mDifferential[FRONT_DIFF].m_rEfficiency*mDifferential[REAR_DIFF].m_rEfficiency;
	 }


	 //init states
    
	 pTransmission->rCurrentInertia = 0.5f;//test

    switch(m_iFrontRearAWD) {
		case DRIVETYPE_RWD:
		
			mDifferential[REAR_DIFF].OutputOut[0]->inertia = pTransmission->rCurrentInertia/ 2.0f + mDifferential[REAR_DIFF].OutputIn[0]->inertia;
			mDifferential[REAR_DIFF].OutputOut[1]->inertia = pTransmission->rCurrentInertia / 2.0f + mDifferential[REAR_DIFF].OutputIn[1]->inertia;
			mDifferential[REAR_DIFF].OutputOut[0]->torque = 0.0f;
			mDifferential[REAR_DIFF].OutputOut[1]->torque = 0.0f;

			mDifferential[REAR_DIFF].InputIn.inertia = pEngine->m_mOfI;
		
			mDifferential[FRONT_DIFF].Reload(_pCarAttr, 0, false);
			mDifferential[REAR_DIFF].Reload(_pCarAttr, 1, true);
			mDifferential[CENTER_DIFF].Reload(_pCarAttr, 2, false);
		

		break;
    case DRIVETYPE_FWD:
		
		mDifferential[FRONT_DIFF].OutputOut[0]->inertia = pTransmission->rCurrentInertia/ 2.0f + mDifferential[FRONT_DIFF].OutputIn[0]->inertia;
		mDifferential[FRONT_DIFF].OutputOut[1]->inertia = pTransmission->rCurrentInertia/ 2.0f + mDifferential[FRONT_DIFF].OutputIn[0]->inertia;
		mDifferential[FRONT_DIFF].OutputOut[0]->torque = 0;
		mDifferential[FRONT_DIFF].OutputOut[1]->torque = 0;
		mDifferential[FRONT_DIFF].InputIn.inertia = pEngine->m_mOfI;

		mDifferential[FRONT_DIFF].Reload(_pCarAttr, 0, true);
		mDifferential[REAR_DIFF].Reload(_pCarAttr, 1, false);
		mDifferential[CENTER_DIFF].Reload(_pCarAttr, 2, false);
	break;

    case DRIVETYPE_AWD:
	
	mDifferential[FRONT_DIFF].OutputOut[0]->inertia = pTransmission->rCurrentInertia/ 4.0f + mDifferential[FRONT_DIFF].OutputIn[0]->inertia;
	mDifferential[FRONT_DIFF].OutputOut[1]->inertia =  pTransmission->rCurrentInertia/ 4.0f + mDifferential[FRONT_DIFF].OutputIn[1]->inertia ;
	mDifferential[FRONT_DIFF].OutputOut[0]->torque = 0.0f;
	mDifferential[FRONT_DIFF].OutputOut[1]->torque = 0.0f;
	
	mDifferential[REAR_DIFF].OutputOut[0]->inertia = pTransmission->rCurrentInertia/ 4.0f + mDifferential[REAR_DIFF].OutputIn[0]->inertia ;
	mDifferential[REAR_DIFF].OutputOut[1]->inertia =  pTransmission->rCurrentInertia/ 4.0f + mDifferential[REAR_DIFF].OutputIn[1]->inertia ;
	mDifferential[REAR_DIFF].OutputOut[0]->torque = 0;
	mDifferential[REAR_DIFF].OutputOut[1]->torque = 0;

	mDifferential[CENTER_DIFF].OutputOut[0]->inertia = pTransmission->rCurrentInertia/ 2.0f + mDifferential[CENTER_DIFF].OutputIn[0]->inertia ;
	mDifferential[CENTER_DIFF].OutputOut[1]->inertia =  pTransmission->rCurrentInertia/ 2.0f+ mDifferential[CENTER_DIFF].OutputIn[1]->inertia ;
	mDifferential[CENTER_DIFF].OutputOut[0]->torque = 0;
	mDifferential[CENTER_DIFF].OutputOut[1]->torque = 0;


	mDifferential[CENTER_DIFF].InputIn.inertia = pEngine->m_mOfI;

	
	mDifferential[FRONT_DIFF].Reload(_pCarAttr, 0, false);
	mDifferential[REAR_DIFF].Reload(_pCarAttr, 1, false);
	mDifferential[CENTER_DIFF].Reload(_pCarAttr, 2, true);

	break;
    }

	m_Inertia = 1.0f;
	m_DriveShaftInertia = _pCarAttr->m_GearBox.rDriveshaftInertia;

}
float CMxDriveline::GetFinalDriveRatio()
{
	if(m_iFrontRearAWD == DRIVETYPE_FWD)
		return( mDifferential[FRONT_DIFF].m_rFinalDrive);
	else if(m_iFrontRearAWD == DRIVETYPE_RWD)
		return( mDifferential[REAR_DIFF].m_rFinalDrive);	
	else //f(m_iFrontRearAWD == DRIVETYPE_AWD)	
        return( mDifferential[CENTER_DIFF].m_rFinalDrive);
	
}

float CMxDriveline::GetOverallRatio()
{
	if(m_iFrontRearAWD == DRIVETYPE_FWD)
	{
		return(pTransmission->GetCurrentRatio() * mDifferential[FRONT_DIFF].m_rFinalDrive);
	
	}
	else if(m_iFrontRearAWD == DRIVETYPE_RWD)
	{
		return(pTransmission->GetCurrentRatio() * mDifferential[REAR_DIFF].m_rFinalDrive);	
	}
	else //(m_iFrontRearAWD == DRIVETYPE_AWD)
	{
        return(pTransmission->GetCurrentRatio() * mDifferential[CENTER_DIFF].m_rFinalDrive);
	}

}
float CMxDriveline::GetDriveWheelVel()
{
	if(m_iFrontRearAWD == DRIVETYPE_FWD)
		return(  (pCar->mWheels[0]->rotVel + pCar->mWheels[1]->rotVel)*0.5f	);
	else if(m_iFrontRearAWD == DRIVETYPE_RWD)
		return(  (pCar->mWheels[2]->rotVel + pCar->mWheels[3]->rotVel)*0.5f	);
	else //f(m_iFrontRearAWD == DRIVETYPE_AWD)	
        return(  (pCar->mWheels[0]->rotVel + pCar->mWheels[1]->rotVel+
					  pCar->mWheels[2]->rotVel + pCar->mWheels[3]->rotVel)*0.25f	);

}

float CMxDriveline::GetDriveWheelAccel()
{
	if(m_iFrontRearAWD == DRIVETYPE_FWD)
		return(  (pCar->mWheels[0]->rotAccel + pCar->mWheels[1]->rotAccel)*0.5f	);
	else if(m_iFrontRearAWD == DRIVETYPE_RWD)
		return(  (pCar->mWheels[2]->rotAccel + pCar->mWheels[3]->rotAccel)*0.5f	);
	else //f(m_iFrontRearAWD == DRIVETYPE_AWD)	
        return(  (pCar->mWheels[0]->rotAccel + pCar->mWheels[1]->rotAccel+
					  pCar->mWheels[2]->rotAccel + pCar->mWheels[3]->rotAccel)*0.25f	);

}
float CMxDriveline::GetClutchVel()
{
	return GetOverallRatio()*GetDriveWheelVel();
	//return(pTransmission->GetCurrentRatio() * mDifferential[FRONT_DIFF]->
}

float CMxDriveline::GetClutchAccel()
{
	return GetOverallRatio()*GetDriveWheelAccel();
}
float CMxDriveline::GetDifferentialInertia()
{
	float effectiveDiffInertia = 0;
	for(int i=0; i<3; i++)
	{
		if(mDifferential[i].m_bPowered)
			effectiveDiffInertia+=mDifferential[i].m_rFinalDrive*mDifferential[i].m_rFinalDrive*mDifferential[i].m_rInertia;
	}
	return effectiveDiffInertia;


}

void CMxDriveline::DoForces(float omegaD)//velocity of  drive wheels
{///note... remove float omegaD. need to get veleocities from diffs

	//find inertia of all of the driveline
	//if not connected to clutch set to 1
	if(pTransmission->m_rClutchPos == 0)
		m_Inertia = 1.0f; // what is this????
	else
	{
		//inertia of gearbox
		m_Inertia = pTransmission->GetCurrentRatio()*pTransmission->GetCurrentRatio();
		m_Inertia *=pTransmission->GetCurrentInertia();
		
		m_Inertia += m_DriveShaftInertia;
		
		//differential inertia
		m_Inertia += GetDifferentialInertia();

		//wheel inertia? somewhat difficult for me since I only use torque axes to connect wheels to diffs...so not sure m_iWhich wheels provide drive
		//for now, just lump 2 wheel inertias
        m_Inertia += 2.0f*pCar->mWheels[0]->momentOfInertia;
               
	
	}

//	float driveshaftVel = pDifferential->GetDriveshaftVelocity();
	float driveshaftVel = GetDriveWheelVel()*GetFinalDriveRatio();//
	
	pTransmission->DoForces(driveshaftVel); //find torque to apply to driveshaft
	//moving driveshaft forces to CMxDriveline, so tranmission doesn't do anything at the moment
	//in the future it should add the influence of current gear inertias 


	rClutchVel = GetClutchVel();//driveshaftVel * pTransmission->GetCurrentRatio();
	//	rClutchVel = omegaD*pTransmission->GetFinalDriveRatio()*pTransmission->GetCurrentRatio(); 

	
	//if((fabs(omegaD) <= 3.0f && pEngine->m_rThrottlePos<0.001f )||pTransmission->m_rClutchPos<1.00f|| pTransmission->m_iCurrentGear ==0)// if engine speed drops below this limit, disengage the clutch
	//	UnlockEngineClutch();
	//else if(fabs(pEngine->vel) >3.0f  &&  (pTransmission->m_iCurrentGear !=0)) //is clutch pedal not pushed down and auto clutch not engaged and not neutral
	//   	LockEngineClutch();
		
		//only test locked clutch for now

	//if(fabs(rClutchVel - pEngine->vel)>5.0f) // clutch and engine not insync
	//	UnlockEngineClutch();
			
	if(pTransmission->m_iCurrentGear ==0 || pTransmission->m_rClutchPos<1.0f)
		UnlockEngineClutch();
	//clutch is engaged (by pedal or by putting gear into neutral), m_iWhich reminds me we need two current gears
	//one for the Driver and one for the car.  Since car may not be in the gear the driver wants (time delay/forced auto clutch/changing gears(into neutral))
	
	if(!IsClutchSlipping())//if locked
	{
		//rTorqueClutch = pTransmission->m_fClutchTorque;//(pEngine->m_rTorque - pEngine->m_mOfI* pEngine->acc1);  //Te - Ie Ae, input to transmission
						////////	rDriveshaftTorque = rTorqueClutch * m_rFowardGearRatios[m_iCurrentGear-1];


		//find ClutchTorque ourselves
		rTorqueClutch = (pEngine->m_rTorque - pEngine->m_mOfI* pEngine->acc1);		
		rDriveshaftTorque = (rTorqueClutch )* pTransmission->GetCurrentRatio();  ;//pTransmission->m_fDriveshaftTorque;//Torque output TO the DRIVESHAFT (before diffs)
	}
	else//slipping
	{
		//rTorqueClutch =pTransmission->m_rClutchPos*(pEngine->m_rTorque - pEngine->m_mOfI* pEngine->acc1);  //Te - Ie Ae
//		rDriveshaftTorque = rTorqueClutch * pTransmission->GetCurrentRatio();
	
		//rDriveshaftTorque = pTransmission->m_fDriveshaftTorque / 100.0f;

//		rTorqueClutch = pTransmission->m_fClutchTorque;//pTransmission->m_rClutchPos*(pEngine->m_rTorque - pEngine->m_mOfI* pEngine->acc1);  //Te - Ie Ae
//		rDriveshaftTorque = pTransmission->m_fDriveshaftTorque;//pTransmission->m_fDriveshaftTorque;
//     		
		//rTorqueClutch = m_iClutchInterfaces * m_rClutchMeanRadius * rClutchNormalForce
		//	* (m_rClutchFriction /EngineClutchRelVel) 	;

		////if(rTorqueClutch<0)
		////	rTorqueClutch =0;

		//rDriveshaftTorque = rTorqueClutch*pTransmission->GetCurrentRatio();
		/////////////end modelica's clutch

		//rTorqueClutch = 0.0f;
			///////		rTorqueClutch = pTransmission->m_fClutchTorque;//(pEngine->m_rTorque - pEngine->m_mOfI* pEngine->acc1);  //Te - Ie Ae, input to transmission
		//rDriveshaftTorque = pTransmission->m_fDriveshaftTorque;

		
		//rTorqueClutch = pTransmission->m_rClutchPos*(pEngine->m_rTorque - pEngine->m_mOfI* pEngine->acc1);
		//rDriveshaftTorque = (rTorqueClutch )* pTransmission->GetCurrentRatio();

		////////based on  Modelica's document clutch
		float	EngineClutchRelVel = (pEngine->vel - rClutchVel);
		rClutchNormalForce = m_rClutchMaxNormalForce * (pTransmission->m_rClutchPos)*(pTransmission->m_rClutchPos);

		//rTorqueClutch = m_iClutchInterfaces * m_rClutchMeanRadius * rClutchNormalForce* (m_rClutchFriction /EngineClutchRelVel);
		//if(fabs(pEngine->m_rTorque - pEngine->m_mOfI* pEngine->acc1)>0.1f)
		//	pTransmission->m_rClutchPos = m_rClutchFriction*EngineClutchRelVel/(pEngine->m_rTorque - pEngine->m_mOfI* pEngine->acc1);
		//if(pTransmission->m_rClutchPos >1.0f)
		//	pTransmission->m_rClutchPos =1.0f;
		//if(pTransmission->m_rClutchPos <0.0f)
		//	pTransmission->m_rClutchPos =0.0f;
		//auto modulate clutch to match engine

		rTorqueClutch = (pTransmission->m_rClutchPos)*(pTransmission->m_rClutchPos)*m_rClutchFriction*EngineClutchRelVel;
 		rDriveshaftTorque = (rTorqueClutch )* pTransmission->GetCurrentRatio();

	}
	//add driveShaftTorque to car body torque for body roll (chassis twist)
 	//pCar->addBodyTorque(Vector3p(0,0,1.0f), pEngine->m_rTorque);// temp bike

	//update differential forces
	for(int i = 0; i<3; i++)
	{
		if(mDifferential[i].m_bPowered)
			mDifferential[i].InputIn.torque = rDriveshaftTorque*GetFinalDriveRatio();//*GetOverallRatio();
		mDifferential[i].DoForces();
	}
	
}


void CMxDriveline::EulerStep()
{
	float clutchEngineVelDeltaPrev, clutchEngineVelDelta;
	clutchEngineVelDeltaPrev = pEngine->vel - rClutchVel;

	if(!IsClutchSlipping())//locked but out of sync
	{
		if(fabs(clutchEngineVelDeltaPrev)>20.0f)
		{//engine/clutch out of sync, unlock until they rotate at same rate
			UnlockEngineClutch();		
		}	
	}

	pEngine->EulerStep(rTorqueClutch);//evolve engine...

	if(IsClutchSlipping()&&pTransmission->m_rClutchPos>0)//slipping
	{
		clutchEngineVelDelta = pEngine->vel - rClutchVel;
		if((clutchEngineVelDeltaPrev>0&&clutchEngineVelDelta<0)||
			(clutchEngineVelDeltaPrev<0&&clutchEngineVelDelta>0)) //engine-clutch difference has reversed
		{	
			LockEngineClutch();
			pEngine->vel = rClutchVel; //force engine to spin at same rate as clutch
			//pEngine->acc1 += pTransmission->GetCurrentRatio()*pDifferential->GetDriveshaftAcceleration();


		}

 	}

}