#include "StdAfx.h"
#include "mxdifferential.h"
#include "Sim.h"//get time step constant



CMxDifferential::CMxDifferential(void)
{

}
//CMxDifferential::CMxDifferential(CMxCarAttribute *_pCarAttr, CMxCar *_Car)
//{
//		pCar = _Car;
//		Reload(_pCarAttr);
//}

CMxDifferential::~CMxDifferential(void)
{
}

void CMxDifferential::Reload(CMxCarAttribute *_pCarAttr, int which, bool bpowered)
{
	//int type;
 //   float	m_rFinalDrive;
 //   float	m_rInertia;
 //   float m_rEfficiency;
 //   float	m_rTorqueBias;
 //   float	m_rTorqueBiasMin;
 //   float	m_rTorqueMax;
 //   float	m_rSlipBiasMas;
 //   float	m_rLockingTorque;
 //   float	m_rViscocity;
 //   float	m_rViscocityMax;
	//

	//m_rTorqueBiasCenter = _pCarAttr->m_Differentials.rCenterTorqueBiasNum/_pCarAttr->m_Differentials.rCenterTorqueBiasDenom;//4.0f/6.0f;  //for center, this is the normal torque split (under no slip)
	//m_rTorqueBiasFront =  _pCarAttr->m_Differentials.rFrontTorqueBiasNum/_pCarAttr->m_Differentials.rFrontTorqueBiasDenom;// torqueBias = torqueGrippy/torqueSlippy
	//m_rTorqueBiasRear =  _pCarAttr->m_Differentials.rRearTorqueBiasNum/_pCarAttr->m_Differentials.rRearTorqueBiasDenom;

	//m_rLockingCoefCenter = _pCarAttr->m_Differentials.rCenterLockingCoeff;
	//m_rLockingCoefRear = _pCarAttr->m_Differentials.rRearLockingCoeff;
	//m_rLockingCoefFront =_pCarAttr->m_Differentials.rFrontLockingCoeff; // for ViscLSD torqueLocking = lockingCoef*outputRotspeedDiff (rad/sec)
	//										//open locking-->lockingCoef = 0;

	//m_rDiffRatioRear =_pCarAttr->m_Differentials.rRearGearRatio; //final Drive if diff on driveAxel
	//m_rDiffRatioFront = _pCarAttr->m_Differentials.rFrontGearRatio; 

	//m_iNumClutchsRear = _pCarAttr->m_Differentials.iRearNumClutches;
	//m_iNumClutchsCenter = _pCarAttr->m_Differentials.iCenterNumClutches;
	//m_iNumClutchsFront = _pCarAttr->m_Differentials.iFrontNumClutches;

	//m_uiDiffTypeFront =  _pCarAttr->m_Differentials.iFrontType;
	//m_uiDiffTypeRear =   _pCarAttr->m_Differentials.iRearType;
	//m_uiDiffTypeCenter =   _pCarAttr->m_Differentials.iCenterType;

	//m_rRotVelIn = 0.0f;//From engine rad/sec
	//m_rTorqueIn = 0.0f;
	//m_rInertiaIn = 0.0f; 

	//for (int i =0 ;i<4; i++)
	//{
	//	m_rRotVelOut[i] = 0.0f; 
	//	m_rTorqueOut[i] = 0.0f;
	//	m_rInertiaOut[i] = 0.0f;
	//}

	m_rViscosityMax  = 1 - exp(-m_rViscosity);

	InputOut.inertia = m_rInertia * m_rFinalDrive*m_rFinalDrive + (OutputIn[0]->inertia + OutputIn[1]->inertia)/m_rEfficiency;

	//reset states
	for(int i =0; i<2 ; i++)
	{
		OutputOut[i]->rotVel = 0.0f;
		OutputOut[i]->torque = 0.0f;
		OutputOut[i]->brakingTorque = 0.0f;

		OutputIn[i]->rotVel = 0.0f;
		OutputIn[i]->torque = 0.0f;
		OutputIn[i]->brakingTorque = 0.0f;
	}

	InputIn.rotVel = 0.0f;
	InputIn.torque = 0.0f;
	InputIn.brakingTorque = 0.0f;

	InputOut.rotVel = 0.0f;
	InputOut.torque = 0.0f;
	InputOut.brakingTorque = 0.0f;

	m_iWhich = which;

	m_bPowered = bpowered;
	if(!m_bPowered)
		InputIn.inertia = 0.2f;
	
	    
}

void CMxDifferential::DoSpoolForces()
{
    float	driveTorque;
    float	rotAccel;
    float	rotVel;
    float	Tbraking;
    float	engineReaction;
    float	inertia;
    float	Tinput, Tinbrake;
    
    driveTorque = InputIn.torque;

    inertia =OutputOut[0]->inertia + OutputOut[1]->inertia;

    Tinput = OutputIn[0]->torque + OutputIn[1]->torque;

    Tinbrake = OutputIn[0]->brakingTorque + OutputIn[1]->brakingTorque;

	rotAccel = CSim::RKh * (driveTorque - Tinput) / inertia;
    rotVel = OutputIn[0]->rotVel + rotAccel;
    
    if(rotVel<0)
		Tbraking = rotVel * Tinbrake;
	else
		Tbraking = -rotVel * Tinbrake;
	
    rotAccel = CSim::RKh * Tbraking / inertia;  
	
	if (((rotAccel * rotVel) < 0.0) && (fabs(rotAccel) > fabs(rotVel)))
	{
		rotAccel = -rotVel;
    }
    if ((rotVel == 0.0) && (rotAccel < 0.0)) 
		rotAccel = 0;
    
    rotVel += rotAccel;
	
	if (m_bPowered)//match revs to wheels rotvel
	{
		engineReaction = 0.0f;//temp
		if (engineReaction != 0.0) 
			rotVel = engineReaction;
	}
	
	OutputOut[0]->rotVel = OutputOut[1]->rotVel = rotVel;

	OutputOut[0]->torque = (OutputOut[0]->rotVel - OutputIn[0]->rotVel) / CSim::RKh * OutputOut[0]->inertia;
	OutputOut[1]->torque = (OutputOut[1]->rotVel - OutputIn[1]->rotVel) / CSim::RKh * OutputOut[1]->inertia;
}


//void CMxDifferential::Forces(float _torqueIn, float _inertiaIn)
void CMxDifferential::DoForces()
{
	///////
	float driveTorque = InputIn.torque;
    float speedRatio = fabs(OutputIn[0]->rotVel - OutputIn[1]->rotVel);
	float speedRatioMax;
	float rotVel0, rotVel1;
	float Tout0, Tout1;
	float Tin0, Tin1;

	float Tdelta;
	float Vdelta;

	float rotAccel;

	rotVel0 = OutputIn[0]->rotVel;
	rotVel1 = OutputIn[1]->rotVel;
	    
    Tin0 = OutputIn[0]->torque;
    Tin1 = OutputIn[1]->torque;

 	if(speedRatio != 0.0f)
	{
		speedRatio =  fabs(OutputIn[0]->rotVel + OutputIn[1]->rotVel);///speedRatio;
        
        //Test Free Diff
		if(type ==DIFF_FREE)
		{
				if(Tin0 > Tin1) //limited by outputIn[1](more traction is available at shaft0)
				{
					if(Tin1 * driveTorque >0)
						Tout1= min(driveTorque, Tin1 *2.0f)*(0.5f + m_rTorqueBias);	//max torque is limited 						
					else
						Tout1 = min(driveTorque, -Tin1 *2.0f)*(0.5f + m_rTorqueBias);
					Tout0 = driveTorque - Tout1;		
				}
				else //torque limited by 0
				{
					if(Tin0 * driveTorque >0)
						Tout0 = min(driveTorque,Tin0 *2.0f)*(0.5f + m_rTorqueBias);							
					else
						Tout0 = min(driveTorque, -Tin0 *2.0f)*(0.5f + m_rTorqueBias);

					Tout1= driveTorque - Tout0;		
				
				}
		}//end free diff


		else if(type ==DIFF_LIMITED_SLIP)
		{
             if (driveTorque > m_rLockingTorque) //if drive exceeds locking torque, spool diff
			 {
				DoSpoolForces();
				return;
			}
			
			 speedRatioMax = m_rSlipBiasMax - driveTorque * m_rSlipBiasMax / m_rLockingTorque;
			 if (speedRatio > speedRatioMax) 
			 {
				 Vdelta = (speedRatio - speedRatioMax) * fabs(rotVel0 + rotVel1) / 2.0f;
				 if (rotVel0 > rotVel1)
				 {
					 rotVel0 -= Vdelta;
					rotVel1 += Vdelta;
				} 
				 else 
				 {
					rotVel0 += Vdelta;
					rotVel1 -= Vdelta;
				}
			 }
			 if (rotVel0 > rotVel1) 
			 {
				Tout1 = driveTorque * (0.5f + m_rTorqueBias);
				Tout0 = driveTorque * (0.5f - m_rTorqueBias);
			 } 
			 else 
			 {
				 Tout1 = driveTorque * (0.5f - m_rTorqueBias);
				 Tout0 = driveTorque * (0.5f + m_rTorqueBias);
			 }
			
		}
		else if(type == DIFF_VISCOUS_COUPLER)
		{
			if (rotVel0 >= rotVel1) 
			{
				Tout0 = driveTorque * m_rTorqueBiasMin;

				Tout1 = driveTorque * (1 - m_rTorqueBiasMin);
				} else 
			{
				Tdelta = m_rTorqueBiasMin + (1.0f - exp(-fabs(m_rViscosity * rotVel0 - rotVel1))) /
								m_rViscosityMax * m_rTorqueBiasMax;
				Tout0 = driveTorque * Tdelta;
				Tout1 = driveTorque * (1 - Tdelta);
				}
		
		}
		else if(type == DIFF_SPOOL)
		{
			DoSpoolForces();
			return;
		
		}
		else //no diff
		{
			Tout0 = 0;
			Tout1 = 0;		
		}	
	}
	else // difference is zero meaning torque split should be even.
	{
		Tout0 = driveTorque / 2.0f;
		Tout1 = driveTorque / 2.0f;
	}
	

	rotAccel = CSim::RKh * (Tout0 - Tin0)/(OutputOut[0]->inertia) 	;
	rotVel0 +=rotAccel;

	rotAccel = CSim::RKh * (Tout1 - Tin1)/(OutputOut[1]->inertia) 	;
	rotVel1 +=rotAccel;

	OutputOut[0]->rotVel = rotVel0;
	OutputOut[1]->rotVel = rotVel1;


	//if connected to engine
	//float aveRotVel =( OutputOut[0]->rotVel + 	OutputOut[1]->rotVel )*0.5f ;


    //update torques
	OutputOut[0]->torque = (OutputOut[0]->rotVel - OutputIn[0]->rotVel)/(CSim::RKh) * OutputOut[0]->inertia;
	OutputOut[1]->torque = (OutputOut[1]->rotVel - OutputIn[1]->rotVel)/(CSim::RKh) * OutputOut[1]->inertia; //T = a* I


}

void CMxDifferential::Initialize()
{

}


float CMxDifferential::GetDriveshaftVelocity()  //not used
{

	// 	
	//if(pCar->mTransmission->m_iFrontRearAWD ==DRIVETYPE_AWD)
	//{
	//	return(
	//	m_rDiffRatioRear
	//		*(pCar->mWheels[2]->rotVel + pCar->mWheels[3]->rotVel)*0.25f
	//	+
	//	m_rDiffRatioFront
	//		*(pCar->mWheels[0]->rotVel + pCar->mWheels[1]->rotVel)*0.25f);
	//}
	//else if(pCar->mTransmission->m_iFrontRearAWD ==DRIVETYPE_RWD)
	//{
	//	return(
	//	m_rDiffRatioRear
	//		*(pCar->mWheels[2]->rotVel + pCar->mWheels[3]->rotVel)*0.5f);
	//}
	//else //DRIVETYPE_FWD
	//{
	//	return(
	//	m_rDiffRatioFront
	//		*(pCar->mWheels[0]->rotVel + pCar->mWheels[1]->rotVel)*0.5f);
	//}

	return 0.0f;

}



float CMxDifferential::GetDriveshaftAcceleration()  //not used
{

	//switch(pCar->mTransmission->m_iFrontRearAWD )
	//{
	//case DRIVETYPE_FWD:
	//	return (pCar->mWheels[0]->rotAccel +pCar->mWheels[1]->rotAccel)*0.5f*m_rDiffRatioFront;
	//case DRIVETYPE_RWD:
	//	return (pCar->mWheels[2]->rotAccel +pCar->mWheels[3]->rotAccel)*0.5f*m_rDiffRatioRear;
	//case DRIVETYPE_AWD:
	//	return ((pCar->mWheels[0]->rotAccel +pCar->mWheels[1]->rotAccel)*0.25*m_rDiffRatioFront + 
	//		(pCar->mWheels[2]->rotAccel +pCar->mWheels[3]->rotAccel)*0.25f*m_rDiffRatioRear);
	//}

	return 0;
}

