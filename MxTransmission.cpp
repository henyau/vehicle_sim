#include "StdAfx.h"
#include "MxTransmission.h"

CMxTransmission::CMxTransmission(CMxCarAttribute *_pCarAttr, CMxEngine *_pEngine)
{
	pEngine = _pEngine;
	m_ShiftTimer = new CMxTimer; //i think there should only be one timer/ sim timer... too bad I can't access it.

}

CMxTransmission::~CMxTransmission(void)
{
}


void CMxTransmission::Reload(CMxCarAttribute *_pCarAttr)
{
	
	///tranny
	m_iCurrentGear = 0;

	//m_fClutchTorque = 0.0f;
	//m_fClutchKv= _pCarAttr->m_GearBox.rClutchFriction;
	//m_iFrontRearAWD = _pCarAttr->m_GearBox.iDriveType; // 0:«eÅX  1:«áÅX  2:AWD
	//m_fFRPowerSplit = _pCarAttr->m_GearBox.rAWDRearPowerRatio;



	m_fEffiency = _pCarAttr->m_GearBox.rEfficiency;

	m_iNumGears = _pCarAttr->m_GearBox.iTotalGear;
	m_rAutoClutchBrake = _pCarAttr->m_GearBox.rAutoClutchBrake;

	m_rFinalDrive = _pCarAttr->m_GearBox.rFinalDrive;

	for(int i = 0 ; i<7; i++)
		m_rFowardGearRatios[i] = _pCarAttr->m_GearBox.rFGearRatio[i];// * m_rFinalDrive;

	m_rReverseGearRatio = _pCarAttr->m_GearBox.rReverseGearRatio;
	m_iMaxGears = _pCarAttr->m_GearBox.iTotalGear;
	

 	m_AutoShiftUp = _pCarAttr->m_GearBox.iMinRPMShiftUp;
	m_AutoShiftDown = _pCarAttr->m_GearBox.iMaxRPMShiftDown;//use torque?
 	m_iAutoTrans = _pCarAttr->m_GearBox.iMode; //0 auto 1 manual

	omegaG = 0.0f; 

	for(int i =0; i<m_iNumGears; i++)
		m_rFowardGearInertias[i] = _pCarAttr->m_GearBox.rFGearInertias[i];

	m_rReverGearInertia = _pCarAttr->m_GearBox.rReverseGearInertia;
	//m_rFinalDriveInertia// in differential...

	m_rClutchPos = 0.0f;
	m_bIsShiftingUp = false;
	m_bIsShiftingDown = false;


	m_iTimeStartShift = 0;
	m_iTimeDepressClutch = _pCarAttr->m_GearBox.iTimeDepressClutch;
	m_iTimeApplyClutch = _pCarAttr->m_GearBox.iTimeApplyClutch;
	m_bAutoBlipThrottle = _pCarAttr->m_GearBox.bAutoBlipThrottle;
	if(!m_iAutoTrans)
		m_bAutoBlipThrottle = false;
	m_bChangedGears = false;
}

void CMxTransmission::BeginUpdate(float _throttle, float _brakePos, float _clutchPos, bool shiftUp, bool shiftDown, int timeMS)//, float _clutchPos, float _handBrakePos)
{
	

	omegaE = pEngine->vel;

	//implement analog clutch later. clutch is internally analog, if you want bool clutch just pass 0/1 to interface
  	//if(!_clutchEngaged)
		//m_rClutchPos =1.0f;
	//else 

	m_rClutchPos =_clutchPos;
//	m_rHandBrakePos = _handBrakePos

    pEngine->vel0 = omegaE;
	pEngine->m_rThrottlePos = _throttle;

	//update auto gear changes
	/* if(m_iAutoTrans ==0 )//&& m_iCurrentGear>0) 
	{
		
		if(omegaE*9.54929658551f >m_AutoShiftUp)//TEST Important, this needs to be changed to torque
		{

			if(m_iCurrentGear<(int) m_iMaxGears && m_iCurrentGear != 0)
			{
			
				omegaE *= m_rFowardGearRatios[m_iCurrentGear] / m_rFowardGearRatios[m_iCurrentGear- 1];			
			 		m_iCurrentGear++;
			}
			else if(m_iCurrentGear==0 )
			{
			
				// omegaE *= 1.0f/m_rFowardGearRatios[m_iCurrentGear];// / m_rFowardGearRatios[m_iCurrentGear- 1];			
				m_iCurrentGear++; 
			}
		}

 		if(omegaE*9.54929658551f <m_AutoShiftDown)
			if(m_iCurrentGear>1) {
				
// 				omegaE *= m_rFowardGearRatios[m_iCurrentGear-2] / m_rFowardGearRatios[m_iCurrentGear - 1];				
//				m_iCurrentGear--;
				omegaE *= m_rFowardGearRatios[m_iCurrentGear-2] / m_rFowardGearRatios[m_iCurrentGear - 1];
				m_iCurrentGear--;
			}


	}
*/
	 //else//manny		 
	 {
		 if((shiftUp&&(m_iCurrentGear<(int)m_iMaxGears))||m_bIsShiftingUp)
		 {
			 if(!m_bOldShiftUpButton&&!m_bIsShiftingUp)// first time (in this sequence).  
			 {
				 m_iTimeStartShift =timeMS;
				 m_bIsShiftingUp = true;
				 m_bChangedGears = false;
			 }
			 else if(m_bIsShiftingUp)// already started to shift
			 {
				 pEngine->m_rThrottlePos= 0.0f; 
				 if(timeMS-m_iTimeStartShift<m_iTimeDepressClutch)
				 {
					 m_rClutchPos = 0.0f;
//					 m_rClutchPos = 1.0f-((float)timeMS-(float)m_iTimeStartShift)/(float)m_iTimeDepressClutch;// linear ease off									 
					 m_rClutchPos *= _clutchPos;
				 }
				 else if(timeMS-m_iTimeStartShift>m_iTimeDepressClutch&&timeMS-m_iTimeStartShift<m_iTimeApplyClutch)
				 {					
					m_rClutchPos = ((float)timeMS-(float)m_iTimeStartShift)/(float)m_iTimeApplyClutch;
					pEngine->m_rThrottlePos =m_rClutchPos*_throttle;
					m_rClutchPos *= _clutchPos;
				 }
				 if(timeMS-m_iTimeStartShift>m_iTimeApplyClutch)
				 {
					pEngine->m_rThrottlePos = _throttle;
					m_rClutchPos = _clutchPos;		 
					m_bIsShiftingUp =false;
				 }
				 if(!m_bChangedGears&&timeMS-m_iTimeStartShift>m_iTimeDepressClutch)
				 {
					 m_iCurrentGear++;
					 //m_bIsShifting = false;//done shifting
					 m_bChangedGears = true;
				 }				 
			 
			 }
		 }

		 else if((shiftDown&& m_iCurrentGear>=0)||m_bIsShiftingDown)	
		 {
			if(!m_bOldShiftDownButton&&!m_bIsShiftingDown)// first time (in this sequence).  
			 {
				 m_iTimeStartShift =timeMS;
				 m_bIsShiftingDown = true;
				 m_bChangedGears = false;
			 }
			 else if(m_bIsShiftingDown)// already started to shift
			 {
				 pEngine->m_rThrottlePos= 0.0f; 
				 if(timeMS-m_iTimeStartShift<m_iTimeDepressClutch)
				 {
					 m_rClutchPos = 0.0f;
					 //m_rClutchPos = 1.0f-((float)timeMS-(float)m_iTimeStartShift)/(float)m_iTimeDepressClutch;// linear ease off									 
					 //m_rClutchPos *= m_rClutchPos*m_rClutchPos;
					 m_rClutchPos *= _clutchPos;
//					 if(pEngine->vel>104.72f)//over 1000rpm
					 if(m_bAutoBlipThrottle&&m_iCurrentGear>1)
						pEngine->m_rThrottlePos = 0.7f;	//blip...			 
				 }
				 else if(timeMS-m_iTimeStartShift>m_iTimeDepressClutch&&timeMS-m_iTimeStartShift<m_iTimeApplyClutch)
				 {					
					m_rClutchPos = ((float)timeMS-(float)m_iTimeStartShift)/(float)m_iTimeApplyClutch;
					pEngine->m_rThrottlePos =m_rClutchPos*_throttle;				
					m_rClutchPos *= _clutchPos;
				 }
				 if(timeMS-m_iTimeStartShift>m_iTimeApplyClutch)
				 {
					pEngine->m_rThrottlePos = _throttle;
					m_rClutchPos = _clutchPos;		 
					m_bIsShiftingDown =false;
				 }
				 if(!m_bChangedGears&&timeMS-m_iTimeStartShift>m_iTimeDepressClutch)
				 {
					 m_iCurrentGear--;
					 m_bChangedGears = true;
				 }				 
			 
			 }
		 		
		 }
		
 		 /* // brute force shifting. clutch not applied
		 if(shiftUp&&!m_bOldShiftUpButton &&m_iCurrentGear<(int)m_iMaxGears && m_iCurrentGear != 0)
		 {	
			// omegaE *= m_rFowardGearRatios[m_iCurrentGear] / m_rFowardGearRatios[m_iCurrentGear- 1];			
			 	 m_iCurrentGear++;
		 }
		else if(shiftUp&&!m_bOldShiftUpButton &&m_iCurrentGear==0 )
		 {		
			m_iCurrentGear++;
		 }
		  else if(shiftDown&&!m_bOldShiftDownButton&& m_iCurrentGear>1)
		 {
			 
		//	 omegaE *= m_rFowardGearRatios[m_iCurrentGear-2] / m_rFowardGearRatios[m_iCurrentGear - 1];
			 m_iCurrentGear--;
		 }
		 else if(shiftDown&&!m_bOldShiftDownButton&& m_iCurrentGear==1)//1rst to neutral
		 {	 
			 m_iCurrentGear--;
		 }
		 else if(shiftDown&&!m_bOldShiftDownButton&&m_iCurrentGear == 0)//neutral shift to reverse
		 {
			m_iCurrentGear--;		 
		 }

		 else if(shiftUp&&!m_bOldShiftUpButton && m_iCurrentGear==-1)//reverse to neutral
		 {
			 m_iCurrentGear++;		 
		 }
		 */

	 }
	m_bOldShiftUpButton = shiftUp;
	m_bOldShiftDownButton = shiftDown;

	if(timeMS-m_iTimeStartShift>m_iTimeDepressClutch && !m_iAutoTrans)
	 {		 
		m_bOldShiftUpButton = false;
		m_bOldShiftDownButton = false;
	 }		
	pEngine->vel = omegaE;
	
}



//void CMxTransmission::DoForces(float omegaD)//drive shaft velocity (from wheels)
//{
//
//	
//	
//    if (m_iCurrentGear >= 1) {                         // forward   
//        omegaG = omegaD * m_rFowardGearRatios[m_iCurrentGear-1];
//		//if(omegaE>omegaG)
//			m_fClutchTorque = m_rClutchPos*m_fClutchKv * (omegaE - omegaG);//omegaE = engine's velocity
//   //     else
//			//m_fClutchTorque = m_rClutchPos*m_fClutchKv * (omegaG - omegaE);
//
//		//m_fClutchTorque = m_rClutchPos*omegaE;//m_rClutchPos*m_fClutchKv * (omegaE - omegaG);
//        //m_fDriveshaftTorque =m_fClutchTorque * m_rFowardGearRatios[m_iCurrentGear - 1];
//
//		//m_fClutchTorque = pEngine->m_rTorque + pEngine->m_mOfI* pEngine->acc1;
//		m_fDriveshaftTorque = m_rClutchPos*(pEngine->m_rTorque - pEngine->m_mOfI* pEngine->acc1
//									- pEngine->acc1*m_rFowardGearInertias[m_iCurrentGear-1])
//										* m_rFowardGearRatios[m_iCurrentGear - 1];		
//
//    }
//    else if (m_iCurrentGear < 0) {                     // reverse
//        omegaG = omegaD * m_rReverseGearRatio;
//        m_fClutchTorque = m_rClutchPos*omegaE;
//        m_fDriveshaftTorque = m_fClutchTorque * m_rReverseGearRatio;
//    }
//    else {                                          // neutral
//        m_fClutchTorque = m_fDriveshaftTorque = 0.0;
//	}
//
//	if(	m_rBrakePos>m_rAutoClutchBrake)
//	{
//	  m_fClutchTorque = m_fDriveshaftTorque = 0.0;
//	}
//
//	
////float omegaD = pCar->getDriveVel();
///*
//	if(omegaD>1000.0f)//bound the drive velocity... could explode
//		omegaD=1000.0f;
//	
// 	float omegaG; //gearbox velocity
//	
//    if (m_iCurrentGear >= 1) {                         // forward   
//        omegaG = omegaD * m_rFowardGearRatios[m_iCurrentGear-1];
//         //m_fClutchTorque = m_rClutchPos*m_fClutchKv * (omegaE - omegaG);//omegaE = engine's velocity
//		m_fClutchTorque = m_rClutchPos*m_fClutchKv * (omegaE - omegaG);
//        m_fDriveshaftTorque =m_fClutchTorque * m_rFowardGearRatios[m_iCurrentGear - 1];
//
//    }
//    else if (m_iCurrentGear < 0) {                     // reverse
//        omegaG = omegaD * m_rReverseGearRatio;
//        m_fClutchTorque = m_rClutchPos*m_fClutchKv * (omegaE - omegaG);
//        m_fDriveshaftTorque = m_fClutchTorque * m_rReverseGearRatio;
//    }
//    else {                                          // neutral
//        m_fClutchTorque = m_fDriveshaftTorque = 0.0;
//	}
//
//	if(	m_rBrakePos>m_rAutoClutchBrake)
//	{
//	  m_fClutchTorque = m_fDriveshaftTorque = 0.0;
//	}
// */
//}

void CMxTransmission::DoForces(float omegaD)//drive shaft velocity 
{
	//	omegaD *= m_rFinalDrive; //drive shaft velocity pass drive shaft velocity since final gear is stored in differentials (and could be up to 3 )
	if (m_iCurrentGear >= 1)   // forward   //for now assume clutch is locked
	{   
		omegaG = omegaD * m_rFowardGearRatios[m_iCurrentGear-1];//clutch/gearbox velocity

		//m_fClutchTorque = m_rClutchPos*m_fClutchKv * (omegaE - omegaG);//omegaE = engine's velocity
//		m_fClutchTorque = 	m_rClutchPos*(pEngine->m_rTorque - pEngine->m_mOfI* pEngine->acc1); //Te - Ie Ae
		//gillespie
		
		/*m_fDriveshaftTorque = m_fClutchTorque* m_rFowardGearRatios[m_iCurrentGear - 1]
					- pEngine->acc1*m_rFowardGearInertias[m_iCurrentGear-1];	*/
			
			/*m_rClutchPos*(pEngine->m_rTorque - pEngine->m_mOfI* pEngine->acc1
									- pEngine->acc1*m_rFowardGearInertias[m_iCurrentGear-1])
										* m_rFowardGearRatios[m_iCurrentGear - 1];*/

		//gillespie //valid when engine/clutch locked
        //m_fDriveshaftTorque = (m_fClutchTorque- m_rFowardGearInertias[m_iCurrentGear]*pEngine->acc1  )
		//										* m_rFowardGearRatios[m_iCurrentGear-1]; //Td = (Tc - It * Ae)* Nt

	//m_fClutchTorque
		//m_fClutchTorque = 	m_rClutchPos*(pEngine->m_rTorque - pEngine->m_mOfI* pEngine->acc1); //Te - Ie Ae
		//m_fDriveshaftTorque = (m_fClutchTorque )* m_rFowardGearRatios[m_iCurrentGear-1]; //Td = (Tc - It * Ae)* Nt
	m_fDriveshaftTorque = 0.0f;// moved clutch to driveline... this is no longer useful
        
    }

    else if (m_iCurrentGear < 0) {                     // reverse
      /*  omegaG = omegaD * m_rReverseGearRatio;
        m_fClutchTorque = m_rClutchPos*omegaE;
        m_fDriveshaftTorque = m_fClutchTorque * m_rReverseGearRatio;*/

		//moved

		m_fDriveshaftTorque = 0.0;
    }
    else {                                          // neutral
      	m_fDriveshaftTorque = 0.0;
	}

	if(	m_rBrakePos>m_rAutoClutchBrake && omegaG*9.54929658551f<800.0f )//temp Idle
	{ 
		  m_fDriveshaftTorque = 0.0;
	}


}

float CMxTransmission::GetCurrentRatio()
{
	if (m_iCurrentGear >= 1) 
		return (m_rFowardGearRatios[m_iCurrentGear-1]);
	else if (m_iCurrentGear < 0)
		return m_rReverseGearRatio;
	else // neutral ..dont want div by zero
		return 0.0f;
}

float CMxTransmission::GetCurrentInertia()
{
	if (m_iCurrentGear >= 1) 
		return (m_rFowardGearInertias[m_iCurrentGear-1]);
	else if (m_iCurrentGear < 0)
		return m_rReverGearInertia;
	else // neutral ..dont want div by zero
		return 1.0f;
}


//float CMxTransmission::GetFinalDriveRatio()
//{
//	return m_rFinalDrive;
//}
