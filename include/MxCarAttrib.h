
#pragma once

#ifndef _MXCAR_H_
#define _MXCAR_H_
#endif

typedef struct tagBodyAttr
{
    float rBodyLength;
    float rBodyWidth;
    float rBodyHeight;
    float iBodyMass;
    float rBodyInertial[3];
    float rPosOfEngine[3];
	float rCoM[2];
    //float rWheelBase;
	float rDistToFront;
	float rDistToRear;
	float rRearSuspHeight;
	float rFrontSuspHeight;
    float rFrontWheelBase;
    float rRearWheelBase;
    float rDragCoef;
    float rFrontDownForceCoef;
    float rRearDownForceCoef;
} BodyAttr;

typedef struct tagWheelAttr
{
    float rWidth;
    float rRadius;
    float rMass;
	float rCamber;
	float rToe;
} WheelAttr;

typedef struct tagTireAttr
{
    float rBrakeForce;
    float rCorneringStiffness;
	float rLateralStiffness;
    float rPrimaryFriction;
	float rPeakBrakingCoef;
	float rKp;

	float magicC;
	float magicE;
	float kMuZ ;
	float kMuY;
	float vxMin;
	float Lx;
	float Ly;
	
} TireAttr;

typedef struct tagSuspensionAttr
{
    float rKp;
    float rKdBounce;
	float rKdRebound;
    float rLength;
    float rUpLimit;
    float rLoLimit;
	float rARB;//anti-roll bar
	float fOffx;
	float fOffz;

	float fLowerControlArmLength;
	float fUpperControlArmLength;
	float fRollCenterY;
	float fRollCenterZ;

	float fInstallRatio;

	float fMountOffsetUpper[2];//x, y offset from susp pos.
	float  fMountOffsetLower[2];

	float fCasterAngle;
	//float fKingpinAngle;
	float fKnuckleOffset[3];

#define SUSP_TYPE_MACPHERSON 0
#define SUSP_TYPE_SLA 2
#define SUSP_TYPE_SEMITRAIL 1
#define SUSP_TYPE_1DSTRUT 3

	int iType;

	float fSpindleLength;// for SLA 
	float fSweepAngle; // for semi-trailing arm
	
} SuspensionAttr;

typedef struct tagEngineAttr
{
	float rLength;
	float rWidth;
	float rHeight;
	float rMass;
    float rTorqueCurve[10];
    float rMaxRPM;
	float rMaxFrictionTorque;
	float rMomentOfInertia;
	float rEngineFriction;
} EngineAttr;

class GearBoxAttr
{
//protected:
public:
    int   iMode;               // 0:自排  1:手排
    int   iDriveType;          // 0:前驅  1:後驅  2:AWD
    float rAWDRearPowerRatio;
    float rEfficiency;
    float rEngineBrakeCoef;
    int   iTotalGear;
    float rFGearRatio[7];
	float rFGearInertias[7];
	float rDriveshaftInertia;
    float rReverseGearRatio;
	float rReverseGearInertia;
    float rFinalDrive;
	float rAutoClutchBrake;
	float rClutchFriction;//rEfficiency may replace this and engine friction, but just in case don't need to add more to interface
	
	float iTimeDepressClutch;
	float iTimeApplyClutch;
	float bAutoBlipThrottle;

	int iMaxRPMShiftDown;
	int iMinRPMShiftUp;
	float rMaxTorqueShiftDown;
	float rMinTorqueShiftUp;
};
//add a diff calss...
class DifferentialAttr
{
public:
	float rFrontTorqueBiasNum, rFrontTorqueBiasDenom;
	float rCenterTorqueBiasNum, rCenterTorqueBiasDenom;
	float rRearTorqueBiasNum, rRearTorqueBiasDenom;

	//float rFrontLockingCoeff, rCenterLockingCoeff, rRearLockingCoeff;
	int iFrontNumClutches, iCenterNumClutches, iRearNumClutches;

	float rFrontGearRatio, rRearGearRatio, rCenterGearRatio;//duplicate "final gear"
	
	int iFrontType, iCenterType, iRearType;//
	/*#define DIFF_NONE		0
	#define DIFF_SPOOL		1
	#define DIFF_FREE		2
	#define DIFF_LIMITED_SLIP	3
	#define DIFF_VISCOUS_COUPLER	4*/

	float rFrontEfficiency, rRearEfficiency, rCenterEfficiency;
	float rFrontMaxSlipBias    ,rRearMaxSlipBias   ,rCenterMaxSlipBias  ;
	float rFrontLockingTorque    ,rRearLockingTorque   , rCenterLockingTorque  ;
	float rFrontViscosity    ,rRearViscosity   , rCenterViscosity  ;
	float rFrontMinTorqueBias    ,rRearMinTorqueBias   , rCenterMinTorqueBias  ;
	float rFrontMaxTorqueBias    ,rRearMaxTorqueBias   , rCenterMaxTorqueBias  ;
	float rFrontInertia    ,rRearInertia     , rCenterInertia    ;


};

class CMxCarAttribute
{
public:
    BodyAttr        m_Body;
    WheelAttr       m_Wheel[2];
    TireAttr        m_Tire[2];
    SuspensionAttr  m_Suspension[2];
    EngineAttr      m_Engine;
    GearBoxAttr     m_GearBox;
	DifferentialAttr m_Differentials;

    float m_rMaxSteeringAngle;
    float m_rSteeringSensitility;
	float m_rHandBrakeForce;
	float m_rRollingCoefSpeed;
	float m_rRollingCoefBasic;

protected:
private:
};