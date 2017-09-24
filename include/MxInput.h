#include <dinput.h>
#pragma once

class CMxInput
{
public:
	CMxInput(void);
	~CMxInput(void);
	HRESULT InitDirectInput();
	//BOOL CALLBACK EnumJoysticksCallback( const DIDEVICEINSTANCE* pdidInstance,
 //                                    VOID* pContext );
	float GetWheelPosition();
	//float GetThrottlePosition();//my brake/throttle wires are cut, can't test
	//float GetBrakePosition();
	float GetThrottlePosition();//use the buttons... man, hardcoding all this stuff now is a bad idea...
	float GetBrakePosition();
	bool ReloadCar();

	bool ButtonPressed(int buttonNumber);
	


	HRESULT InitFFEffect();


	//oh dang... needs to be global for the callback to work
	/*LPDIRECTINPUT8       m_pDI;
	LPDIRECTINPUTDEVICE8 m_pJoystick; */
};
