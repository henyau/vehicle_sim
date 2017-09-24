#include "StdAfx.h"
#include ".\mxinput.h"
//simple joystick class

#define SAFE_DELETE(p)  { if(p) { delete (p);     (p)=NULL; } }
#define SAFE_RELEASE(p) { if(p) { (p)->Release(); (p)=NULL; } }
LPDIRECTINPUT8       m_pDI;
LPDIRECTINPUTDEVICE8 m_pJoystick; 

CMxInput::CMxInput(void)
{
	m_pDI    = NULL;         
	m_pJoystick    = NULL;   
	//InitDirectInput();
}

CMxInput::~CMxInput(void)
{
	    // Unacquire the device one last time just in case 
    // the app tried to exit while the device is still acquired.
    if( m_pJoystick ) 
        m_pJoystick->Unacquire();
    
    // Release any DirectInput objects.
    SAFE_RELEASE( m_pJoystick );
    SAFE_RELEASE( m_pDI );
}

BOOL CALLBACK EnumJoysticksCallback( const DIDEVICEINSTANCE* pdidInstance,
                                     VOID* pContext )
{
    HRESULT hr;

    // Obtain an interface to the enumerated Joystick.
	hr = m_pDI->CreateDevice( pdidInstance->guidInstance, &m_pJoystick, NULL );

    // If it failed, then we can't use this Joystick. (Maybe the user unplugged
    // it while we were in the middle of enumerating it.)
    if( FAILED(hr) ) 
        return DIENUM_CONTINUE;

    // Stop enumeration. Note: we're just taking the first Joystick we get. You
    // could store all the enumerated Joysticks and let the user pick.
    return DIENUM_STOP;
}


HRESULT CMxInput::InitDirectInput()
{
    HRESULT hr;

    // Register with the DirectInput subsystem and get a pointer
    // to a IDirectInput interface we can use.
    // Create a DInput object
    if( FAILED( hr = DirectInput8Create( GetModuleHandle(NULL), DIRECTINPUT_VERSION, 
                                         IID_IDirectInput8, (VOID**)&m_pDI, NULL ) ) )
        return hr;

    // Look for a simple Joystick we can use for this sample program.
    if( FAILED( hr = m_pDI->EnumDevices( DI8DEVCLASS_GAMECTRL, 
										EnumJoysticksCallback,
                                         NULL, DIEDFL_ATTACHEDONLY ) ) )
        return hr;



    // Set the data format to "simple Joystick" - a predefined data format 
    //
    // A data format specifies which controls on a device we are interested in,
    // and how they should be reported. This tells DInput that we will be
    // passing a DIJOYSTATE2 structure to IDirectInputDevice::GetDeviceState().
    if( FAILED( hr = m_pJoystick->SetDataFormat( &c_dfDIJoystick2 ) ) )
        return hr;


    return S_OK;
}



float CMxInput::GetWheelPosition()
{
    HRESULT     hr;
    DIJOYSTATE2 js;           // DInput Joystick state 

    if( NULL == m_pJoystick ) 
        return S_OK;

    // Poll the device to read the current state
    hr = m_pJoystick->Poll(); 
    if( FAILED(hr) )  
    {
        hr = m_pJoystick->Acquire();
        while( hr == DIERR_INPUTLOST ) 
            hr = m_pJoystick->Acquire();
        return S_OK; 
    }

    // Get the input's device state
    if( FAILED( hr = m_pJoystick->GetDeviceState( sizeof(DIJOYSTATE2), &js ) ) )
        return hr; // The device should have been acquired during the Poll()

   	return js.lX;//all we care about

	
}

float CMxInput::GetBrakePosition()
{
    HRESULT     hr;
    DIJOYSTATE2 js;           // DInput Joystick state 
    
    if( NULL == m_pJoystick ) 
        return S_OK;
    
    hr = m_pJoystick->Poll(); 
    if( FAILED(hr) )  
    {
        hr = m_pJoystick->Acquire();
        while( hr == DIERR_INPUTLOST ) 
            hr = m_pJoystick->Acquire();
        return S_OK; 
    }

    // Get the input's device state
    if( FAILED( hr = m_pJoystick->GetDeviceState( sizeof(DIJOYSTATE2), &js ) ) )
        return hr; // The device should have been acquired during the Poll()

	//digital button
/*		if(js.rgbButtons[0] & 0x80)
		{
			return true;
		}	
		return false;*/ 
	
	//analog axis
	return js.lRz;
	
}


float CMxInput::GetThrottlePosition()
{
    HRESULT     hr;
    DIJOYSTATE2 js;           // DInput Joystick state 
    
    if( NULL == m_pJoystick ) 
        return S_OK;
    
    hr = m_pJoystick->Poll(); 
    if( FAILED(hr) )  
    {
        hr = m_pJoystick->Acquire();
        while( hr == DIERR_INPUTLOST ) 
            hr = m_pJoystick->Acquire();
        return S_OK; 
    }

    // Get the input's device state
    if( FAILED( hr = m_pJoystick->GetDeviceState( sizeof(DIJOYSTATE2), &js ) ) )
        return hr; // The device should have been acquired during the Poll()

	/*	if(js.rgbButtons[1] & 0x80)
		{
			return true;
		}
		
		return false;*/
	return js.lY;

}


bool CMxInput::ReloadCar()
{
	HRESULT     hr;
    DIJOYSTATE2 js;           // DInput Joystick state 
    
    if( NULL == m_pJoystick ) 
        return S_OK;
    
    hr = m_pJoystick->Poll(); 
    if( FAILED(hr) )  
    {
        hr = m_pJoystick->Acquire();
        while( hr == DIERR_INPUTLOST ) 
            hr = m_pJoystick->Acquire();
        return S_OK; 
    }

    // Get the input's device state
    if( FAILED( hr = m_pJoystick->GetDeviceState( sizeof(DIJOYSTATE2), &js ) ) )
        return hr; // The device should have been acquired during the Poll()

		if(js.rgbButtons[2] & 0x80)
		{
			return true;
		}
		
		return false;

}

bool CMxInput::ButtonPressed(int buttonNumber)
{
	HRESULT     hr;
    DIJOYSTATE2 js;           // DInput Joystick state 
    
    if( NULL == m_pJoystick ) 
        return S_OK;
    
    hr = m_pJoystick->Poll(); 
    if( FAILED(hr) )  
    {
        hr = m_pJoystick->Acquire();
        while( hr == DIERR_INPUTLOST ) 
            hr = m_pJoystick->Acquire();
        return S_OK; 
    }

    // Get the input's device state
    if( FAILED( hr = m_pJoystick->GetDeviceState( sizeof(DIJOYSTATE2), &js ) ) )
        return hr; // The device should have been acquired during the Poll()

		if(js.rgbButtons[buttonNumber] & 0x80)
		{
			return true;
		}
		
		return false;

}

HRESULT CMxInput::InitFFEffect()
{

	HRESULT  hr;
	LPDIRECTINPUTEFFECT lpdiEffect;  // receives pointer to created effect
              // parameters for created effect
	DIEFFECT diEffect; 
	DWORD    dwAxes[2] = { DIJOFS_X, DIJOFS_Y };
	LONG     lDirection[2] = { 18000, 0 };

	DICONSTANTFORCE diConstantForce; 

	ZeroMemory(&diEffect,sizeof(diEffect));
	diConstantForce.lMagnitude = DI_FFNOMINALMAX;   // Full force

	diEffect.dwSize          = sizeof(DIEFFECT); 
	diEffect.dwFlags         = DIEFF_CARTESIAN|DIEFF_OBJECTOFFSETS;//DIEFF_POLAR | DIEFF_OBJECTOFFSETS; 
	diEffect.dwDuration      = (DWORD)(1110.5 * DI_SECONDS);
	diEffect.dwSamplePeriod  = 0;                 // = default 
	diEffect.dwGain          = DI_FFNOMINALMAX;   // No scaling
	diEffect.dwTriggerButton = DIEB_NOTRIGGER;    // Not a button response
	diEffect.dwTriggerRepeatInterval = 0;         // Not applicable
	diEffect.cAxes                   = 1; 
	diEffect.rgdwAxes                = &dwAxes[0]; 
	diEffect.rglDirection            = &lDirection[0]; 
	diEffect.lpEnvelope              = 0; 
	diEffect.cbTypeSpecificParams    = sizeof(DICONSTANTFORCE);
	diEffect.lpvTypeSpecificParams   = &diConstantForce;  
	diEffect.dwStartDelay=0;
	
	hr = m_pJoystick->CreateEffect(GUID_ConstantForce,&diEffect,&lpdiEffect,0);


	DIDEVCAPS diDevCaps;
	diDevCaps.dwSize=sizeof(DIDEVCAPS);
	m_pJoystick->GetCapabilities(&diDevCaps);
///disable auto center
	 DIPROPDWORD dw;
	dw.diph.dwSize=sizeof(DIPROPDWORD);
	dw.diph.dwHeaderSize=sizeof(DIPROPHEADER);
	dw.diph.dwObj=0;
	dw.diph.dwHow=DIPH_DEVICE;
	dw.dwData=DIPROPAUTOCENTER_OFF;
	m_pJoystick->SetProperty(DIPROP_AUTOCENTER,&dw.diph);

	///

	//lpdiEffect->Start(1,0);
	return hr;

}

