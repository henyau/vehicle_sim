#include "StdAfx.h"
#include "include\mxtimer.h"

CMxTimer::CMxTimer(void)
{
	Reset();
}

CMxTimer::~CMxTimer(void)
{
}

//reset the starting tick of timer
void CMxTimer::ResetBase()
{
	iBaseTicks=(int)GetTickCount();

}

void CMxTimer::UpdateTicks()
{
    if(!bIsRunning)
		return;	
	int t;
	t=(int)GetTickCount();
	iTicks+=t-iBaseTicks;
	iBaseTicks=t;
}

void CMxTimer::Reset()
{ 
	ResetBase();
	iTicks=0;
	bIsRunning=FALSE;
	curRealTime = 0;
	  	
	curSimTime = 0;
	lastSimTime = 0;
	timeSpan = 0;
}
void CMxTimer::Start()
{
  if(bIsRunning)return;
  ResetBase();
  bIsRunning=TRUE;
}
void CMxTimer::Stop()
{
  if(!bIsRunning)return;
  UpdateTicks();
  bIsRunning=FALSE;
}
//////////////////

unsigned long  CMxTimer::GetSeconds()
{
	UpdateTicks();
	return iTicks/1000;
}

int CMxTimer::GetTicks()
{
	UpdateTicks();
	return (int)iTicks;
}

int CMxTimer::GetMilliSeconds()
{
	UpdateTicks();
	return iTicks;///1000;
}

void CMxTimer::AdjustMilliSeconds(int delta)
{
  // Adjust msecs
  iTicks+=delta;
}

void CMxTimer::Update()
{
	//curSimTime = GetMilliSeconds();
	curRealTime = GetMilliSeconds();

}

void CMxTimer::AddSimTime(int msecs)
{
  curSimTime+=msecs;
}

void CMxTimer::SetSpan(int ms)
{
	timeSpan=ms;
}