#pragma once

class CMxTimer
{
public:
	CMxTimer(void);
	~CMxTimer(void);

 
 
	int curRealTime;
	int curSimTime;	// Time calculated in the sim
	int lastSimTime;
	int  GetSimTime(){ return curSimTime; }
	int  GetRealTime(){ return curRealTime; }
	int timeSpan;


  
	void AddSimTime(int msecs);
	void SetSpan(int ms);

	int GetTimeSpan(){return timeSpan;}
	int GetLastSimTime(){return lastSimTime;}
	void SetLastSimTime(){ lastSimTime=curSimTime; }
	
	// Methods
	void Start();
	void Stop();
	void Reset();
	void ResetBase();
	void Update();

	/////////////
	//actual timer
	int   iTicks;
	int   iBaseTicks;
	bool bIsRunning;
	void UpdateTicks();
	bool  IsRunning(){ return bIsRunning; }

	unsigned long GetSeconds();
	int   GetMilliSeconds();

	int    GetTicks();
		// Adjust time
	void AdjustMilliSeconds(int delta);

	// Higher level
	void WaitForTime(int secs,int msecs=0);

};


