/*
** Timer.h - header for a timing class 
*/
#ifndef FPS_TIMER_H
#define FPS_TIMER_H

#include <windows.h>
#include <string>

using namespace std;

class Timer
{
public:
	Timer(string name, int windowSize);

	void  IterationStarted();
	void  IterationEnded(size_t numBytesMoved);
	float TimeSinceFirstIteration();
	float AverageBandwidth(); // returns the average bit rate during the session, in units of [Mbps]

private:
	int _windowSize;
	int _sampleCounter;
	int _windowsCounter;
	LARGE_INTEGER _start, _end, _frequency, _firstIterationStart;
	float _sessionBandwidthSum;
	size_t  _accumulatedBytes;
	 
	string _name;
};

#endif