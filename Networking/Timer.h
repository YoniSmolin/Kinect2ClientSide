/*
** Timer.h - header for a timing class 
*/
#ifndef FPS_TIMER_H
#define FPS_TIMER_H

#include "PacketClient.h"
#include <windows.h>
#include <string>
#include <fstream>

using namespace std;

class Timer
{
	string _name;

	int _windowSize;
	int _frameCounter;
	int _frameWindowCounter;

	LARGE_INTEGER _start, _end, _compressoinStart, _frequency;

	float _sessionBandwidthSum;
	float _sessionDurationSum;
	size_t  _accumulatedBytes;
	float _totalCompressionTime; //ns
	ofstream _decompressionLatenciesFile;

	bool _insideIteration;

public:
	Timer(string name, int windowSize);
	~Timer();

	void IterationStarted(int index);
	void FrameDecomptressionStarted();
	void FrameDecompressionEnded();
	void IterationEnded(size_t numBytesMoved);	
	float AverageBandwidth(); // returns the average bit rate during the session, in units of [Mbps]
	float AverageDecompressionLatency();
	float AverageFrameRate();
};

#endif