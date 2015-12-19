/*
** Timer.cpp - an implementation of a frame rate measuring class
*/
#include "Timer.h"
#include <stdio.h>

Timer::Timer(string name, int windowSize) : _sampleCounter(0), _accumulatedBytes(0), _name(name), _windowSize(windowSize), _windowsCounter(0), _sessionBandwidthSum(0), _start({0}), _end({0}), _firstIterationStart({0})
{
	QueryPerformanceFrequency(&_frequency);
}

void Timer::IterationStarted()
{
	if (_sampleCounter == 0)
	{
		QueryPerformanceCounter(&_start); // first sample of the curernt window
		if (_windowsCounter == 0)
		{ // first sample of first samples windows
			_firstIterationStart = _start;
		}
	}

	_sampleCounter++;
}

void Timer::IterationEnded(size_t numBytesMoved)
{
	_accumulatedBytes += numBytesMoved;
	 
	if (_sampleCounter == _windowSize)
	{
		QueryPerformanceCounter(&_end);
		float accumulatedTime = (float)(_end.QuadPart - _start.QuadPart);
		float cycle = (accumulatedTime / _frequency.QuadPart) / _windowSize;
		float bandwidth = (_accumulatedBytes * 8 / (accumulatedTime / _frequency.QuadPart)) / (1 << 20); // Mbps
		printf("%s : Rate - %2.1f [Hz], Cycle - %2.1f [mSec], Bandwidth - %2.1f [Mbps]\n", _name.c_str(), 1/cycle, 1000 * cycle, bandwidth);

		_sessionBandwidthSum += bandwidth;
 		_accumulatedBytes = 0;
		_sampleCounter = 0;
		_windowsCounter++;
	}
}

float Timer::TimeSinceFirstIteration()
{
	LARGE_INTEGER currentTime;
	QueryPerformanceCounter(&currentTime);
	return (float)(currentTime.QuadPart - _firstIterationStart.QuadPart) / _frequency.QuadPart;
}

float Timer::AverageBandwidth()
{
	if (_windowsCounter == 0)
		return 0;

	return _sessionBandwidthSum / _windowsCounter;
}