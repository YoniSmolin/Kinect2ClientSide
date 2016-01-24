/*
** Timer.cpp - an implementation of a frame rate measuring class
*/
#include "Timer.h"
#include <stdio.h>
#include <iostream>

Timer::Timer(string name, int windowSize) : _frameCounter(0), _accumulatedBytes(0), _name(name), _windowSize(windowSize), _frameWindowCounter(0),
											_sessionBandwidthSum(0), _start({0}), _end({0}), _insideIteration(false), _totalCompressionTime(0),
											_compressoinStart({0}), _sessionDurationSum(0)
{
	QueryPerformanceFrequency(&_frequency);
	_decompressionLatenciesFile.open("../Data/DecompressionLatencies.log", ios::out);
	_decompressionLatenciesFile << "[";
}

Timer::~Timer()
{
	_decompressionLatenciesFile << "];";
	_decompressionLatenciesFile.close();
}

void Timer::IterationStarted(int index)
{
	if (!_insideIteration)
	{
		if (_frameCounter == 0)
		{
			QueryPerformanceCounter(&_start); // first sample of the curernt window
		}

		_frameCounter++;
		_insideIteration = true;
	}
}

void Timer::FrameDecomptressionStarted()
{
	if (_insideIteration) QueryPerformanceCounter(&_compressoinStart);
}

void Timer::FrameDecompressionEnded()
{
	if (_insideIteration)
	{
		QueryPerformanceCounter(&_end);
		float decompressionLatency = (float)(_end.QuadPart - _compressoinStart.QuadPart) / _frequency.QuadPart;
		_decompressionLatenciesFile << 1000 * decompressionLatency << ";" << endl;
		_totalCompressionTime += decompressionLatency;
	}
}

void Timer::IterationEnded(size_t numBytesMoved)
{
	if (_insideIteration)
	{
		_accumulatedBytes += numBytesMoved;		

		if (_frameCounter == _windowSize)
		{
			QueryPerformanceCounter(&_end);
			float accumulatedTime = (float)(_end.QuadPart - _start.QuadPart);
			float cycle = (accumulatedTime / _frequency.QuadPart) / _windowSize;
			float bandwidth = (_accumulatedBytes * 8 / (accumulatedTime / _frequency.QuadPart)) / (1 << 20); // Mbps
			printf("%s : Rate - %2.1f [Hz], Cycle - %2.1f [mSec], Bandwidth - %2.1f [Mbps]\n", _name.c_str(), 1 / cycle, 1000 * cycle, bandwidth);

			_sessionBandwidthSum += bandwidth;
			_sessionDurationSum += accumulatedTime;
			_accumulatedBytes = 0;
			_frameCounter = 0;
			_frameWindowCounter++;
		}

		_insideIteration = false;
	}
}

float Timer::AverageBandwidth()
{
	if (_frameWindowCounter == 0)
		return 0;

	return _sessionBandwidthSum / _frameWindowCounter;
}

float Timer::AverageDecompressionLatency()
{
	return  1000 * _totalCompressionTime /  (_windowSize * _frameWindowCounter + _frameCounter); // 1000 factor: s -> ms
}

float Timer::AverageFrameRate()
{
	if (_frameWindowCounter > 0)
		return (_windowSize * _frameWindowCounter) / (_sessionDurationSum / _frequency.QuadPart);

	QueryPerformanceCounter(&_end); // just in case we don't make it up to a full window
	return (float)_frameCounter / ((float)(_end.QuadPart - _start.QuadPart) / _frequency.QuadPart);
}
