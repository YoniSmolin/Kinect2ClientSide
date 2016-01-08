#include "CalibrationPatternRecorder.h"

#include "opencv2/calib3d/calib3d.hpp"
#include <Windows.h> // playing a audio file
#include <iostream>

namespace Recording
{
	const std::string PatternFilesSubDirectory("CalibrationFiles");

	CalibrationPatternRecorder::CalibrationPatternRecorder(const std::string& recordingPath, unsigned int recordingCycle, unsigned int cameraIndex, unsigned int boardWidt, unsigned int boradHeight) :
		BaseRecorder(recordingPath, recordingCycle, cameraIndex), _boardSize(cv::Size(boardWidt, boradHeight))
	{
		_recordingPath = _recordingPath + std::string("/") + PatternFilesSubDirectory;
		CreateDirectoryA(_recordingPath.c_str(), NULL);

		char cameraIndexString[3];
		sprintf_s(cameraIndexString, "_%d", cameraIndex);				

		_patternCornersFile.open(_recordingPath + std::string("/") + std::string("CalibrationFrames_Camera") + std::string(cameraIndexString) + std::string(".xml"), cv::FileStorage::WRITE);
	}

	bool CalibrationPatternRecorder::DetectCalibrationPattern(cv::Mat frame, unsigned int frameNumber)
	{
		if (_savedFramesCount * _recordingCycle < frameNumber)
			return findChessboardCorners(frame, _boardSize, _currentPatternCorners, CV_CALIB_CB_ADAPTIVE_THRESH); // consider adding: CV_CALIB_CB_NORMALIZE_IMAGE

		return false;
	}

	void CalibrationPatternRecorder::RecordLastCalibrationPattern()
	{
		char savedFrameCountString[10];
		sprintf_s(savedFrameCountString, "%02d", _savedFramesCount); // assuming savedFrameCount < 100, 2 digits should suffice

		_patternCornersFile << "Frame_" + std::string(savedFrameCountString) << _currentPatternCorners;

		std::cout << "~~~~~~~~~~~~~~~~~~~~ Camera " << _cameraIndex + 1 << ": Taking a shot ~~~~~~~~~~~~~~~~~~~~" << std::endl;

		if (_cameraIndex == 0) // don't want to waste resources on playing sound files in all threads
			PlaySound(CameraShutterAudioFile.c_str(), NULL, SND_FILENAME | SND_ASYNC); // SND_ASYNC is important, otherwise this call will block

		_savedFramesCount++;
	}

	CalibrationPatternRecorder::~CalibrationPatternRecorder()
	{		
		_patternCornersFile.release();
	}
}