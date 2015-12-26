#include "FrameRecorder.h"
#include <iostream>
#include <windows.h> // playing a audio file

#define CAMERA_SHUTTER_AUDIO_FILE "../Data/CameraShot.wav"

namespace Recording
{
	FrameRecorder::FrameRecorder(const std::string& recordingDirectory, const Networking::ChannelProperties* channelProperties, int recordingCycle, int cameraIndex) :
		_channelProperties(channelProperties), _recordingCycle(recordingCycle), _cameraIndex(cameraIndex), _savedFramesCount(0)
	{
		char cameraIndexString[3];
		sprintf_s(cameraIndexString, "%d", _cameraIndex);
		_recordingDirectory = recordingDirectory + std::string("/") + cameraIndexString;
		CreateDirectoryA(_recordingDirectory.c_str(), NULL);

		if (_channelProperties->ChannelType == Networking::ChannelType::Color)
			_imageFileType = ".jpg";
		else
			_imageFileType = ".png";
	}

	void FrameRecorder::RecordFrame(cv::Mat frame, int frameNumber)
	{
		if (_savedFramesCount * _recordingCycle < frameNumber) // (pressedKey == 32) // 32 is the spacebar // 
		{
			char savedFrameCountString[10];
			sprintf_s(savedFrameCountString, "%02d", _savedFramesCount); // assuming savedFrameCount < 100, 2 digits should suffice
			cv::imwrite(_recordingDirectory + std::string("/") + std::string(savedFrameCountString) + _imageFileType, frame);
			std::cout << "~~~~~~~~~~~~~~~~~~~~ Camera " << _cameraIndex << ": Taking a shot ~~~~~~~~~~~~~~~~~~~~" << std::endl;
			if (_cameraIndex == 1)
				PlaySound(TEXT(CAMERA_SHUTTER_AUDIO_FILE), NULL, SND_FILENAME | SND_ASYNC);

			_savedFramesCount++;
		}
	}
}