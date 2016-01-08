#include "FrameRecorder.h"
#include <iostream>
#include <windows.h> // playing a audio file

namespace Recording
{
	FrameRecorder::FrameRecorder(const std::string& recordingDirectory, const Networking::ChannelProperties* channelProperties, unsigned int recordingCycle, unsigned int cameraIndex) :
		BaseRecorder(recordingDirectory, recordingCycle, cameraIndex), _channelProperties(channelProperties)
	{
		char cameraIndexString[3];
		sprintf_s(cameraIndexString, "%d", _cameraIndex);
		_recordingPath = _recordingPath + std::string("/") + cameraIndexString;
		CreateDirectoryA(_recordingPath.c_str(), NULL);

		if (_channelProperties->ChannelType == Networking::ChannelType::Color)
			_imageFileType = ".jpg";
		else
			_imageFileType = ".png";
	}

	void FrameRecorder::RecordFrame(cv::Mat frame, unsigned int frameNumber)
	{
		if (_savedFramesCount * _recordingCycle < frameNumber) // (pressedKey == 32) // 32 is the spacebar // 
		{
			char savedFrameCountString[10];
			sprintf_s(savedFrameCountString, "%02d", _savedFramesCount); // assuming savedFrameCount < 100, 2 digits should suffice
			std::string fileName(_recordingPath + std::string("/") + std::string(savedFrameCountString) + _imageFileType);
			cv::imwrite(fileName, frame);			
			std::cout << "~~~~~~~~~~~~~~~~~~~~ Camera " << _cameraIndex + 1 << ": Taking a shot ("  << fileName << ") ~~~~~~~~~~~~~~~~~~~~" << std::endl;

			if (_cameraIndex == 0)
				PlaySound(CameraShutterAudioFile.c_str(), NULL, SND_FILENAME | SND_ASYNC); // SND_ASYNC is important, otherwise this call will block

			_savedFramesCount++;
		}
	}
}