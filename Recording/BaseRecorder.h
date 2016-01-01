#pragma once
#include <string>

namespace Recording
{
	const std::wstring CameraShutterAudioFile(L"../Sounds/CameraShot.wav"); // all of those "magic strings" should be externalized, but for now this will do

	class BaseRecorder
	{
	protected:
		unsigned int _recordingCycle; // time in frames between two consecutive frame recordings
		unsigned int _cameraIndex; // index of the camera associated with this recorder
		unsigned int _savedFramesCount;
		std::string _recordingPath;

	protected:
		BaseRecorder(const std::string& recordingPath, unsigned int recordingCycle, unsigned int cameraIndex) :
			_recordingCycle(recordingCycle), _cameraIndex(cameraIndex), _recordingPath(recordingPath), _savedFramesCount(1) {}
	};
}
