#pragma once
#include <string>

#define CAMERA_SHUTTER_AUDIO_FILE "../Data/CameraShot.wav"

namespace Recording
{
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
