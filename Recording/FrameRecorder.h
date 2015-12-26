#pragma once

#include <string>
#include <opencv2/highgui/highgui.hpp>
#include "Networking/ChannelProperties.h"

namespace Recording
{
	// does not manage any memory
	class FrameRecorder
	{
		std::string _recordingDirectory;
		const Networking::ChannelProperties* _channelProperties;
		int _recordingCycle; // time in frames between two consecutive frame recordings
		int _cameraIndex; // index of the camera associated with this recorder

	public:
		FrameRecorder(const std::string& recordingDirectory, const Networking::ChannelProperties* channelProperties, int recordingCycle, int cameraIndex);

		void RecordFrame(cv::Mat frame, int frameNumber);

	private:
		int _savedFramesCount;
		std::string _imageFileType;
	};
}
