#pragma once

#include <string>
#include <opencv2/highgui/highgui.hpp>
#include "Networking/ChannelProperties.h"
#include "BaseRecorder.h"

namespace Recording
{
	// does not manage any memory
	class FrameRecorder : public BaseRecorder
	{
		const Networking::ChannelProperties* _channelProperties;		

	public:
		FrameRecorder(const std::string& recordingDirectory, const Networking::ChannelProperties* channelProperties, unsigned int recordingCycle, unsigned int cameraIndex);

		void RecordFrame(cv::Mat frame, unsigned int frameNumber);

	private:
		std::string _imageFileType;
	};
}
