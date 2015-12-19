#pragma once

#include "PacketClient.h"
#include <opencv2/highgui/highgui.hpp>

namespace Networking
{
	class NetworkPacketProcessor
	{
		const ChannelProperties* _channelProperties; // not managed
		uchar* _imageData; // managed

	public:
		NetworkPacketProcessor(const ChannelProperties* channelProperties);
		~NetworkPacketProcessor();

		cv::Mat ProcessPacket(NetworkPacket packet);
		
	private:
		cv::Mat ProcessJpegPacket(NetworkPacket packet);
		cv::Mat ProcessPngPacket(NetworkPacket packet);
	};
}