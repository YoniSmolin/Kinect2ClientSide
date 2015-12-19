#include "NetworkPacketProcessor.h"

namespace Networking
{
	NetworkPacketProcessor::NetworkPacketProcessor(const ChannelProperties* channelProperties) : _channelProperties(channelProperties), _imageData(NULL)
	{
		_imageData = new uchar[_channelProperties->Width * _channelProperties->Height * _channelProperties->PixelSize];
	}

	NetworkPacketProcessor::~NetworkPacketProcessor()
	{
		if (_imageData) delete[] _imageData;
	}

	cv::Mat NetworkPacketProcessor::ProcessPacket(NetworkPacket packet)
	{		
		cv::Mat currentFrameMat;

		switch (_channelProperties->ChannelType)
		{
		case ChannelType::Color: 
			currentFrameMat = ProcessJpegPacket(packet);
			break;
		case ChannelType::Depth: 
			ProcessPngPacket(packet).convertTo(currentFrameMat, _channelProperties->PixelType, _channelProperties->DepthResolution, 0);
			break;
		case ChannelType::Ir:    
			currentFrameMat = ProcessPngPacket(packet);
			break;
		default:				 
			throw std::runtime_error("Could not identify packet type");
		};

		assert(currentFrameMat.cols == _channelProperties->Width && currentFrameMat.rows == _channelProperties->Height && currentFrameMat.elemSize() == _channelProperties->PixelSize);
		return currentFrameMat;
	}

	cv::Mat NetworkPacketProcessor::ProcessJpegPacket(NetworkPacket packet)
	{		
		cv::Mat currentFrameMat(_channelProperties->Height, _channelProperties->Width, _channelProperties->PixelType, _imageData);
		imdecode(packet, CV_LOAD_IMAGE_ANYCOLOR, &currentFrameMat); // how does openCV know that receivedJpeg contains a compressed Jpeg image ? by its content.

		return currentFrameMat;
	}

	cv::Mat NetworkPacketProcessor::ProcessPngPacket(NetworkPacket packet)
	{
		cv::Mat currentFrameMat(_channelProperties->Height, _channelProperties->Width, _channelProperties->PixelType, _imageData);
		imdecode(packet, CV_LOAD_IMAGE_ANYDEPTH, &currentFrameMat); // how does openCV know that receivedPNG contains a compressed PNG image ? by its content.

		return currentFrameMat;
	}
}