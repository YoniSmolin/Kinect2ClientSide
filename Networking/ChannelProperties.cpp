#include "ChannelProperties.h"

namespace Networking
{
	ChannelProperties::ChannelProperties(enum ChannelType type) : ChannelType(type)
	{
		switch (type)
		{
		case ChannelType::Color:
			Height = 1080;
			Width = 1920;
			PixelType = CV_8UC4;
			PixelSize = 4;
			break;
		case ChannelType::Depth:
			Height = 424;
			Width = 512;
			PixelType = CV_16UC1;
			PixelSize = 2;
			break;
		case ChannelType::Ir:
			Height = 424;
			Width = 512;
			PixelType = CV_8UC1;
			PixelSize = 1;
			break;
		default:
			throw std::runtime_error("Unidentifed network packet type");
		};
	}

	std::string ChannelProperties::ToString() const
	{
		std::string channelType;
		switch (ChannelType)
		{
		case ChannelType::Color: 
			channelType = std::string("Color");
			break;
		case ChannelType::Depth:
			channelType = std::string("Depth");
			break;
		case ChannelType::Ir:
			channelType = std::string("IR");
			break;
		default:
			throw std::runtime_error("Unidentifed network packet type");
		};
		return channelType + " channel: " + std::to_string(Height) + " X " + std::to_string(Width) + " pixels per frame, " + std::to_string(PixelSize) + " bytes per pixel";
	}


}