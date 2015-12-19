#pragma once

#include <string>
#include <stdexcept>
#include <opencv2/highgui/highgui.hpp>

namespace Networking
{
	enum ChannelType
	{
		Color = 1,
		Ir = 2,
		Depth = 4
	};	 

	struct ChannelProperties
	{
		enum ChannelType ChannelType;
		unsigned int Height;
		unsigned int Width;
		int PixelType;
		unsigned int PixelSize; // in bytes

		const float DepthResolution = 2.5f; // units - mm
		const float DepthExpectedMax = 4000.f; // units - mm

		ChannelProperties(enum ChannelType type);
		std::string ToString() const;
	};
}