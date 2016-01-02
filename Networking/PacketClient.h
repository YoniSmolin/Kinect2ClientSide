#pragma once

#include "Client.h"
#include "ChannelProperties.h"

#include <string>
#include <vector>

namespace Networking
{
	struct Timestamp
	{
		long Seconds;
		long Milliseconds;

		// output: in milliseconds
		long operator-(Timestamp rhs) {	return 1000 * (Seconds - rhs.Seconds) + Milliseconds - rhs.Milliseconds; }
	};

	struct NetworkPacket
	{
		std::vector<unsigned char> Data;
		Timestamp Timestamp;
	};

	// manages its own memory
	class PacketClient : public Client
	{
		ChannelProperties* _channelProperties;
		char* _networkBuffer;
		unsigned int _maximalPacketSize;

	public:
		PacketClient(std::string clientName) : Client(clientName), _channelProperties(NULL), _networkBuffer(NULL), _maximalPacketSize(0) {}
		~PacketClient();

		const ChannelProperties* ReceiveMetadataPacket(); // call 1st - after connection to server !!
		void AllocateBuffers(); // call 2nd
		NetworkPacket ReceivePacket(); // call 3rd
	};
}