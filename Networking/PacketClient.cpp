#include "PacketClient.h"
#include <stdexcept>

namespace Networking
{
	const unsigned int BitsInByte = 8;
	const unsigned int BytesInHeader = 3;
	const unsigned int BytesInTimestampField = 4; // super dangerous - relying on sizeof(time_t) = sizeof(long) = 4 on the JetsonBoard. 
												  // The solution: the server should actually dictate the size of this field when the connection is
												  // established. I'm too lazy + have no time to implement that.
	const unsigned int BytesInMetadata = 1;

	PacketClient::~PacketClient()
	{
		if (_channelProperties) delete _channelProperties;

		if (_networkBuffer) delete[] _networkBuffer;
	}

	NetworkPacket PacketClient::ReceivePacket()
	{
		if (_channelProperties == NULL)
		{
			ReceiveMetadataPacket();
			AllocateBuffers();
		}

		NetworkPacket emptyPacket;

		// receive timestamp
		if (BytesInTimestampField != sizeof(unsigned long)) throw std::runtime_error("Expecting " + std::to_string(BytesInTimestampField) + "bytes for seconds & milliseconds fields of the timestamp. Cant put that in a long.");
		Timestamp timestamp;

		unsigned int totalReceived = waitUntilReceived((char*)&timestamp.Seconds, BytesInTimestampField);
		if (totalReceived < BytesInTimestampField) return emptyPacket;
		totalReceived = waitUntilReceived((char*)&timestamp.Milliseconds, BytesInTimestampField);
		if (totalReceived < BytesInTimestampField) return emptyPacket;

		// receive header
		uchar header[BytesInHeader];
		totalReceived = waitUntilReceived((char*)header, BytesInHeader);
		if (totalReceived < BytesInHeader) // totalReceived will be less than BYTES_IN_HEADER only if server has closed connection
			return emptyPacket;

		unsigned int dataSize = header[0];
		dataSize += header[1] << BitsInByte;
		dataSize += header[2] << 2 * BitsInByte;

		if (dataSize > _maximalPacketSize)
			throw std::runtime_error("Failed to receive packet - data size is too large");

		// receive data
		totalReceived = waitUntilReceived(_networkBuffer, dataSize);
		if (totalReceived < dataSize) // totalReceived will be less than BYTES_IN_HEASER only if server has closed connection
			throw std::runtime_error("Failed to receive packet - data size is smaller than expected");

		NetworkPacket receivedPacket{ std::vector<unsigned char>(_networkBuffer, _networkBuffer + dataSize), timestamp };

		return receivedPacket;
	}

	const ChannelProperties* PacketClient::ReceiveMetadataPacket()
	{
		uchar metadata[BytesInMetadata];
		int totalReceived = waitUntilReceived((char*)metadata, BytesInMetadata);
		if (totalReceived < BytesInMetadata) // totalReceived will be less than BYTES_IN_HEADER only if server has closed connection
			throw std::runtime_error("Failed to receive metadata packet - size is smaller than expected");	
		
		if (_channelProperties) delete _channelProperties;

		_channelProperties = new ChannelProperties((enum ChannelType)metadata[0]);

		return _channelProperties;
	}

	void PacketClient::AllocateBuffers()
	{
		if (!_channelProperties) ReceiveMetadataPacket();

		if (_networkBuffer) delete[] _networkBuffer;

		_maximalPacketSize = _channelProperties->Width * _channelProperties->Height * _channelProperties->PixelSize;
		_networkBuffer = new char[_maximalPacketSize];
	}
}