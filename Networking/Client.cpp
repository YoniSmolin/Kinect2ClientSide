/*
* Client.cpp -- a TCP client class
*/

#include <ws2tcpip.h>
#include <stdio.h>
#include <stdexcept> // exceptions

#include "Client.h"

#pragma region Constructors and Distructors

Client::~Client() 
{
	CloseConnection();
}

#pragma endregion

#pragma region open and close connection methods

int Client::ConnectToServer(const char* serverName, const char* portNumber)
{
	WSADATA wsaData;

	int iResult;

	// Initialize Winsock
	iResult = WSAStartup(MAKEWORD(2, 2), &wsaData);
	if (iResult != 0) {
		printf("%s: WSAStartup failed: %d\n", _name.c_str(), iResult);
		return 1;
	}

	struct addrinfo *result = NULL,
		*ptr = NULL,
		hints;

	ZeroMemory(&hints, sizeof(hints));
	hints.ai_family = AF_UNSPEC;
	hints.ai_socktype = SOCK_STREAM;
	hints.ai_protocol = IPPROTO_TCP;

	// Resolve the server address and port
	iResult = getaddrinfo(serverName, portNumber, &hints, &result);
	if (iResult != 0) {
		printf("%s: getaddrinfo failed: %d\n", _name.c_str(), iResult);
		WSACleanup();
		return 1;
	}

	// Attempt to connect to the first address returned by
	// the call to getaddrinfo
	ptr = result;

	while (ptr != NULL)
	{
		// Create a SOCKET for connecting to server
		_sockfd = socket(ptr->ai_family, ptr->ai_socktype, ptr->ai_protocol);

		if (_sockfd == INVALID_SOCKET) {
			printf("%s: Error at socket(): %ld\n", _name.c_str(), WSAGetLastError());
			freeaddrinfo(result);
			WSACleanup();
			return 1;
		}

		// Connect to server.
		iResult = connect(_sockfd, ptr->ai_addr, (int)ptr->ai_addrlen);
	
		if (iResult != SOCKET_ERROR) break; // successfully established connection

		closesocket(_sockfd);
		_sockfd = INVALID_SOCKET;
		ptr = ptr->ai_next;
	}

	freeaddrinfo(result);

	if (_sockfd == INVALID_SOCKET) {
		printf("%s: Unable to connect to server!\n", _name.c_str());
		WSACleanup();
		return 1;
	}

	// shutdown the connection for sending since no more data will be sent
	// the client can still use the ConnectSocket for receiving data
	iResult = shutdown(_sockfd, SD_SEND);
	if (iResult == SOCKET_ERROR) {
		printf("%s: shutdown failed: %d\n", _name.c_str(), WSAGetLastError());
		closesocket(_sockfd);
		WSACleanup();
		return 1;
	}

	return 0;
}

void Client::CloseConnection()
{
	// cleanup
	closesocket(_sockfd);
	WSACleanup();
}

#pragma endregion

#pragma region send and recieve methods

int Client::ReceiveMessage(char* message, int length)
{
	int numBytes;

	numBytes = recv(_sockfd, message, length, 0);
	if (numBytes < 0)
		printf("%s: recv failed: %d\n", _name.c_str(), WSAGetLastError());

	return numBytes;
}

int Client::waitUntilReceived(char* buffer, int length)
{
	int totalReceived = 0;

	while (totalReceived < length)
	{
		int received = ReceiveMessage(buffer + totalReceived, length - totalReceived);
		if (received == 0)
		{
			printf("%s: Server closed connection\n", _name.c_str());
			break;
		}
		totalReceived += received;
	}

	return totalReceived;
}

#pragma endregion