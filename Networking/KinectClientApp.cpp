#include <opencv2/highgui/highgui.hpp>
#include <iostream>

#include "PacketClient.h" // networking class
#include "NetworkPacketProcessor.h"
#include "Timer.h"  // telemetry class

#include <windows.h>  // multithreading
#include <process.h>  // multithreading
// don't forget to pick the correct multithreading runtime library in the Visual Studio project properties in order to get this code to compile

using namespace std;
using namespace cv;

#define PORT "3490" // randomly chosen (must agree with the servers' port)
#define SERVER_NAME_HEADER "JetsonBoard" // JetsonBoardi.local where i stands for the index of the board (written with red Sharpie on the board itself)
#define SERVER_NAME_TAIL   ".local"

#define RECORDING_DIRECTORY "../Recordings/" // path to the directory where the recordings from different Kinects will be stored (relative to solution .sln file)
#define TIME_BETWEEN_SHOTS 5 // seconds to wait until the consecutive frame should be saved

#define FRAMES_BETWEEN_TELEMETRY_MESSAGES 30 // every so many frames, the average FPS and BW will be printed to the command window

#define THREAD_BARRIER_SPIN_COUNT -1 // the number of times a thread will attempt to check the barrier state (a.k.a. - spin) before it blocks (-1 corresponds to a system defalut value of 2k)

LPSYNCHRONIZATION_BARRIER barrier; // a GLOBAL barrier shared among all the threads
bool FastestThreadFinished = false;// the first thread thread that finished its work raises this flag which consequently kills all the other threads
bool Recording = false; // a flag to signify whether the incoming depth streams neet to be recorded

unsigned __stdcall KinectClientThreadFunction(void* kinectIndex); // implemented below

int main(int argc, char** argv)
{
#pragma region process input arguments

	if (argc > 3 || argc < 2)
	{
		cout << "usage: " << argv[0] << " n [-r]" << endl
			<< "n - a mandatory parameter, specifies the number of clients to launch" << endl
			<< "[-r] - an optinal flag that turns on recording" << endl;
		return 1;
	}

	int cameraCount = atoi(argv[1]);
	if (cameraCount > 2) throw exception("Currently only supporting up to two cameras. Need to figure out a way to connect to the boards by their hostname in order to overcome this.");

	Recording = (argc == 3 && _strcmpi(argv[2], "-r") == 0);
	if (Recording)
	{
		CreateDirectoryA(RECORDING_DIRECTORY, NULL);
	}

#pragma endregion

#pragma region initialize synchronziation barrier	

	barrier = (LPSYNCHRONIZATION_BARRIER)malloc(sizeof(SYNCHRONIZATION_BARRIER));
	BOOL result = InitializeSynchronizationBarrier(barrier, cameraCount, -1);

	if (result != TRUE)
	{
		cout << "Failed to initalize thread barrier" << endl << "last error: " << GetLastError() << endl;
		return 1;
	}

#pragma endregion

#pragma region launch threads

	HANDLE* handlesToThreads = new HANDLE[cameraCount];
	int* threadIndices = new int[cameraCount];

	for (int i = 0; i < cameraCount; i++)
	{
		threadIndices[i] = i + 1; // I don't like to enumerate stuff from 0, this enumaration should be consistend with the physical enumration of the devices (written in red)
		handlesToThreads[i] = (HANDLE)_beginthreadex(NULL, 0, &KinectClientThreadFunction, &threadIndices[i], CREATE_SUSPENDED, NULL);
	}

	for (int i = 0; i < cameraCount; i++)
	{
		ResumeThread(handlesToThreads[i]);
	}

#pragma endregion

#pragma region wait for threads to terminate

	for (int i = 0; i < cameraCount; i++)
	{
		WaitForSingleObject(handlesToThreads[i], INFINITE);
	}

	DeleteSynchronizationBarrier(barrier);

#pragma endregion
}

unsigned __stdcall KinectClientThreadFunction(void* kinectIndex)
{
#pragma region initialize client object

	int index = *((int*)kinectIndex);
	char clientIndex[3];

	_itoa_s(index, clientIndex, 3, 10); // 10 is the radix

	auto clientName = string(clientIndex);
	Networking::PacketClient client(clientName);
	cout << "Initialized client #" << clientIndex << " successfully" << endl; // not sure this printing is thread sage

#pragma endregion

#pragma region connect to server
	
	string serverName = string(SERVER_NAME_HEADER) + string(clientIndex) + string(SERVER_NAME_TAIL);
	while (client.ConnectToServer(serverName.c_str(), PORT)); // loop until server goes up 
	cout << "Connected to server #" << clientIndex << " successfully" << endl;

#pragma endregion

#pragma region obtain metadata from server

	const Networking::ChannelProperties* channelProperties = client.ReceiveMetadataPacket();
	cout << "client #" << clientIndex << " metadata: " << channelProperties->ToString() << endl;
	client.AllocateBuffers();
	cout << "Allocated netwrok buffers for client #" << clientIndex << endl;

#pragma endregion

#pragma region initialize packet processor

	Networking::NetworkPacketProcessor packetProcessor(channelProperties);
	cout << "Initialized packet processor for client #" << clientIndex << endl;

#pragma endregion

#pragma region main loop

	string windowName = string("Client #") + string(clientIndex);
	namedWindow(windowName); // OpenCV window to show the depth stream on screen

	Timer telemetry(string("Kinect #") + string(clientIndex), FRAMES_BETWEEN_TELEMETRY_MESSAGES);

	string recordingDir = RECORDING_DIRECTORY + string(clientIndex);
	if (Recording)
	{
		CreateDirectoryA(recordingDir.c_str(), NULL);
	}

	while (1)
	{
		EnterSynchronizationBarrier(barrier, 0); // threads wlil block here until all the threads reach here [0 means no flags]

		if (FastestThreadFinished) // as soon as one thread is done, everybody's closing their basta
			break;

		telemetry.IterationStarted();

		auto packet = client.ReceivePacket(); // after it is received the matrix is stored internally in the client object

		if (packet.size() == 0)
		{
			FastestThreadFinished = true;
		}
		else
		{
			auto lastFrame = packetProcessor.ProcessPacket(packet);
			if (channelProperties->ChannelType == Networking::ChannelType::Depth) // lastFrame contains depth in mm but needs to be scaled for visualizatiuon purposes
				lastFrame = ((1 << channelProperties->PixelSize * 8) / channelProperties->DepthExpectedMax) * lastFrame; 

			imshow(windowName, lastFrame);
			int pressedKey = waitKey(1);

			if (Recording)
			{
				static unsigned int savedFrameCount = 1;
				if (pressedKey == 32) //(savedFrameCount * TIME_BETWEEN_SHOTS < telemetry.TimeSinceFirstIteration())
				{
					string fileType;
					if (channelProperties->ChannelType == Networking::ChannelType::Color)
						fileType = ".jpg";
					else
						fileType = ".png";
					imwrite(recordingDir + string("/") + to_string(savedFrameCount) + fileType, lastFrame); // saving the correct depth values
					savedFrameCount++;
				}
			}

			telemetry.IterationEnded(packet.size());
		}
	}

#pragma endregion

#pragma region wrap-up

	client.CloseConnection();

	printf("Average bandwidth for Kinect #%s on this session was: %2.1f [Mbps]\n", clientIndex, telemetry.AverageBandwidth());

	return 0;

#pragma endregion
}