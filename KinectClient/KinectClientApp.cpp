#include <opencv2\highgui\highgui.hpp>
#include <iostream>

#include "Networking\PacketClient.h" // networking class
#include "Networking\NetworkPacketProcessor.h"
#include "Networking\Timer.h"  // telemetry class

#include "Recording\FrameRecorder.h"
#include "Recording\CalibrationPatternRecorder.h"

#include <windows.h>  // multithreading
#include <process.h>  // multithreading
#include <memory>
#include <math.h>
// don't forget to pick the correct multithreading runtime library in the Visual Studio project properties in order to get this code to compile

using namespace std;
using namespace cv;

#define PORT "3490" // randomly chosen (must agree with the servers' port)
#define SERVER_NAME_HEADER "JetsonBoard" // JetsonBoardi.local where i stands for the index of the board (written with red Sharpie on the board itself)
#define SERVER_NAME_TAIL   ".local"

#define RECORDING_DIRECTORY "../Data" // path to the directory where the recordings from different Kinects will be stored (relative to solution .sln file)
#define FRAMES_BETWEEN_SHOTS 80 // frames to wait until the consecutive frame should be saved

#define FRAMES_BETWEEN_TELEMETRY_MESSAGES 30 // every so many frames, the average FPS and BW will be printed to the command window

#define THREAD_BARRIER_SPIN_COUNT -1 // the number of times a thread will attempt to check the barrier state (a.k.a. - spin) before it blocks (-1 corresponds to a system defalut value of 2k)

#define SYNCHRONIZATION_THRESHOLD 30 // frames with a time gap of less than or equal to SYNCHRONIZATION_THRESHOLD [ms] will be considered synchronized

#define MAX_NUMBER_OF_CAMERAS 4

#define CALIBRATION_PATTERN_WIDTH  6
#define CALIBRATION_PATTERN_HEIGHT 9

#pragma region Globals

LPSYNCHRONIZATION_BARRIER barrier; // a GLOBAL barrier shared among all the threads
bool FirstThreadFinished = false;// the first thread thread that finished its work raises this flag which consequently kills all the other threads
Networking::NetworkPacket Packets[MAX_NUMBER_OF_CAMERAS];
bool DroppingFrame = false;

bool RecordImages = false; // a flag to signify whether the incoming stream neet to be recorded (once every FRAMES_BETWEEN_SHOTS)
bool DisplayImages = false; // a flag to signify whether the incoming strems need to be displayed to screen
bool RecordCalibrationPattern = false; // a flag to signify whether the calibration pattern needs to be recorded (once every once every FRAMES_BETWEEN_SHOTS)
bool PatternFound[MAX_NUMBER_OF_CAMERAS] = { false, false }; // two variables that indicate whether a calibration pattern was detected in the last frame
unsigned int cameraCount;

#pragma endregion

unsigned __stdcall KinectClientThreadFunction(void* kinectIndex); // implemented below

int main(int argc, char** argv)
{
#pragma region process input arguments

	if (argc > 4 || argc < 2)
	{
		cout << "usage: " << argv[0] << " n [-r]" << endl
			<< "n - a mandatory parameter, specifies the number of clients to launch" << endl
			<< "[-ri] - an optinal flag that turns on image recording" << endl
			<< "[-rc] - an optional flag that turns on calibration pattern recording" << endl
			<< "[-di] - an optional flag do display the received image" << endl;
		return 1;
	}

	cameraCount = atoi(argv[1]);
	if (cameraCount > MAX_NUMBER_OF_CAMERAS) throw runtime_error("Currently only supporting up to " + std::to_string(MAX_NUMBER_OF_CAMERAS) + " cameras. Need to figure out a way to connect to the boards by their hostname in order to overcome this.");

	for (int argIndex = 2; argIndex < argc; argIndex++)
	{
		RecordImages = RecordImages || _strcmpi(argv[argIndex], "-ri") == 0;
		DisplayImages = DisplayImages || _strcmpi(argv[argIndex], "-di") == 0;
		RecordCalibrationPattern = RecordCalibrationPattern || _strcmpi(argv[argIndex], "-rc") == 0;
	}

	CreateDirectoryA(RECORDING_DIRECTORY, NULL);	

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

	for (unsigned int i = 0; i < cameraCount; i++)
	{
		threadIndices[i] = i; // thread index corresponds to the index of the Jetson board this thread will be talking to (but thread indices are zero-based)
		handlesToThreads[i] = (HANDLE)_beginthreadex(NULL, 0, &KinectClientThreadFunction, &threadIndices[i], CREATE_SUSPENDED, NULL);
	}

	for (unsigned int i = 0; i < cameraCount; i++)
	{
		ResumeThread(handlesToThreads[i]);
	}

#pragma endregion

#pragma region wait for threads to terminate

	for (unsigned int i = 0; i < cameraCount; i++)
	{
		WaitForSingleObject(handlesToThreads[i], INFINITE);
	}

	DeleteSynchronizationBarrier(barrier);

#pragma endregion
}

unsigned __stdcall KinectClientThreadFunction(void* kinectIndex)
{
#pragma region initialize client object

	int threadIndex = *((int*)kinectIndex);
	char cameraNameString[3];

	_itoa_s(threadIndex + 1, cameraNameString, 3, 10); // 10 is the radix, camera indices are 1-based

	Networking::PacketClient client(std::string("Client #") + cameraNameString);
	cout << "Initialized client #" << cameraNameString << " successfully" << endl; // not sure this printing is thread sage

#pragma endregion

#pragma region connect to server
	
	string serverName = string(SERVER_NAME_HEADER) + string(cameraNameString) + string(SERVER_NAME_TAIL);
	while (client.ConnectToServer(serverName.c_str(), PORT)); // loop until server goes up 
	cout << "Connected to server #" << cameraNameString << " successfully" << endl;

#pragma endregion

#pragma region obtain metadata from server

	const Networking::ChannelProperties* channelProperties = client.ReceiveMetadataPacket();
	cout << "client #" << cameraNameString << " metadata: " << channelProperties->ToString() << endl;
	client.AllocateBuffers();
	cout << "Allocated netwrok buffers for client #" << cameraNameString << endl;

#pragma endregion

#pragma region initialize packet processor

	Networking::NetworkPacketProcessor packetProcessor(channelProperties);
	cout << "Initialized packet processor for client #" << cameraNameString << endl;

#pragma endregion

#pragma region main loop

	#pragma region loop initialization

	string windowName = string("Client #") + string(cameraNameString);
	if (DisplayImages && threadIndex == 0) // will display only first thread's stream not to clutter the workspace
	{		
		namedWindow(windowName); // OpenCV window to show the image stream on screen
	}

	Timer telemetry(string("Kinect #") + string(cameraNameString), FRAMES_BETWEEN_TELEMETRY_MESSAGES);

	Recording::FrameRecorder* frameRecorder = NULL;
	if (RecordImages) frameRecorder = new Recording::FrameRecorder(RECORDING_DIRECTORY, channelProperties, FRAMES_BETWEEN_SHOTS, threadIndex);

	Recording::CalibrationPatternRecorder* calibrationRecorder = NULL;
	if (RecordCalibrationPattern) calibrationRecorder = new Recording::CalibrationPatternRecorder(RECORDING_DIRECTORY, FRAMES_BETWEEN_SHOTS, threadIndex, CALIBRATION_PATTERN_WIDTH, CALIBRATION_PATTERN_HEIGHT);

	unsigned int frameCount = 0;
	unsigned int savedFrameCount = 0;

	#pragma endregion

	while (1)
	{			
		telemetry.IterationStarted(threadIndex);
		DroppingFrame = false;

		#pragma region obtain frame

		Packets[threadIndex] = client.ReceivePacket(); // after it is received, the matrix is stored internally in the client object
		if (Packets[threadIndex].Data.size() == 0) FirstThreadFinished = true;

		RecieveBarrier: EnterSynchronizationBarrier(barrier, 0); // entering threads wlil block until all the threads reach this point [0 means no flags]

		if (FirstThreadFinished) break; // as soon as one thread is done, everybody's closing their basta
										// There's a race condition here: if FirstThreadFinished is false, one thread may quickly finish its iteration,
										// start the following one, grab an empty frame and write true to FirstThreadFinished. Then, the other thread
										// will read this value and won't be able to finish its last iteration. What a pitty.
		
		#pragma endregion

		#pragma region synchronize

		if (cameraCount > 1)
		{
			auto timestamp = Packets[threadIndex].Timestamp;
			long minGap = 0;
			for (unsigned int i = 0; i < cameraCount; i++)		
				minGap = min(minGap, timestamp - Packets[i].Timestamp);			

			if (minGap < -SYNCHRONIZATION_THRESHOLD) DroppingFrame = true;
			
			EnterSynchronizationBarrier(barrier, 0);

			if (DroppingFrame && minGap < -SYNCHRONIZATION_THRESHOLD)
			{				
				cout << "Client #" << cameraNameString << " has dropped a frame" << endl;
				continue;
			}
			else if (DroppingFrame)
			{
				goto RecieveBarrier;
			}
		}
		// once here - all threads have synchronized their frames :-)

	#pragma endregion		

		#pragma region record and/or display frame

		frameCount++;
		auto lastFrame = packetProcessor.ProcessPacket(Packets[threadIndex]);

		if (RecordImages) frameRecorder->RecordFrame(lastFrame, frameCount);

		if (RecordCalibrationPattern)
		{
			PatternFound[threadIndex] = calibrationRecorder->DetectCalibrationPattern(lastFrame, frameCount);
			EnterSynchronizationBarrier(barrier, 0); // this is wasteful in terms of time (even more so when also recording images), but I don't care (only happens when we do calibration)
			bool everyoneFound = true;
			for (unsigned int i = 0; i < cameraCount; i++)
			{
				everyoneFound = everyoneFound && PatternFound[i];
			}
			if (everyoneFound) 
				calibrationRecorder->RecordLastCalibrationPattern();
		}

		if (DisplayImages && threadIndex == 0) // remove the index == 0 condition if you wish to display the stream for every client
		{
			if (channelProperties->ChannelType == Networking::ChannelType::Depth) // lastFrame contains depth in mm but needs to be scaled for visualizatiuon purposes
				lastFrame = ((1 << channelProperties->PixelSize * 8) / channelProperties->DepthExpectedMax) * lastFrame;
			imshow(windowName, lastFrame);
			waitKey(1);
		}

	#pragma endregion

		telemetry.IterationEnded(Packets[threadIndex].Data.size());
	}

#pragma endregion

#pragma region wrap-up

	client.CloseConnection();

	printf("Average bandwidth for Kinect #%s on this session was: %2.1f [Mbps]\n", cameraNameString, telemetry.AverageBandwidth());	

	if (RecordImages) delete frameRecorder;
	if (RecordCalibrationPattern) delete calibrationRecorder;

	return 0;

#pragma endregion
}