#include <iostream>
#include <string>
#include <vector>
#include <iomanip>

#include <opencv2\highgui\highgui.hpp>
#include "opencv2/calib3d/calib3d.hpp"

// all of the below defines should probably turn into some sort of input argumnets in the future
#define NUMBER_OF_CAMERAS 2

#define CALIBRATION_PATTERN_WIDTH  6  // its super-important that those will agree with the widh and height in CalibrationPatternRecorder.cpp. Those constants should be organized in a common header.
#define CALIBRATION_PATTERN_HEIGHT 9
#define CALIBRATION_PATTERN_SQUARE_SIZE 5.1f // cm

#define IMAGE_WIDTH   1920
#define IMAGE_HEIGHT  1080

typedef std::vector<cv::Point2f> ImagePlanePoints;
typedef std::vector<cv::Point3f> ObjectSpacePoints;

const std::string PathToCalibrationFiles("../Data/CalibrationFiles");
const std::string OutputFileName("../Data/CalibrationResult.xml");

// output - an array of calibration frames, each frame is represented via the image coordinates of the detected calibration pattern points
std::vector<ImagePlanePoints> ReadCameraCalibrationPatterns(int cameraNumber);

// output - the 3d coordinates of the calibration pattern in the local coordinate frame of the calibration board
ObjectSpacePoints ComputeCoordinatesOfCalibrationPatternCorners(cv::Size calibrationBoardSize, float squareSize);

// copied this from stackoverflow due to lack of time
void print(cv::Mat mat, int prec);

int main()
{ 
	#pragma region prepare input for calibration

	// obtain the image plane coordinates of calibration patterns
	std::vector<std::vector<ImagePlanePoints>> calibrationCoordinatesImagePlane; // we've got 2 cameras, each has an array of ~20 calibration frames
	size_t numberOfCalibrationFrames = 0;
	for (int cameraNumber = 0; cameraNumber < NUMBER_OF_CAMERAS; cameraNumber++)
	{
		std::vector<ImagePlanePoints> imagePlanePoints = ReadCameraCalibrationPatterns(cameraNumber);
		if (!numberOfCalibrationFrames) numberOfCalibrationFrames = imagePlanePoints.size();

		if (numberOfCalibrationFrames != imagePlanePoints.size()) throw std::runtime_error("All cameras must have an equal amount of calibration frames");

		calibrationCoordinatesImagePlane.push_back(imagePlanePoints);		
	}
 
	std::cout << "Using " << numberOfCalibrationFrames << " frames to calibrate the cameras" << std::endl;

	// obtain the object space coordinates of the calibration pattern
	cv::Size calibrationBoardSize(CALIBRATION_PATTERN_WIDTH, CALIBRATION_PATTERN_HEIGHT);
	ObjectSpacePoints objectSpacePoints = ComputeCoordinatesOfCalibrationPatternCorners(calibrationBoardSize, CALIBRATION_PATTERN_SQUARE_SIZE);
	std::vector<ObjectSpacePoints> calibrationCoordinatesObjectSpace(numberOfCalibrationFrames, objectSpacePoints);

	cv::Size imageSize(IMAGE_WIDTH, IMAGE_HEIGHT);

	std::vector<cv::Mat> distortionCoeffs(NUMBER_OF_CAMERAS, cv::Mat::zeros(5, 1, CV_64F));
	int flags = CV_CALIB_ZERO_TANGENT_DIST | CV_CALIB_FIX_K3;
	// the tangential distortion is usually small, so we fix it at 0 (has a bad affect on numeric stability of the output)
	// same holds true for K3

	std::vector<cv::Mat> cameraMatrix(NUMBER_OF_CAMERAS);
	std::vector<cv::Mat> dummy1, dummy2;
	
	#pragma endregion

	#pragma region intrinsic calibration

	std::vector<double> intrinsicCalibrationReprojectionRmsError(NUMBER_OF_CAMERAS);

	for (int camera = 0; camera < NUMBER_OF_CAMERAS; camera++)
	{
		intrinsicCalibrationReprojectionRmsError[camera] = calibrateCamera(calibrationCoordinatesObjectSpace, calibrationCoordinatesImagePlane[camera],
			imageSize, cameraMatrix[camera], distortionCoeffs[camera], dummy1, dummy2, flags);
		if (!checkRange(cameraMatrix[camera]) || !checkRange(distortionCoeffs[camera]))
			throw std::runtime_error("Camera matrix or distortion coefficient vector have out-of-range entries");
		
		std::cout << "Intrinsic calibration reprojection error for camera " << camera + 1 << " is: " << intrinsicCalibrationReprojectionRmsError[camera] << std::endl;
	}

	#pragma endregion

	#pragma region extrinsic calibration

	flags = CV_CALIB_FIX_INTRINSIC; // the whole point of doing intrinsic calibration first is to obtain an accurate approximation of the intrinsic parameters
									// -> don't want the extrinsic calibration to recompute them

	cv::Mat R, T;

	double extrinsicCalibrationReprojectionRmsError = stereoCalibrate(calibrationCoordinatesObjectSpace, calibrationCoordinatesImagePlane[0], calibrationCoordinatesImagePlane[1],
		cameraMatrix[0], distortionCoeffs[0], cameraMatrix[1], distortionCoeffs[1], imageSize, R, T, dummy1[0], dummy2[0], flags, 
		cv::TermCriteria(cv::TermCriteria::COUNT, 30, 0));

	std::cout << "Extrinsic calibration reprojection error is: " << extrinsicCalibrationReprojectionRmsError << std::endl;

	#pragma endregion

	#pragma region write result to file

	cv::FileStorage fileStorage;
	fileStorage.open(OutputFileName, cv::FileStorage::WRITE);

	fileStorage << "cameraMatrix1" << cameraMatrix[0];
	fileStorage << "cameraMatrix2" << cameraMatrix[1];

	fileStorage << "distortionCoeffs1" << distortionCoeffs[0];
	fileStorage << "distortionCoeffs2" << distortionCoeffs[1];

	fileStorage << "Rotation" << R;
	fileStorage << "Translation" << T;

	fileStorage.release();

	#pragma endregion

	#pragma region print results

	std::cout << "Translation: " << std::endl;
	print(T, 3);
	std::cout << "Rotation: " << std::endl;
	print(R, 3);

	#pragma endregion

	return 0;
}

// this function is poorely written, all constant strings etc. should be extracted to some place that gathers this kind of crap.
std::vector<ImagePlanePoints> ReadCameraCalibrationPatterns(int cameraNumber)
{
	char cameraName[50];
	sprintf_s(cameraName, "CalibrationFrames_Camera_%d.xml", cameraNumber);
	std::string filePath = PathToCalibrationFiles + std::string("/") + cameraName;
	cv::FileStorage fileStorage(filePath, cv::FileStorage::READ);
	if (!fileStorage.isOpened())	throw std::runtime_error(std::string("Could not open file ") + filePath.c_str());

	std::vector<ImagePlanePoints> result;

	char frameString[10];
	int frameNumber = 1;
	sprintf_s(frameString, "Frame_%02d", frameNumber);
	
	while (!fileStorage[frameString].isNone())
	{
		std::vector<cv::Point2f> imagePoints;
		fileStorage[frameString] >> imagePoints;
		result.push_back(imagePoints);

		sprintf_s(frameString, "Frame_%02d", ++frameNumber);
	}

	return result;
}

ObjectSpacePoints ComputeCoordinatesOfCalibrationPatternCorners(cv::Size calibrationBoardSize, float squareSize)
{
	ObjectSpacePoints corners;

	for (int i = 0; i < calibrationBoardSize.height; i++)
		for (int j = 0; j < calibrationBoardSize.width; j++)
			corners.push_back(cv::Point3f(j*squareSize, i*squareSize, 0)); // the calibration pattern plane is defined as z=0 for simplicity

	return corners;
}

void print(cv::Mat mat, int prec)
{
	for (int i = 0; i<mat.size().height; i++)
	{
		std::cout << "[";
		for (int j = 0; j<mat.size().width; j++)
		{
			std::cout << std::setprecision(prec) << mat.at<double>(i, j);
			if (j != mat.size().width - 1)
				std::cout << ", ";
			else
				std::cout << "]" << std::endl;
		}
	}
}