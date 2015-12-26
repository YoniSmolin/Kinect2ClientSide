/*
* 3calibration.cpp -- Calibrate 3 cameras in a horizontal line together.
* This is a pretty crappy peace of code that I modified to support 2 cameras instead of 3.
*/

#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/imgcodecs/imgcodecs.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/utility.hpp"

#include <stdio.h>
#include <string.h>
#include <time.h>

using namespace cv;
using namespace std;

enum { DETECTION = 0, CAPTURING = 1, CALIBRATED = 2 };

static void help()
{
	printf("\nThis is a camera calibration sample that calibrates 3 horizontally placed cameras together.\n"
		"Usage: 3calibration\n"
		"     -w=<board_width>         # the number of inner corners per one of board dimension\n"
		"     -h=<board_height>        # the number of inner corners per another board dimension\n"
		"     [-s=<squareSize>]       # square size in some user-defined units (1 by default)\n"
		"     [-o=<out_camera_params>] # the output filename for intrinsic [and extrinsic] parameters\n"
		"     [-zt]                    # assume zero tangential distortion\n"
		"     [-a=<aspectRatio>]      # fix aspect ratio (fx/fy)\n"
		"     [-p]                     # fix the principal point at the center\n"
		"     [input_data]             # input data - text file with a list of the images of the board\n"
		"\n");

}

static void calcChessboardCorners(Size boardSize, float squareSize, vector<Point3f>& corners)
{
	corners.resize(0);

	for (int i = 0; i < boardSize.height; i++)
		for (int j = 0; j < boardSize.width; j++)
			corners.push_back(Point3f(float(j*squareSize),
				float(i*squareSize), 0));
}

static bool run3Calibration(vector<vector<Point2f> > imagePoints1,	vector<vector<Point2f> > imagePoints2,
	Size imageSize, Size boardSize,	float squareSize, float aspectRatio,	int flags,
	Mat& cameraMatrix1, Mat& distCoeffs1, Mat& cameraMatrix2, Mat& distCoeffs2,
	Mat& R12, Mat& T12)
{
	int c, i;

	// step 1: calibrate each camera individually
	vector<vector<Point3f> > objpt(1);
	vector<vector<Point2f> > imgpt;
	calcChessboardCorners(boardSize, squareSize, objpt[0]);
	vector<Mat> rvecs, tvecs;

	for (c = 1; c <= 2; c++)
	{
		const vector<vector<Point2f> >& imgpt0 = c == 1 ? imagePoints1 : imagePoints2; 
		imgpt.clear();
		int N = 0;
		for (i = 0; i < (int)imgpt0.size(); i++)
			if (!imgpt0[i].empty())
			{
				imgpt.push_back(imgpt0[i]);
				N += (int)imgpt0[i].size();
			}

		if (imgpt.size() < 3)
		{
			printf("Error: not enough views for camera %d\n", c);
			return false;
		}

		printf("Using a total of %zu images from camera %d for intrinsic calibration\n", imgpt.size(), c);

		objpt.resize(imgpt.size(), objpt[0]);

		Mat cameraMatrix = Mat::eye(3, 3, CV_64F);
		if (flags & CALIB_FIX_ASPECT_RATIO)
			cameraMatrix.at<double>(0, 0) = aspectRatio;

		Mat distCoeffs = Mat::zeros(5, 1, CV_64F);

		double err = calibrateCamera(objpt, imgpt, imageSize, cameraMatrix,
			distCoeffs, rvecs, tvecs,
			flags | CALIB_FIX_K3/*|CALIB_FIX_K4|CALIB_FIX_K5|CALIB_FIX_K6*/); // err is RMS over all points - check out calib3d\src\calibration.cpp, line 1561
		bool ok = checkRange(cameraMatrix) && checkRange(distCoeffs);
		if (!ok)
		{
			printf("Error: camera %d was not calibrated\n", c);
			return false;
		}
		printf("Camera %d calibration reprojection error = %g (intrinsic calibration)\n", c, err);

		if (c == 1)
			cameraMatrix1 = cameraMatrix, distCoeffs1 = distCoeffs;
		else
			cameraMatrix2 = cameraMatrix, distCoeffs2 = distCoeffs;
	}

	vector<vector<Point2f> > imgpt_right;

	imgpt.clear();
	imgpt_right.clear();
	int N = 0;

	for (i = 0; i < (int)std::min(imagePoints1.size(), imagePoints2.size()); i++)
		if (!imagePoints1[i].empty() && !imagePoints2[i].empty())
		{
			if (imagePoints1[i].size() != imagePoints2[i].size()) throw std::runtime_error("Image point vectors must be of the same size");

			imgpt.push_back(imagePoints1[i]);
			imgpt_right.push_back(imagePoints2[i]);		
			N += (int)imagePoints1[i].size();
		}

	if (imgpt.size() < 3)
	{
		printf("Error: not enough shared views for cameras 1 and %d\n", c);
		return false;
	}

	printf("Using a total of %zu images for extrinsic calibration\n", imgpt.size());

	objpt.resize(imgpt.size(), objpt[0]);
	Mat cameraMatrix = cameraMatrix2;
	Mat distCoeffs = distCoeffs2;
	Mat R, T, E, F;
	double err = stereoCalibrate(objpt, imgpt, imgpt_right, cameraMatrix1, distCoeffs1,
		cameraMatrix, distCoeffs,
		imageSize, R, T, E, F,
		CALIB_FIX_INTRINSIC,
		TermCriteria(TermCriteria::COUNT, 30, 0)); // error is RMS over all points - check out calib3d\src\calibration.cpp, line 2092 
	printf("Extrinsic calibration reprojection error = %g\n", err); 
	cameraMatrix2 = cameraMatrix;
	distCoeffs2 = distCoeffs;
	R12 = R; T12 = T;

	return true;
}

static bool readStringList(const string& filename, vector<string>& l)
{
	l.resize(0);
	FileStorage fs(filename, FileStorage::READ);
	if (!fs.isOpened())
		return false;
	FileNode n = fs.getFirstTopLevelNode();
	if (n.type() != FileNode::SEQ)
		return false;
	FileNodeIterator it = n.begin(), it_end = n.end();
	for (; it != it_end; ++it)
		l.push_back((string)*it);
	return true;
}


int main(int argc, char** argv)
{
	int i, k;
	int flags = 0;
	Size boardSize, imageSize;
	float squareSize, aspectRatio;
	string outputFilename;
	string inputFilename = "";

	vector<vector<Point2f> > imgpt[2];
	vector<string> imageList;

	cv::CommandLineParser parser(argc, argv,
		"{help ||}{w||}{h||}{s|1|}{o|out_camera_data.yml|}"
		"{zt||}{a|1|}{p||}{@input||}");
	if (parser.has("help"))
	{
		help();
		return 0;
	}
	boardSize.width = parser.get<int>("w");
	boardSize.height = parser.get<int>("h");
	squareSize = parser.get<float>("s");
	aspectRatio = parser.get<float>("a");
	if (parser.has("a"))
		flags |= CALIB_FIX_ASPECT_RATIO;
	if (parser.has("zt"))
		flags |= CALIB_ZERO_TANGENT_DIST;
	if (parser.has("p"))
		flags |= CALIB_FIX_PRINCIPAL_POINT;
	outputFilename = parser.get<string>("o");
	inputFilename = parser.get<string>("@input");
	if (!parser.check())
	{
		help();
		parser.printErrors();
		return -1;
	}
	if (boardSize.width <= 0)
		return fprintf(stderr, "Invalid board width\n"), -1;
	if (boardSize.height <= 0)
		return fprintf(stderr, "Invalid board height\n"), -1;
	if (squareSize <= 0)
		return fprintf(stderr, "Invalid board square width\n"), -1;
	if (aspectRatio <= 0)
		return printf("Invalid aspect ratio\n"), -1;
	if (inputFilename.empty() ||
		!readStringList(inputFilename, imageList) ||
		imageList.size() == 0 || imageList.size() % 2 != 0)
	{
		printf("Error: the input image list is not specified, or can not be read, or the number of files is not divisible by 3\n");
		return -1;
	}

	Mat view, viewGray;
	Mat cameraMatrix[2], distCoeffs[2], R[2], P[2], R12, T12;
	for (k = 0; k < 2; k++)
	{
		cameraMatrix[k] = Mat_<double>::eye(3, 3);
		cameraMatrix[k].at<double>(0, 0) = aspectRatio;
		cameraMatrix[k].at<double>(1, 1) = 1;
		distCoeffs[k] = Mat_<double>::zeros(5, 1);
	}	

	FileStorage fs;
	//namedWindow("Image View", 0);

	for (k = 0; k < 2; k++)
		imgpt[k].resize(imageList.size() / 2);

	for (i = 0; i < (int)(imageList.size() / 2); i++) // frame index
	{
		for (k = 0; k < 2; k++) // camera index
		{
			auto imageIndex = i + k * imageList.size() / 2;
			printf("%s\n", imageList[imageIndex].c_str());
			view = imread(imageList[imageIndex], 1);

			if (!view.empty())
			{
				vector<Point2f> ptvec(boardSize.height * boardSize.width); 
				// the original code constructed an empty vector but this caused an assertion error at the end of the iteration.
				// I don't have the time to debug this now, but there's probably some memory management issue that needs to be taken care of.
				imageSize = view.size();
				cvtColor(view, viewGray, COLOR_BGR2GRAY);
				bool found = findChessboardCorners(view, boardSize, ptvec, CALIB_CB_ADAPTIVE_THRESH); // consider adding: CV_CALIB_CB_NORMALIZE_IMAGE
				
				drawChessboardCorners(view, boardSize, Mat(ptvec), found);
				if (found)
				{
					imgpt[k][i].resize(ptvec.size());
					std::copy(ptvec.begin(), ptvec.end(), imgpt[k][i].begin()); // the discovered image plane points are written into the entry associated with the previous camera
				}
				//imshow("view", view);
				//int c = waitKey(0) & 255;
				//if( c == 27 || c == 'q' || c == 'Q' )
				//    return -1;
			}
		}
	}

	printf("Running calibration ...\n");

	run3Calibration(imgpt[0], imgpt[1], imageSize,
		boardSize, squareSize, aspectRatio, flags | CALIB_FIX_K4 | CALIB_FIX_K5,
		cameraMatrix[0], distCoeffs[0],
		cameraMatrix[1], distCoeffs[1],
		R12, T12);

	fs.open(outputFilename, FileStorage::WRITE);

	fs << "cameraMatrix1" << cameraMatrix[0];
	fs << "cameraMatrix2" << cameraMatrix[1];

	fs << "distCoeffs1" << distCoeffs[0];
	fs << "distCoeffs2" << distCoeffs[1];

	fs << "R12" << R12;
	fs << "T12" << T12;

	fs << "imageWidth" << imageSize.width;
	fs << "imageHeight" << imageSize.height;
}