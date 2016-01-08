#pragma once

#include "opencv2/highgui/highgui.hpp"
#include "BaseRecorder.h"

namespace Recording
{
	class CalibrationPatternRecorder : public BaseRecorder
	{
		cv::Size _boardSize;

		std::vector<cv::Point2f> _currentPatternCorners;

		cv::FileStorage _patternCornersFile;

	public:
		CalibrationPatternRecorder(const std::string& recordingPath, unsigned int recordingCycle, unsigned int cameraIndex, unsigned int boardWidt, unsigned int boradHeight);
		~CalibrationPatternRecorder();

		bool DetectCalibrationPattern(cv::Mat frame, unsigned int frameNumber);
		void RecordLastCalibrationPattern();
	};
}