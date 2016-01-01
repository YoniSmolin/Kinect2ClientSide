#pragma once

#include "opencv2/highgui/highgui.hpp"
#include "BaseRecorder.h"

namespace Recording
{
	class CalibrationPatternRecorder : public BaseRecorder
	{
		static const cv::Size _boardSize;

		std::vector<cv::Point2f> _currentPatternCorners;

		cv::FileStorage _patternCornersFile;

	public:
		CalibrationPatternRecorder(unsigned int recordingCycle, unsigned int cameraIndex, const std::string& recordingPath);
		~CalibrationPatternRecorder();

		bool DetectCalibrationPattern(cv::Mat frame, unsigned int frameNumber);
		void RecordLastCalibrationPattern();
	};
}