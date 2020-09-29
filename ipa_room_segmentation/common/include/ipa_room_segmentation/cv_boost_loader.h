#pragma once
#include <opencv2/opencv.hpp>

#if CV_MAJOR_VERSION == 2
void loadBoost(CvBoost& boost, std::string const& filename);
#else
void loadBoost(cv::Ptr<cv::ml::Boost>& boost, std::string const& filename);
#endif
