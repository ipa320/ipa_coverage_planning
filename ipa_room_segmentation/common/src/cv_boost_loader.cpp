#include <ipa_room_segmentation/cv_boost_loader.h>

#if CV_MAJOR_VERSION == 2
void loadBoost(CvBoost& boost, std::string const& filename)
{
	boost.load(filename.c_str());
}
#else
void loadBoost(cv::Ptr<cv::ml::Boost>& boost, std::string const& filename)
{
#if CV_MAJOR_VERSION == 3 && CV_MINOR_VERSION<=2
	boost = cv::Algorithm::load<cv::ml::Boost>(filename.c_str());
#else
	boost = cv::ml::Boost::load(filename.c_str());
#endif
}
#endif
