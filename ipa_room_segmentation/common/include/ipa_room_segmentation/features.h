//Header for featurecalculation
//number of Features that can be calculated: 23
#pragma once

#include <opencv2/opencv.hpp>


class LaserScannerFeatures
{
public:
	//function to get the number of the features implemented
	int get_feature_count();
	// reset cached data
	void resetCachedData();
	//function for calculating the feature
	double get_feature(const std::vector<double>& beams, const std::vector<double>& angles, cv::Point point, int feature);
	void get_features(const std::vector<double>& beams, const std::vector<double>& angles, cv::Point point, cv::Mat& features);
	//feature 1: average difference between beamlenghts
	double calc_feature1(const std::vector<double>& beams);
	//feature 2: standard deviation of difference between beamlengths
	double calc_feature2(const std::vector<double>& beams);
	//feature 3: average difference between beamlengths, but cutting of too long beams
	double calc_feature3(const std::vector<double>& beams, double maxlength);
	//feature 4: standard deviation of difference between limited beamlenghts
	double calc_feature4(const std::vector<double>& beams, double maxval);
	//feature 5: average beamlength
	double calc_feature5(const std::vector<double>& beams);
	//feature 6: standard deviation of the beamlengths
	double calc_feature6(const std::vector<double>& beams);
	//feature 7: number of gaps between the beams (a gap is when the difference between the beamlenghts is larger than a threshold)
	double calc_feature7(const std::vector<double>& beams);
	//feature 8: distance between two Points corresponding to local minima of beamlengths
	double calc_feature8(const std::vector<double>& beams, const std::vector<double>& angles);
	//feature 9: angle between two Points corresponding to local minima of beamlenghts
	double calc_feature9(const std::vector<double>& beams, const std::vector<double>& angles);
	//feature 10: average of relation between two neighboring beams
	double calc_feature10(const std::vector<double>& beams);
	//feature 11: standard deviation of relation between two neighboring beams
	double calc_feature11(const std::vector<double>& beams);
	//feature 12: number of relative gaps, a relativ gap is when the relation of two negihboring beams is larger than a threshold
	double calc_feature12(const std::vector<double>& beams);
	//feature 13: Kurtosis = (Sum((x - mean)^4))/sigma^4) - 3, where mean is the mean-value and sigma is the standard deviation
	double calc_feature13(const std::vector<double>& beams);
	//feature 22: average of beam lengths / max. length
	double calc_feature22(const std::vector<double>& beams);
	//feature 23: standard deviation of the beam lengths divided by the maximal value
	double calc_feature23(const std::vector<double>& beams);
	//****************from now on Features that need a polygonal approximation of the beams*****************************
	//calculate the polygonal approximation
	std::vector<cv::Point> calc_polygonal_approx(const std::vector<double>&, const std::vector<double>&, cv::Point location);
	//calcilate the centroid for each polygonal approximation
	cv::Point calc_centroid(const std::vector<double>& beams, const std::vector<double>& angles, cv::Point location);
	//feature 14: area of the polygonal approximation
	double calc_feature14(const std::vector<double>& beams, const std::vector<double>& angles, cv::Point location);
	//feature 15: perimeter of the polygonal approximation
	double calc_feature15(const std::vector<double>& beams, const std::vector<double>& angles, cv::Point location);
	//feature 16: area divided by perimeter of the polygonal approximation
	double calc_feature16(const std::vector<double>& beams, const std::vector<double>& angles, cv::Point location);
	//feature 17: average distance between centroid and the boundary of the polygonal approach
	double calc_feature17(const std::vector<double>& beams, const std::vector<double>& angles, cv::Point location);
	//feature 18: standard deviation of distance between centroid and the boundary of the polygonal approach
	double calc_feature18(const std::vector<double>& beams, const std::vector<double>& angles, cv::Point location);
	//feature 19: half the major axis of the ellipse that surrounds the polygon, calculated with openCV
	double calc_feature19(const std::vector<double>& beams, const std::vector<double>& angles, cv::Point location);
	//feature 20: half the minor axis of the ellipse that surrounds the polygon, calculated with openCV
	double calc_feature20(const std::vector<double>& beams, const std::vector<double>& angles, cv::Point location);
	//feature 21: major axis/minor axis
	double calc_feature21(const std::vector<double>& beams, const std::vector<double>& angles, cv::Point location);

private:

	std::vector<double> features_;
	std::vector<bool> features_computed_;

	std::vector<cv::Point> polygon_;
	bool polygon_computed_;

	cv::Point centroid_;
	bool centroid_computed_;

};
