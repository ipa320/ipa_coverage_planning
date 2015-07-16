//Header for featurecalculation
//number of Features that can be calculated: 23
#ifndef FEATURES
#define FEATURES
#include <opencv/cv.h>

//function to get the number of the features implemented
int get_feature_count();
//function for calculating the feature
double get_feature(std::vector<double> beams, std::vector<double> angles, cv::Point point, int feature);
//feature 1: average difference between beamlenghts
double calc_feature1(std::vector<double> beams);
//feature 2: standard deviation of difference between beamlengths
double calc_feature2(std::vector<double> beams);
//feature 3: average difference between beamlengths, but cutting of too long beams
double calc_feature3(std::vector<double> beams, double maxlength);
//feature 4: standard deviation of difference between limited beamlenghts
double calc_feature4(std::vector<double> beams, double maxval);
//feature 5: average beamlength
double calc_feature5(std::vector<double> beams);
//feature 6: standard deviation of the beamlengths
double calc_feature6(std::vector<double> beams);
//feature 7: number of gaps between the beams (a gap is when the difference between the beamlenghts is larger than a threshold)
double calc_feature7(std::vector<double> beams);
//feature 8: distance between two Points corresponding to local minima of beamlengths
double calc_feature8(std::vector<double> beams, std::vector<double> angles);
//feature 9: angle between two Points corresponding to local minima of beamlenghts
double calc_feature9(std::vector<double> beams, std::vector<double> angles);
//feature 10: average of relation between two neighboring beams
double calc_feature10(std::vector<double> beams);
//feature 11: standard deviation of relation between two neighboring beams
double calc_feature11(std::vector<double> beams);
//feature 12: number of relative gaps, a relativ gap is when the relation of two negihboring beams is larger than a threshold
double calc_feature12(std::vector<double> beams);
//feature 13: Kurtosis = (Sum((x - mean)^4))/sigma^4) - 3, where mean is the mean-value and sigma is the standard deviation
double calc_feature13(std::vector<double> beams);
//feature 22: average of beam lengths / max. length
double calc_feature22(std::vector<double> beams);
//feature 23: standard deviation of the beam lengths divided by the maximal value
double calc_feature23(std::vector<double> beams);
//****************from now on Features that need a polygonal approximation of the beams*****************************
//calculate the polygonal approximation
std::vector<cv::Point> calc_polygonal_approx(std::vector<double> beams, std::vector<double> angles, cv::Point location);
//calcilate the centroid for each polygonal approximation
cv::Point calc_centroid(std::vector<double> beams, std::vector<double> angles, cv::Point location);
//feature 14: area of the polygonal approximation
double calc_feature14(std::vector<double> beams, std::vector<double> angles, cv::Point location);
//feature 15: perimeter of the polygonal approximation
double calc_feature15(std::vector<double> beams, std::vector<double> angles, cv::Point location);
//feature 16: area divided by perimeter of the polygonal approximation
double calc_feature16(std::vector<double> beams, std::vector<double> angles, cv::Point location);
//feature 17: average distance between centroid and the boundary of the polygonal approach
double calc_feature17(std::vector<double> beams, std::vector<double> angles, cv::Point location);
//feature 18: standard deviation of distance between centroid and the boundary of the polygonal approach
double calc_feature18(std::vector<double> beams, std::vector<double> angles, cv::Point location);
//feature 19: half the major axis of the ellipse that surrounds the polygon, calculated with openCV
double calc_feature19(std::vector<double> beams, std::vector<double> angles, cv::Point location);
//feature 20: half the minor axis of the ellipse that surrounds the polygon, calculated with openCV
double calc_feature20(std::vector<double> beams, std::vector<double> angles, cv::Point location);
//feature 21: major axis/minor axis
double calc_feature21(std::vector<double> beams, std::vector<double> angles, cv::Point location);

#endif
