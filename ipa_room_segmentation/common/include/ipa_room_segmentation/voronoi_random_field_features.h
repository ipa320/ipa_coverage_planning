//Header for featurecalculation
//number of Features that can be calculated: 23
#pragma once
#include <opencv/cv.h>
#include <queue>
#include <ipa_room_segmentation/contains.h>

//function to get the number of the features implemented
int getFeatureCount();
//function for calculating the feature
double getFeature(const std::vector<double>& beams, const std::vector<double>& angles,
		const std::vector<cv::Point>& clique_points, cv::Point point, int feature);
//feature 1: average difference between beamlenghts
double calcFeature1(const std::vector<double>& beams);
//feature 2: standard deviation of difference between beamlengths
double calcFeature2(const std::vector<double>& beams);
//feature 3: average difference between beamlengths, but cutting of too long beams
double calcFeature3(const std::vector<double>& beams, double maxlength);
//feature 4: standard deviation of difference between limited beamlenghts
double calcFeature4(const std::vector<double>& beams, double maxval);
//feature 5: average beamlength
double calcFeature5(const std::vector<double>& beams);
//feature 6: standard deviation of the beamlengths
double calcFeature6(const std::vector<double>& beams);
//feature 7: number of gaps between the beams (a gap is when the difference between the beamlenghts is larger than a threshold)
double calcFeature7(const std::vector<double>& beams);
//feature 8: distance between two Points corresponding to local minima of beamlengths
double calcFeature8(const std::vector<double>& beams, const std::vector<double>& angles);
//feature 9: angle between two Points corresponding to local minima of beamlenghts
double calcFeature9(const std::vector<double>& beams, const std::vector<double>& angles);
//feature 10: average of relation between two neighboring beams
double calcFeature10(const std::vector<double>& beams);
//feature 11: standard deviation of relation between two neighboring beams
double calcFeature11(const std::vector<double>& beams);
//feature 12: number of relative gaps, a relativ gap is when the relation of two negihboring beams is larger than a threshold
double calcFeature12(const std::vector<double>& beams);
//feature 13: Kurtosis = (Sum((x - mean)^4))/sigma^4) - 3, where mean is the mean-value and sigma is the standard deviation
double calcFeature13(const std::vector<double>& beams);
//feature 22: average of beam lengths / max. length
double calcFeature22(const std::vector<double>& beams);
//feature 23: standard deviation of the beam lengths divided by the maximal value
double calcFeature23(const std::vector<double>& beams);
//****************from now on Features that need a polygonal approximation of the beams*****************************
//calculate the polygonal approximation
std::vector<cv::Point> calcPolygonalApprox(const std::vector<double>&, const std::vector<double>&, cv::Point location);
//calcilate the centroid for each polygonal approximation
cv::Point calcCentroid(const std::vector<double>& beams, const std::vector<double>& angles, cv::Point location);
//feature 14: area of the polygonal approximation
double calcFeature14(const std::vector<double>& beams, const std::vector<double>& angles, cv::Point location);
//feature 15: perimeter of the polygonal approximation
double calcFeature15(const std::vector<double>& beams, const std::vector<double>& angles, cv::Point location);
//feature 16: area divided by perimeter of the polygonal approximation
double calcFeature16(const std::vector<double>& beams, const std::vector<double>& angles, cv::Point location);
//feature 17: average distance between centroid and the boundary of the polygonal approach
double calcFeature17(const std::vector<double>& beams, const std::vector<double>& angles, cv::Point location);
//feature 18: standard deviation of distance between centroid and the boundary of the polygonal approach
double calcFeature18(const std::vector<double>& beams, const std::vector<double>& angles, cv::Point location);
//feature 19: half the major axis of the ellipse that surrounds the polygon, calculated with openCV
double calcFeature19(const std::vector<double>& beams, const std::vector<double>& angles, cv::Point location);
//feature 20: half the minor axis of the ellipse that surrounds the polygon, calculated with openCV
double calcFeature20(const std::vector<double>& beams, const std::vector<double>& angles, cv::Point location);
//feature 21: major axis/minor axis
double calcFeature21(const std::vector<double>& beams, const std::vector<double>& angles, cv::Point location);
// feature 24: the curvature for a given clique
double calcFeature24(std::vector<cv::Point> clique_points);
// size of min. loop for one point by using breadth-first search
double getFeature26(const std::vector<cv::Point>& clique_points, const cv::Mat& voronoi_map);
