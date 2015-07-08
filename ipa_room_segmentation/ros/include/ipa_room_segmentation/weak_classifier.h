#include <opencv/highgui.h>

#ifndef WEAK_CLASSIFIER_
#define WEAK_CLASSIFIER_

class WeakClassifier
{
public:
	//attributes
	double theta;
	std::vector<double> weights;
	int p;
	int feature;

	//methods
	//Constructor
	WeakClassifier();
	WeakClassifier(int feature, std::vector<double> weights);
	WeakClassifier(double theta, int p, int feature);
	void train(std::vector<int> labels, std::vector<cv::Point> points, std::vector<std::vector<double> > beams,
			std::vector<std::vector<double> > angles);
	int classify(std::vector<double> beams, std::vector<double> angles, cv::Point point);
	int classify(double feature_value);
};

#endif
