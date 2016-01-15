#include <functional>

#include <ipa_room_segmentation/voronoi_random_field_segmentation.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>


int main()
{
	cv::Mat map = cv::imread("/home/rmb-fj/Pictures/map.png", 0); // office_b.png

	for(unsigned int u = 0; u < map.rows; ++u)
	{
		for(unsigned int v = 0; v < map.cols; ++v)
		{
			if(map.at<unsigned char>(u,v) < 250)
			{
				map.at<unsigned char>(u,v) = 0;
			}
			else
			{
				map.at<unsigned char>(u,v) = 255;
			}
		}
	}

	VoronoiRandomFieldSegmentation segmenter(false);

//	segmenter.segmentMap(map, 7, 50, 5, 7, true); // 7, 50, 4, 5


	std::vector<std::vector<double> > d(4);
	std::vector<double> a (4, 100);
	std::vector<double> b (4, 200);

	std::vector<double> c = a + b; // operator overload in voronoi_random_field_segmentation.h
	d[0] = d[0] + a; // TODO: overload operator +=
	d[0] = d[0] + b;

	for(size_t i = 0; i < d.size(); ++i)
		for(size_t j = 0; j < d[i].size(); ++j)
			std::cout << d[i][j] << std::endl;


//	CvBoost boost;
//	boost.load("/home/rmb-fj/git/care-o-bot-indigo/src/autopnp/ipa_room_segmentation/common/files/training_results/trained_room_boost.xml");
//
//	std::vector<double> angles_for_simulation_;
//	for (double angle = 0; angle < 360; angle++)
//	{
//		angles_for_simulation_.push_back(angle);
//	}
//
//	std::vector<cv::Point> clique_members;
//
//	std::vector<double> temporary_beams = raycasting(map, cv::Point(350,350));
//
//	std::cout << "map value: " << (double) map.at<unsigned char>(350,350) << std::endl;
//
//	cv::Mat featuresMat(1, getFeatureCount()-1, CV_32FC1); //OpenCV expects a 32-floating-point Matrix as feature input
//	for (int f = 1; f <= getFeatureCount()-1; ++f)
//	{
//		//get the features for each room and put it in the featuresMat
//		featuresMat.at<float>(0, f - 1) = (float) getFeature(temporary_beams, angles_for_simulation_, clique_members, cv::Point(350,350), f);
//	}
//
//	CvMat features = featuresMat;									// Wanted from OpenCV to get the weak hypothesis from the
//	CvMat weak_hypothesis = cv::Mat(1, boost.get_weak_predictors()->total, CV_32F);	// separate weak classifiers.
//
//	std::cout << "weak: " << weak_hypothesis.cols << " features: " << features.cols << std::endl;
//
//	float sum = boost.predict(&features, 0, &weak_hypothesis, CV_WHOLE_SEQ, false, true);
//
//	float test_sum = 0;
//
//	for(size_t f = 0; f < weak_hypothesis.cols; ++f)
//	{
//		std::cout << CV_MAT_ELEM(weak_hypothesis, float, 0, f) << " " << f << std::endl;
//		test_sum += CV_MAT_ELEM(weak_hypothesis, float, 0, f);
//	}
//
//	std::cout << "complete sum: " << test_sum << " real sum: " << sum << std::endl;


//	cv::Mat ger(1,2,CV_32F);
//
//	ger.at<float>(0,0) = 2.3;
//	ger.at<float>(0,1) = 3.5;
//
//	CvMat test = ger;
//
//	float pixval = CV_MAT_ELEM(test, float, 0, 1);
//
//	std::cout << "normal Mat: " << (double) ger.at<float>(0, 1) << " new Mat: " << (double) pixval << std::endl;

//	std::vector<cv::Point> testpoints;
//	testpoints.push_back(cv::Point(200, 205));
//	testpoints.push_back(cv::Point(210, 205));
//	testpoints.push_back(cv::Point(200, 300));
//
//	cv::Point2f center;
//	float radius;
//
//	cv::minEnclosingCircle(testpoints, center, radius);
//
//    cv::circle(map, center, radius, cv::Scalar(127), CV_FILLED);
//    cv::circle(map, testpoints[0], 1, cv::Scalar(100), CV_FILLED);
//    cv::circle(map, testpoints[1], 1, cv::Scalar(100), CV_FILLED);
//    cv::circle(map, testpoints[2], 1, cv::Scalar(100), CV_FILLED);
//    cv::imshow("ellipse", map);
//    cv::imwrite("/home/rmb-fj/Pictures/ellipse.png", map);
//    cv::waitKey();




//	for(unsigned int i = 1; i < 4; ++i)
//	{
//		std::cout << i << " ";
//		if((i+1)%4 == 0)
//		{
//			std::cout << 1 << std::endl;
//		}
//		else
//		{
//			std::cout << i+1 << std::endl;
//		}
//	}

	return 0;
}
