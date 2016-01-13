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

	segmenter.segmentMap(map, 7, 50, 5, 7, true); // 7, 50, 4, 5

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
