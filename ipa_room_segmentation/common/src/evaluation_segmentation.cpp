#include <ipa_room_segmentation/evaluation_segmentation.h>



std::vector<cv::Mat> EvaluationSegmentation::gt_calculation(cv::Mat& map)
{
	cv::Mat bw_map, gray;
	std::vector<cv::Mat> gt;

//	double CannyAccThresh = cv::threshold(map,gray,0,255,CV_THRESH_BINARY|CV_THRESH_OTSU);
//	cv::Canny(map,bw_map,0,CannyAccThresh*1);
	cv::threshold(map, bw_map, 253, 255, cv::THRESH_BINARY);
	cv::imshow( "bw", bw_map );
	std::vector<std::vector<cv::Point> > contours;
	cv::vector<cv::Vec4i> hierarchy;
	cv::findContours( bw_map, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_NONE, cv::Point(0, 0) );
	cv::Mat drawing = cv::Mat::zeros( bw_map.size(), CV_8UC3 );

	for (unsigned int i = 0; i < contours.size(); i++)
	{
		double contour_area = std::fabs(cv::contourArea(contours[i]));

		if (contour_area > (map.cols*map.rows)/100. && contour_area < 0.7*(map.cols*map.rows) )
		{
			std::cout<<"area: "<<contour_area<<std::endl;
			std::cout<<"(map.cols*map.rows)/100.: "<<(map.cols*map.rows)/100.<<std::endl;
			std::cout<<"0.8*(map.cols*map.rows): "<<0.8*(map.cols*map.rows)<<std::endl;
			cv::namedWindow( "Contours", CV_WINDOW_AUTOSIZE );

			cv::Scalar color = cv::Scalar(255,0,255);
			cv::drawContours( drawing, contours, i, color, 2, 8, hierarchy, 0, cv::Point() );
			cv::imshow( "Contours", drawing );
			cv::waitKey();
		}
	}

	return gt;

}

std::vector<cv::Mat> EvaluationSegmentation::calculate_white_pixels(cv::Mat& map)
{
	std::vector<cv::Mat> gt;

	return gt;
}

int main(int argc, char** argv)

{
	if (argc < 2)
	{
		std::cout << "error: not enough input parameters!" << std::endl;
		return -1;
	}

	cv::Mat gt_map = cv::imread(argv[1],CV_8U);
	cv::Mat segmented_map = cv::imread(argv[2],CV_8U);

	EvaluationSegmentation EvaluationSegmentation;
	std::vector<cv::Mat> gt;

	gt = EvaluationSegmentation.gt_calculation(gt_map);


}
