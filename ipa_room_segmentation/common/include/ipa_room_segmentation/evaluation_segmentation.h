
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <iostream>
#include <vector>
#include <math.h>
#include <map>

#include <ctime>

class EvaluationSegmentation
{
public:
	void GT_Vector_calculation(const cv::Mat &bw_map, std::vector < std::vector<cv::Point2i> > &gt);
	void Segmentation_Vector_calculation(const cv::Mat &segmented_map, std::map<cv::Point2i, cv::Vec3b> &segmented_room_mapping);
	void Recall_Precision_Calculation(std::map<cv::Point2i, cv::Point2f> &results, std::vector <cv::Mat> gt_mat_vector, std::vector <cv::Mat> segmentation_mat_vector);
};
