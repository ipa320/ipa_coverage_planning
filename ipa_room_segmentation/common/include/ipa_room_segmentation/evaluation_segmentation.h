
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <iostream>
#include <vector>
#include <math.h>
#include <map>

#include <ctime>


class EvaluationSegmentation
{
private:
	// fills each closed area (= ground truth room) with a unique id and collects all room points in gt[room id][pixel index]
	void groundTruthVectorCalculation(const cv::Mat &bw_map, std::vector < std::vector<cv::Point2i> > &gt);

//	void Segmentation_Vector_calculation(const cv::Mat &segmented_map, std::map<cv::Point2i, cv::Vec3b> &segmented_room_mapping);
//	void Recall_Precision_Calculation(std::map<cv::Point2i, cv::Point2f> &results, std::vector <cv::Mat> gt_mat_vector, std::vector <cv::Mat> segmentation_mat_vector);

public:

	// gt_map = ground truth gray scale map (>250 = free space), each room is a closed contour (rooms are separated by drawn in borders), no colors
	// gt_map_color = colored ground truth map (maybe computed by or provided to the function depending on value of compute_gt_map_color)
	// segmented_map in CV_32SC1
	void computePrecisionRecall(const cv::Mat& gt_map, cv::Mat& gt_map_color, const cv::Mat& segmented_map,
			double& precision_micro, double& precision_macro, double& recall_micro, double& recall_macro, bool compute_gt_map_color=true);
};
