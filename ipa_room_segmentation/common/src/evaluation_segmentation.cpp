#include <ipa_room_segmentation/evaluation_segmentation.h>

#include <ros/ros.h>
#include <ros/package.h>

#include <map>
#include <vector>
#include <set>


//int pixel_number_calculation (cv::Mat matrix)
//{
//	int pixel_number;
//	for (unsigned int i=0; i<matrix.cols; i++)
//	{
//
//		for (unsigned int j=0; j<matrix.cols; j++)
//		{
//			if (matrix.at<uchar>(i,j) == 0)
//			{
//				++pixel_number;
//			}
//		}
//	}
//	return pixel_number;
//}
//struct byPixelsnumber
//{
//    bool operator () (const cv::Mat & a,const cv::Mat & b)
//    {
//    	int OccupiedPixelsnumber_a,OccupiedPixelsnumber_b = 0;
//    	for (unsigned int i=0; i<a.cols; i++)
//    	{
//    		for (unsigned int j=0; j<a.cols; j++)
//    		{
//    			if (a.at<uchar>(i,j) == 0)
//    			{
//    				++OccupiedPixelsnumber_a;
//    			}
//    		}
//    	}
//
//    	for (unsigned int i=0; i<b.cols; i++)
//    	{
//    		for (unsigned int j=0; j<b.cols; j++)
//    		{
//    			if (a.at<uchar>(i,j) == 0)
//    			{
//    				++OccupiedPixelsnumber_b;
//    			}
//    		}
//    	}
//
//         return OccupiedPixelsnumber_a > OccupiedPixelsnumber_b ;
//    }
//};


void EvaluationSegmentation::groundTruthVectorCalculation(const cv::Mat &bw_map, VectorOfPointSets& gt)
{
	gt.clear();

	cv::Mat label_image;
	bw_map.convertTo(label_image, CV_32SC1);

	int label_count = 2; // starts at 2 because 0,1 are used already

	for (int y = 0; y < label_image.rows; y++)
	{
		for (int x = 0; x < label_image.cols; x++)
		{
			if (bw_map.at<uchar>(y,x) != 255 || label_image.at<int>(y,x)!=255)
				continue;

			// fill each room area with a unique id
			cv::Rect rect;
			cv::floodFill(label_image, cv::Point(x,y), label_count, &rect, 0, 0, 8);

			// collect all pixels that belong to this room
			PointSet blob;
			for (int j = rect.y; j < (rect.y + rect.height); j++)
			{
				int* row = (int*)label_image.ptr(j);
				for (int i = rect.x; i < (rect.x + rect.width); i++)
				{
					if (row[i] != label_count)
						continue;
					blob.insert(cv::Point(i,j));
				}
			}
			gt.push_back(blob);

			label_count++;
		}
	}
}

void EvaluationSegmentation::computePrecisionRecall(const cv::Mat& gt_map, cv::Mat& gt_map_color, const cv::Mat& segmented_map,
		double& precision_micro, double& precision_macro, double& recall_micro, double& recall_macro, bool compute_gt_map_color)
{
	// create vector of rooms that contain a set of the room pixels from the ground truth map
	VectorOfPointSets gt_points_vector;	// room points: gt[room id][pixel index]
	if (compute_gt_map_color == true)
	{
		cv::Mat bw_map;
		cv::threshold(gt_map, bw_map, 250, 255, cv::THRESH_BINARY);
		//compute the ground truth matrix
		groundTruthVectorCalculation(bw_map, gt_points_vector);

		// generate colored ground truth map
		gt_map_color = cv::Mat::zeros(gt_map.size(), CV_8UC3);
		for(size_t i=0; i < gt_points_vector.size(); i++)
		{
			cv::Vec3b color(1 + rand()%255, 1 + rand()%255, 1 + rand()%255);
			for(PointSet::iterator it=gt_points_vector[i].begin(); it!=gt_points_vector[i].end(); it++)
				gt_map_color.at<cv::Vec3b>(*it) = color;
		}
	}
	else
	{
		// get point sets for segmentation map
		std::map<int, PointSet> gt_points_map;		// maps a label key identifier to a vector of points belonging to that room label
		const cv::Vec3b black(0,0,0);
		for (int v=0; v<gt_map_color.rows; ++v)
		{
			for (int u=0; u<gt_map_color.cols; ++u)
			{
				const cv::Vec3b& color = gt_map_color.at<cv::Vec3b>(v,u);
				if (color != black)
				{
					int key = color.val[0] + color.val[1]<<8 + color.val[2]<<16;
					gt_points_map[key].insert(cv::Point(u,v));
				}
			}
		}
		for (std::map<int, PointSet>::iterator it=gt_points_map.begin(); it!=gt_points_map.end(); ++it)
			if (it->second.size() > 100)
				gt_points_vector.push_back(it->second);
	}

	// remove mini rooms from gt
	for (VectorOfPointSets::iterator it=gt_points_vector.begin(); it!=gt_points_vector.end();)
	{
		if (it->size() <= 100)
			gt_points_vector.erase(it);
		else
			it++;
	}

	// get point sets for segmentation map
	std::map<int, PointSet> seg_points_map;		// maps a label key identifier to a vector of points belonging to that room label
	for (int v=0; v<segmented_map.rows; ++v)
	{
		for (int u=0; u<segmented_map.cols; ++u)
		{
			const int label = segmented_map.at<int>(v,u);
			if (label != 0)
				seg_points_map[label].insert(cv::Point(u,v));
		}
	}
	VectorOfPointSets seg_points_vector;
	for (std::map<int, PointSet>::iterator it=seg_points_map.begin(); it!=seg_points_map.end(); ++it)
		if (it->second.size() > 100)
			seg_points_vector.push_back(it->second);

	// set up matrix of overlaps: rows=seg_points_vector , cols = gt_points_vector
	cv::Mat overlap = cv::Mat::zeros(seg_points_vector.size(), gt_points_vector.size(), CV_64FC1);
	for (size_t v=0; v<seg_points_vector.size(); ++v)
	{
		for (size_t u=0; u<gt_points_vector.size(); ++u)
		{
			int overlapping = 0;
			for (PointSet::iterator it=seg_points_vector[v].begin(); it!=seg_points_vector[v].end(); it++)//       size_t pv=0; pv<seg_points_vector[v].size(); ++pv)
			{
				if (gt_points_vector[u].find(*it) != gt_points_vector[u].end())//         std::find(gt_points_vector[u].begin(), gt_points_vector[u].end(), seg_points_vector[v][pv]) != gt_points_vector[u].end())
					++overlapping;
			}
			overlap.at<double>(v,u) = overlapping;
		}
	}

	// precision
	precision_micro = 0.;	// average of individual precisions
	precision_macro = 0.;	// pixel count of all overlap areas / pixel count of all found segment areas
	double pdenominator_macro = 0.;
	for (size_t v=0; v<seg_points_vector.size(); ++v)
	{
		double max_overlap = 0.;
		for (size_t u=0; u<gt_points_vector.size(); ++u)
		{
			if (overlap.at<double>(v,u) > max_overlap)
				max_overlap = overlap.at<double>(v,u);
		}
		precision_micro += max_overlap / (double)seg_points_vector[v].size();
		precision_macro += max_overlap;
		pdenominator_macro += (double)seg_points_vector[v].size();
	}
	precision_micro /= (double)seg_points_vector.size();
	precision_macro /= pdenominator_macro;

	// recall
	recall_micro = 0.;	// average of individual recalls
	recall_macro = 0.;	// pixel count of all overlap areas / pixel count of all gt segment areas
	double rdenominator_macro = 0.;
	for (size_t u=0; u<gt_points_vector.size(); ++u)
	{
		double max_overlap = 0.;
		for (size_t v=0; v<seg_points_vector.size(); ++v)
		{
			if (overlap.at<double>(v,u) > max_overlap)
				max_overlap = overlap.at<double>(v,u);
		}
		recall_micro += max_overlap / (double)gt_points_vector[u].size();
		recall_macro += max_overlap;
		rdenominator_macro += (double)gt_points_vector[u].size();
	}
	recall_micro /= (double)gt_points_vector.size();
	recall_macro /= rdenominator_macro;
	//std::cout << recall_micro << "\t" << precision_micro << "\t" << recall_macro << "\t" << precision_macro << std::endl;
}

//void EvaluationSegmentation::Segmentation_Vector_calculation(const cv::Mat &segmented_map, std::map<cv::Point2i, cv::Vec3b> &segmented_room_mapping)
//{
//	for(unsigned int i=0; i <segmented_map.cols ; i++)
//	{
//		for(unsigned int j=0; j <segmented_map.rows ; j++)
//		{
//			if (segmented_map.at<cv::Vec3b>(i,j)[0] != 0 && segmented_map.at<cv::Vec3b>(i,j)[1] != 0 && segmented_map.at<cv::Vec3b>(i,j)[2] != 0)
//			{
//				segmented_room_mapping[cv::Point2i (i,j)] = segmented_map.at<cv::Vec3b>(i,j);
//			}
//		}
//	}
//}
//
//void EvaluationSegmentation::Recall_Precision_Calculation(std::map<cv::Point2i, cv::Point2f> &results, std::vector <cv::Mat> gt_mat_vector, std::vector <cv::Mat> segmentation_mat_vector)
//{
////	std::sort(gt_mat_vector.begin(), gt_mat_vector.end(), byPixelsnumber());
////	std::sort(segmentation_mat_vector.begin(), segmentation_mat_vector.end(), byPixelsnumber());
//
//	for (unsigned int i = 0; i < segmentation_mat_vector.size();i++)
//	{
//		int max_pixel_number = 0;
//		int pixel_number = 0;
//		int selected_gt_mat_pixel_number = 0;
//		for (unsigned int j = 0; j < gt_mat_vector.size();j++)
//		{
//			cv::Mat overlapped_matrix;
//			cv::bitwise_or(gt_mat_vector[i],segmentation_mat_vector[j],overlapped_matrix);
//
//			pixel_number = pixel_number_calculation(overlapped_matrix);
//
//			if (pixel_number > max_pixel_number)
//			{
//				max_pixel_number = pixel_number;
//				selected_gt_mat_pixel_number = pixel_number_calculation(gt_mat_vector[j]);
//			}
//		}
//
//		float recall = float(max_pixel_number/selected_gt_mat_pixel_number);
//		float precision = float(max_pixel_number/ (pixel_number_calculation(segmentation_mat_vector[i])));
//
//		// calculate the first pixel of segmentation matrix
//		for (unsigned int m=0; m<segmentation_mat_vector[i]; m++)
//		{
//			for (unsigned int n=0; n<segmentation_mat_vector[i]; n++)
//			{
//				if (segmentation_mat_vector[i].at<uchar>(m,n) == 0)
//				{
//					results[cv::Point2i(m,n)] = cv::Point2f (recall,precision);
//					break;
//				}
//			}
//		}
//	}
//}

/*
int main(int argc, char** argv)
{
	ros::init(argc, argv, "evaluation_seg");
	ros::NodeHandle n;

//	if (argc < 2)
//	{
//		std::cout << "error: not enough input parameters!" << std::endl;
//		return -1;
//	}

	std::vector< std::string > map_names;
//	map_names.push_back("lab_ipa");
//	map_names.push_back("lab_c_scan");
//	map_names.push_back("Freiburg52_scan");
//	map_names.push_back("Freiburg79_scan");
//	map_names.push_back("lab_b_scan");
//	map_names.push_back("lab_intel");
//	map_names.push_back("Freiburg101_scan");
//	map_names.push_back("lab_d_scan");
//	map_names.push_back("lab_f_scan");
//	map_names.push_back("lab_a_scan");
//	map_names.push_back("NLB");
//	map_names.push_back("office_a");
//	map_names.push_back("office_b");
//	map_names.push_back("office_c");
//	map_names.push_back("office_d");
//	map_names.push_back("office_e");
//	map_names.push_back("office_f");
//	map_names.push_back("office_g");
//	map_names.push_back("office_h");
//	map_names.push_back("office_i");
	map_names.push_back("lab_ipa_furnitures");
	map_names.push_back("lab_c_scan_furnitures");
	map_names.push_back("Freiburg52_scan_furnitures");
	map_names.push_back("Freiburg79_scan_furnitures");
	map_names.push_back("lab_b_scan_furnitures");
	map_names.push_back("lab_intel_furnitures");
	map_names.push_back("Freiburg101_scan_furnitures");
	map_names.push_back("lab_d_scan_furnitures");
	map_names.push_back("lab_f_scan_furnitures");
	map_names.push_back("lab_a_scan_furnitures");
	map_names.push_back("NLB_furnitures");
	map_names.push_back("office_a_furnitures");
	map_names.push_back("office_b_furnitures");
	map_names.push_back("office_c_furnitures");
	map_names.push_back("office_d_furnitures");
	map_names.push_back("office_e_furnitures");
	map_names.push_back("office_f_furnitures");
	map_names.push_back("office_g_furnitures");
	map_names.push_back("office_h_furnitures");
	map_names.push_back("office_i_furnitures");

	const std::string segmented_map_path = "room_segmentation/"; //ros::package::getPath("ipa_room_segmentation") + "/common/files/segmented_maps/";

	for (size_t image_index = 0; image_index<map_names.size(); ++image_index)
	{
		// load map
		std::string map_name = map_names[image_index];
		std::string seg_image_filename = segmented_map_path + map_name + "_segmented_4semantic.png";
		std::size_t pos = map_name.find("_furnitures");
		if (pos != std::string::npos)
			map_name = map_name.substr(0, pos);
		std::string gt_image_filename = ros::package::getPath("ipa_room_segmentation") + "/common/files/test_maps/" + map_name + "_gt_segmentation.png";
		//std::cout << gt_image_filename << "\n" << seg_image_filename << std::endl;
		cv::Mat gt_map = cv::imread(gt_image_filename.c_str(),CV_8U);
		cv::Mat segmented_map = cv::imread(seg_image_filename.c_str());
		cv::Mat segmented_map_int = cv::Mat::zeros(segmented_map.rows, segmented_map.cols, CV_32SC1);
		for (int v=0; v<segmented_map.rows; ++v)
		{
			for (int u=0; u<segmented_map.cols; ++u)
			{
				const cv::Vec3b& color = segmented_map.at<cv::Vec3b>(v,u);
				segmented_map_int.at<int>(v,u) = color.val[0] + color.val[1]<<8 + color.val[2]<<16;
			}
		}

		double precision_micro, precision_macro, recall_micro, recall_macro;
		cv::Mat gt_map_color;
		EvaluationSegmentation es;
		es.computePrecisionRecall(gt_map, gt_map_color, segmented_map, precision_micro, precision_macro, recall_micro, recall_macro, true);
		std::cout << recall_micro << "\t" << precision_micro << "\t" << recall_macro << "\t" << precision_macro << std::endl;
		std::string gt_image_filename_color = segmented_map_path + map_name + "_gt_color_segmentation.png"; //ros::package::getPath("ipa_room_segmentation") + "/common/files/test_maps/" + map_name + "_gt_color_segmentation.png";
		cv::imwrite(gt_image_filename_color.c_str(), gt_map_color);
	}
//
//	cv::Mat gt_map = cv::imread(argv[1],CV_8U);
//	cv::Mat segmented_map = cv::imread(argv[2]);
//
//	EvaluationSegmentation es;
//
//	cv::Mat bw_map;
//	cv::threshold(gt_map, bw_map, 250, 255, cv::THRESH_BINARY);
//	//compute the GT matrix
//	std::vector < std::vector<cv::Point> > gt_points_vector;
//	es.GT_Vector_calculation(bw_map, gt_points_vector);
//
//	cv::Mat GT_matrix = cv::Mat::zeros(gt_map.size(), CV_8UC3);
//	for(size_t i=0; i < gt_points_vector.size(); i++)
//	{
//		unsigned char r = 255 * (rand()/(1.0 + RAND_MAX));
//		unsigned char g = 255 * (rand()/(1.0 + RAND_MAX));
//		unsigned char b = 255 * (rand()/(1.0 + RAND_MAX));
//
//		for(size_t j=0; j < gt_points_vector[i].size(); j++)
//		{
//			int x = gt_points_vector[i][j].x;
//			int y = gt_points_vector[i][j].y;
//
//			GT_matrix.at<cv::Vec3b>(y,x)[0] = b;
//			GT_matrix.at<cv::Vec3b>(y,x)[1] = g;
//			GT_matrix.at<cv::Vec3b>(y,x)[2] = r;
//		}
//	}
//	cv::imshow("labeled", GT_matrix);
//	cv::waitKey(0);
//
//	// save ground truth and segmentation into maps
//	std::map<cv::Point2i, cv::Vec3b> segmented_room_mapping, GT_room_mapping;
//	std::vector <cv::Mat> gt_mat_vector, segmentation_mat_vector;
//	es.Segmentation_Vector_calculation(GT_matrix, GT_room_mapping);
//	es.Segmentation_Vector_calculation(segmented_map, segmented_room_mapping);
//
//	for(std::map<cv::Point2i, cv::Vec3b>::iterator iterator = GT_room_mapping.begin(); iterator != GT_room_mapping.end(); iterator++)
//	{
////todo: convert map<cv::Point2i, cv::Vec3b> to vector<Mat>
//	}
//
//	std::map<cv::Point2i, cv::Point2f> results;
//
////	cv::namedWindow("labeled");
////	cv::imshow("labeled", GT_matrix);
////	cv::waitKey(0);


	return 0;
}
*/
