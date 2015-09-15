#include <ipa_room_segmentation/evaluation_segmentation.h>

int pixel_number_calculation (cv::Mat matrix)
{
	int pixel_number;
	for (unsigned int i=0; i<matrix.cols; i++)
	{

		for (unsigned int j=0; j<matrix.cols; j++)
		{
			if (matrix.at<uchar>(i,j) == 0)
			{
				++pixel_number;
			}
		}
	}
	return pixel_number;
}
struct byPixelsnumber
{
    bool operator () (const cv::Mat & a,const cv::Mat & b)
    {
    	int OccupiedPixelsnumber_a,OccupiedPixelsnumber_b = 0;
    	for (unsigned int i=0; i<a.cols; i++)
    	{
    		for (unsigned int j=0; j<a.cols; j++)
    		{
    			if (a.at<uchar>(i,j) == 0)
    			{
    				++OccupiedPixelsnumber_a;
    			}
    		}
    	}

    	for (unsigned int i=0; i<b.cols; i++)
    	{
    		for (unsigned int j=0; j<b.cols; j++)
    		{
    			if (a.at<uchar>(i,j) == 0)
    			{
    				++OccupiedPixelsnumber_b;
    			}
    		}
    	}

         return OccupiedPixelsnumber_a > OccupiedPixelsnumber_b ;
    }
};

void EvaluationSegmentation::GT_Vector_calculation(const cv::Mat &bw_map, std::vector < std::vector<cv::Point2i> > &gt)
{
    gt.clear();

    cv::Mat label_image;
    bw_map.convertTo(label_image, CV_32SC1);

    int label_count = 2; // starts at 2 because 0,1 are used already

    for(int y=0; y < label_image.rows; y++) {
        int *row = (int*)label_image.ptr(y);
        for(int x=0; x < label_image.cols; x++) {
            if(row[x] != 255) {
                continue;
            }

            cv::Rect rect;
            cv::floodFill(label_image, cv::Point(x,y), label_count, &rect, 0, 0, 8);

            std::vector <cv::Point2i> blob;

            for(int i=rect.y; i < (rect.y+rect.height); i++) {
                int *row2 = (int*)label_image.ptr(i);
                for(int j=rect.x; j < (rect.x+rect.width); j++) {
                    if(row2[j] != label_count) {
                        continue;
                    }

                    blob.push_back(cv::Point2i(j,i));
                }
            }

            gt.push_back(blob);

            label_count++;
        }
    }
}

void EvaluationSegmentation::Segmentation_Vector_calculation(const cv::Mat &segmented_map, std::map<cv::Point2i, cv::Vec3b> &segmented_room_mapping)
{
	for(unsigned int i=0; i <segmented_map.cols ; i++)
	{
		for(unsigned int j=0; j <segmented_map.rows ; j++)
		{
			if (segmented_map.at<cv::Vec3b>(i,j)[0] != 0 && segmented_map.at<cv::Vec3b>(i,j)[1] != 0 && segmented_map.at<cv::Vec3b>(i,j)[2] != 0)
			{
				segmented_room_mapping[cv::Point2i (i,j)] = segmented_map.at<cv::Vec3b>(i,j);
			}
		}
	}
}

void EvaluationSegmentation::Recall_Precision_Calculation(std::map<cv::Point2i, cv::Point2f> &results, std::vector <cv::Mat> gt_mat_vector, std::vector <cv::Mat> segmentation_mat_vector)
{
//	std::sort(gt_mat_vector.begin(), gt_mat_vector.end(), byPixelsnumber());
//	std::sort(segmentation_mat_vector.begin(), segmentation_mat_vector.end(), byPixelsnumber());

	for (unsigned int i = 0; i < segmentation_mat_vector.size();i++)
	{
		int max_pixel_number = 0;
		int pixel_number = 0;
		int selected_gt_mat_pixel_number = 0;
		for (unsigned int j = 0; j < gt_mat_vector.size();j++)
		{
			cv::Mat overlapped_matrix;
			cv::bitwise_or(gt_mat_vector[i],segmentation_mat_vector[j],overlapped_matrix);

			pixel_number = pixel_number_calculation(overlapped_matrix);

			if (pixel_number > max_pixel_number)
			{
				max_pixel_number = pixel_number;
				selected_gt_mat_pixel_number = pixel_number_calculation(gt_mat_vector[j]);
			}
		}

		float recall = float(max_pixel_number/selected_gt_mat_pixel_number);
		float precision = float(max_pixel_number/ (pixel_number_calculation(segmentation_mat_vector[i])));

		// calculate the first pixel of segmentation matrix
		for (unsigned int m=0; m<segmentation_mat_vector[i]; m++)
		{
			for (unsigned int n=0; n<segmentation_mat_vector[i]; n++)
			{
				if (segmentation_mat_vector[i].at<uchar>(m,n) == 0)
				{
					results[cv::Point2i(m,n)] = cv::Point2f (recall,precision);
					break;
				}
			}
		}
	}
}

int main(int argc, char** argv)

{
	if (argc < 2)
	{
		std::cout << "error: not enough input parameters!" << std::endl;
		return -1;
	}

	cv::Mat gt_map = cv::imread(argv[1],CV_8U);
	cv::Mat segmented_map = cv::imread(argv[2]);

	EvaluationSegmentation EvaluationSegmentation;

	cv::Mat GT_matrix = cv::Mat::zeros(gt_map.size(), CV_8UC3);

	cv::Mat bw_map;
	std::vector < std::vector<cv::Point2i> > gt_points_vector;
	std::map<cv::Point2i, cv::Vec3b> segmented_room_mapping, GT_room_mapping;

	std::vector <cv::Mat> gt_mat_vector, segmentation_mat_vector;

	cv::threshold(gt_map, bw_map, 253, 255, cv::THRESH_BINARY);
	//compute the GT matrix
	EvaluationSegmentation.GT_Vector_calculation(bw_map, gt_points_vector);

	for(size_t i=0; i < gt_points_vector.size(); i++)
	{
		unsigned char r = 255 * (rand()/(1.0 + RAND_MAX));
		unsigned char g = 255 * (rand()/(1.0 + RAND_MAX));
		unsigned char b = 255 * (rand()/(1.0 + RAND_MAX));

		for(size_t j=0; j < gt_points_vector[i].size(); j++)
		{
			int x = gt_points_vector[i][j].x;
			int y = gt_points_vector[i][j].y;

			GT_matrix.at<cv::Vec3b>(y,x)[0] = b;
			GT_matrix.at<cv::Vec3b>(y,x)[1] = g;
			GT_matrix.at<cv::Vec3b>(y,x)[2] = r;
		}
	}

	// save ground truth and segmentation into maps
	EvaluationSegmentation.Segmentation_Vector_calculation(GT_matrix, GT_room_mapping);
	EvaluationSegmentation.Segmentation_Vector_calculation(segmented_map, segmented_room_mapping);

	for(std::map<cv::Point2i, cv::Vec3b>::iterator iterator = GT_room_mapping.begin(); iterator != GT_room_mapping.end(); iterator++)
	{
//todo: convert map<cv::Point2i, cv::Vec3b> to vector<Mat>
	}

	std::map<cv::Point2i, cv::Point2f> results;

//	cv::namedWindow("labeled");
//	cv::imshow("labeled", GT_matrix);
//	cv::waitKey(0);


	return 0;
	}

