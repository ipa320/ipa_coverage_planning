#include <functional>

#include <ipa_room_segmentation/voronoi_random_field_segmentation.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <ros/package.h> // TODO: put everything in the segmentation_server


int main()
{
	const std::string package_path = ros::package::getPath("ipa_room_segmentation");
	cv::Mat map = cv::imread(package_path + "/common/files/test_maps/Freiburg79_scan.png", 0); // office_b.png  /home/rmb-fj/Pictures/map.png

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

	long double exponent = 200;
	long double res = exp(exponent);
	std::cout << res << std::endl;

//	uchar t = 127;
//	uint g = 127;

	std::vector<uint> possible_labels(3); // vector that stores the possible labels that are drawn in the training maps. Order: room - hallway - doorway
	possible_labels[0] = 77;
	possible_labels[1] = 115;
	possible_labels[2] = 179;

	// string that stores the path to the saving files
	std::string conditional_weights_path = package_path + "/common/files/training_results/conditional_field_weights.txt";
	std::string boost_file_path = package_path + "/common/files/training_results/";

	// load the training maps
	cv::Mat training_map;
	std::vector<cv::Mat> training_maps;
	training_map = cv::imread(package_path + "/common/files/training_maps/voronoi_random_field_training/training_maps/training_Fr52.png", 0);
	training_maps.push_back(training_map);
	training_map = cv::imread(package_path + "/common/files/training_maps/voronoi_random_field_training/training_maps/training_Fr101.png", 0);
	training_maps.push_back(training_map);
	training_map = cv::imread(package_path + "/common/files/training_maps/voronoi_random_field_training/training_maps/training_intel.png", 0);
	training_maps.push_back(training_map);
	training_map = cv::imread(package_path + "/common/files/training_maps/voronoi_random_field_training/training_maps/training_lab_d_furniture.png", 0);
	training_maps.push_back(training_map);
	training_map = cv::imread(package_path + "/common/files/training_maps/voronoi_random_field_training/training_maps/training_lab_ipa.png", 0);
	training_maps.push_back(training_map);
	training_map = cv::imread(package_path + "/common/files/training_maps/voronoi_random_field_training/training_maps/training_NLB_furniture.png", 0);
	training_maps.push_back(training_map);
//	training_map = cv::imread(package_path + "/common/files/training_maps/voronoi_random_field_training/training_maps/training_office_e.png", 0);
//	training_maps.push_back(training_map);
	// load the voronoi maps
	std::vector<cv::Mat> voronoi_maps;
	training_map = cv::imread(package_path + "/common/files/training_maps/voronoi_random_field_training/voronoi_maps/Fr52_voronoi.png", 0);
	voronoi_maps.push_back(training_map);
	training_map = cv::imread(package_path + "/common/files/training_maps/voronoi_random_field_training/voronoi_maps/Fr101_voronoi.png", 0);
	voronoi_maps.push_back(training_map);
	training_map = cv::imread(package_path + "/common/files/training_maps/voronoi_random_field_training/voronoi_maps/lab_intel_voronoi.png", 0);
	voronoi_maps.push_back(training_map);
	training_map = cv::imread(package_path + "/common/files/training_maps/voronoi_random_field_training/voronoi_maps/lab_d_furnitures_voronoi.png", 0);
	voronoi_maps.push_back(training_map);
	training_map = cv::imread(package_path + "/common/files/training_maps/voronoi_random_field_training/voronoi_maps/lab_ipa_voronoi.png", 0);
	voronoi_maps.push_back(training_map);
	training_map = cv::imread(package_path + "/common/files/training_maps/voronoi_random_field_training/voronoi_maps/NLB_voronoi.png", 0);
	voronoi_maps.push_back(training_map);
//	training_map = cv::imread(package_path + "/common/files/training_maps/voronoi_random_field_training/voronoi_maps/office_e_voronoi.png", 0);
//	voronoi_maps.push_back(training_map);
	// load the voronoi-nodes maps
	std::vector<cv::Mat> voronoi_node_maps;
	training_map = cv::imread(package_path + "/common/files/training_maps/voronoi_random_field_training/voronoi_node_maps/Fr52_voronoi_nodes.png", 0);
	voronoi_node_maps.push_back(training_map);
	training_map = cv::imread(package_path + "/common/files/training_maps/voronoi_random_field_training/voronoi_node_maps/Fr101_voronoi_nodes.png", 0);
	voronoi_node_maps.push_back(training_map);
	training_map = cv::imread(package_path + "/common/files/training_maps/voronoi_random_field_training/voronoi_node_maps/lab_intel_voronoi_nodes.png", 0);
	voronoi_node_maps.push_back(training_map);
	training_map = cv::imread(package_path + "/common/files/training_maps/voronoi_random_field_training/voronoi_node_maps/lab_d_furnitures_voronoi_nodes.png", 0);
	voronoi_node_maps.push_back(training_map);
	training_map = cv::imread(package_path + "/common/files/training_maps/voronoi_random_field_training/voronoi_node_maps/lab_ipa_voronoi_nodes.png", 0);
	voronoi_node_maps.push_back(training_map);
	training_map = cv::imread(package_path + "/common/files/training_maps/voronoi_random_field_training/voronoi_node_maps/NLB_voronoi_nodes.png", 0);
	voronoi_node_maps.push_back(training_map);
//	training_map = cv::imread(package_path + "/common/files/training_maps/voronoi_random_field_training/voronoi_node_maps/office_e_voronoi_nodes.png", 0);
//	voronoi_node_maps.push_back(training_map);
	// load the original maps
	std::vector<cv::Mat> original_maps;
	training_map = cv::imread(package_path + "/common/files/training_maps/voronoi_random_field_training/original_maps/Fr52_voronoi_nodes.png", 0);
	original_maps.push_back(training_map);
	training_map = cv::imread(package_path + "/common/files/training_maps/voronoi_random_field_training/original_maps/Fr101_voronoi_nodes.png", 0);
	original_maps.push_back(training_map);
	training_map = cv::imread(package_path + "/common/files/training_maps/voronoi_random_field_training/original_maps/lab_intel_voronoi_nodes.png", 0);
	original_maps.push_back(training_map);
	training_map = cv::imread(package_path + "/common/files/training_maps/voronoi_random_field_training/original_maps/lab_d_furnitures_voronoi_nodes.png", 0);
	original_maps.push_back(training_map);
	training_map = cv::imread(package_path + "/common/files/training_maps/voronoi_random_field_training/original_maps/lab_ipa_voronoi_nodes.png", 0);
	original_maps.push_back(training_map);
	training_map = cv::imread(package_path + "/common/files/training_maps/voronoi_random_field_training/original_maps/NLB_voronoi_nodes.png", 0);
	original_maps.push_back(training_map);
//	training_map = cv::imread(package_path + "/common/files/training_maps/voronoi_random_field_training/original_maps/office_e_voronoi_nodes.png", 0);
//	original_maps.push_back(training_map);

	std::cout << training_maps.size() << " " << voronoi_maps.size() << " " << voronoi_node_maps.size() << " " << original_maps.size() << std::endl;
//	for(size_t i = 0; i < training_maps.size(); ++i)
//	{
//		cv::imshow("training_map", training_maps[i]);
//		cv::imshow("voronoi_map", voronoi_maps[i]);
//		cv::imshow("nodes", voronoi_node_maps[i]);
//		cv::imshow("original_maps", original_maps[i]);
//		cv::waitKey();
//	}


	VoronoiRandomFieldSegmentation segmenter(false, false);

//	segmenter.trainAlgorithms(training_maps, voronoi_maps, voronoi_node_maps, original_maps, possible_labels, conditional_weights_path, boost_file_path);
//
	segmenter.segmentMap(map, 7, 50, 5, possible_labels, 7, true, conditional_weights_path, boost_file_path, 9000); // 7, 50, 4, 5

//	segmenter.testFunc(map, possible_labels, conditional_weights_path, boost_file_path);

//	size_t arr[] = {2};
//
//	std::cout << *arr << " " << *(arr+1) << std::endl;

//	std::vector<std::vector<double> > params(2);
//	params[0].push_back(5);
//	params[0].push_back(10);
//	params[0].push_back(2);
//	params[0].push_back(7);
//	params[1].push_back(5);
//	params[1].push_back(8);
//	params[1].push_back(4);
//	params[1].push_back(1);
//
//	std::vector<double> starting(2, 0.0);
//
//	column_vector r = segmenter.findMinValue(2, 3.0, params, starting);
//
//	std::cout << r << std::endl;

	return 0;
}
