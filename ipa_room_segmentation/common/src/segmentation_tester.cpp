#include <functional>

#include <ipa_room_segmentation/voronoi_random_field_segmentation.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <ros/package.h>


int main()
{
	std::string package_path = ros::package::getPath("ipa_room_segmentation");
	std::string map_path = package_path + "/common/files/test_maps/";

	std::vector<std::string> map_names;
//	map_names.push_back("office_b.png");
	map_names.push_back("office_e.png");
//	map_names.push_back("NLB.png");
	map_names.push_back("lab_ipa.png");
	map_names.push_back("NLB_furnitures.png");
//	map_names.push_back("office_e_furnitures.png");
//	map_names.push_back("lab_c_scan_furnitures.png");

	std::vector<cv::Mat> maps(map_names.size());

	for(size_t i = 0; i < map_names.size(); ++i)
	{
		cv::Mat map = cv::imread(map_path + map_names[i], 0);

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

//		cv::imshow("test", map);
//		cv::waitKey();

		maps[i] = map.clone();
	}

	std::vector<uint> possible_labels(3); // vector that stores the possible labels that are drawn in the training maps. Order: room - hallway - doorway
	possible_labels[0] = 77;
	possible_labels[1] = 115;
	possible_labels[2] = 179;

	// strings that stores the path to the saving files
	std::string conditional_weights_path = package_path + "/common/files/classifier_models/conditional_field_weights.txt";
	std::string boost_file_path = package_path + "/common/files/classifier_models/";

	// optimal result saving path
	std::string conditional_weights_optimal_path = package_path + "/common/files/optimal_vrf_res/conditional_field_weights.txt";
	std::string boost_file_optimal_path = package_path + "/common/files/optimal_vrf_res/";

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
	training_map = cv::imread(package_path + "/common/files/training_maps/voronoi_random_field_training/training_maps/training_office_e.png", 0);
	training_maps.push_back(training_map);
	training_map = cv::imread(package_path + "/common/files/training_maps/voronoi_random_field_training/training_maps/training_office_h.png", 0);
	training_maps.push_back(training_map);
	training_map = cv::imread(package_path + "/common/files/training_maps/voronoi_random_field_training/training_maps/training_lab_c_furnitures.png", 0);
	training_maps.push_back(training_map);
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
	training_map = cv::imread(package_path + "/common/files/training_maps/voronoi_random_field_training/voronoi_maps/office_e_voronoi.png", 0);
	voronoi_maps.push_back(training_map);
	training_map = cv::imread(package_path + "/common/files/training_maps/voronoi_random_field_training/voronoi_maps/office_h_voronoi.png", 0);
	voronoi_maps.push_back(training_map);
	training_map = cv::imread(package_path + "/common/files/training_maps/voronoi_random_field_training/voronoi_maps/lab_c_furnitures_voronoi.png", 0);
	voronoi_maps.push_back(training_map);
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
	training_map = cv::imread(package_path + "/common/files/training_maps/voronoi_random_field_training/voronoi_node_maps/office_e_voronoi_nodes.png", 0);
	voronoi_node_maps.push_back(training_map);
	training_map = cv::imread(package_path + "/common/files/training_maps/voronoi_random_field_training/voronoi_node_maps/office_h_voronoi_nodes.png", 0);
	voronoi_node_maps.push_back(training_map);
	training_map = cv::imread(package_path + "/common/files/training_maps/voronoi_random_field_training/voronoi_node_maps/lab_c_furnitures_voronoi_nodes.png", 0);
	voronoi_node_maps.push_back(training_map);
	// load the original maps
	std::vector<cv::Mat> original_maps;
	training_map = cv::imread(package_path + "/common/files/training_maps/voronoi_random_field_training/original_maps/Fr52_original.png", 0);
	original_maps.push_back(training_map);
	training_map = cv::imread(package_path + "/common/files/training_maps/voronoi_random_field_training/original_maps/Fr101_original.png", 0);
	original_maps.push_back(training_map);
	training_map = cv::imread(package_path + "/common/files/training_maps/voronoi_random_field_training/original_maps/lab_intel_original.png", 0);
	original_maps.push_back(training_map);
	training_map = cv::imread(package_path + "/common/files/training_maps/voronoi_random_field_training/original_maps/lab_d_furnitures_original.png", 0);
	original_maps.push_back(training_map);
	training_map = cv::imread(package_path + "/common/files/training_maps/voronoi_random_field_training/original_maps/lab_ipa_original.png", 0);
	original_maps.push_back(training_map);
	training_map = cv::imread(package_path + "/common/files/training_maps/voronoi_random_field_training/original_maps/NLB_original.png", 0);
	original_maps.push_back(training_map);
	training_map = cv::imread(package_path + "/common/files/training_maps/voronoi_random_field_training/original_maps/office_e_original.png", 0);
	original_maps.push_back(training_map);
	training_map = cv::imread(package_path + "/common/files/training_maps/voronoi_random_field_training/original_maps/office_h_original.png", 0);
	original_maps.push_back(training_map);
	training_map = cv::imread(package_path + "/common/files/training_maps/voronoi_random_field_training/original_maps/lab_c_furnitures_original.png", 0);
	original_maps.push_back(training_map);

	std::cout << training_maps.size() << " " << voronoi_maps.size() << " " << voronoi_node_maps.size() << " " << original_maps.size() << std::endl;
//	for(size_t i = 0; i < training_maps.size(); ++i)
//	{
//		cv::imshow("training_map", training_maps[i]);
//		cv::imshow("voronoi_map", voronoi_maps[i]);
//		cv::imshow("nodes", voronoi_node_maps[i]);
//		cv::imshow("original_maps", original_maps[i]);
//		cv::waitKey();
//	}

	double map_resolution = 0.05;
	double room_lower_limit_voronoi_ = 1.53;	//1.53;
	double room_upper_limit_voronoi_ = 1000000.;	//120.0;

	std::vector<cv::Point> door_points;

	VoronoiRandomFieldSegmentation segmenter(false, false);

//	segmenter.trainAlgorithms(training_maps, voronoi_maps, voronoi_node_maps, original_maps, possible_labels, conditional_weights_path, boost_file_path);

	for(size_t i = 0; i < map_names.size(); ++i)
	{
		segmenter.segmentMap(maps[i], maps[i], 5, 50, 4, possible_labels, 7, true, conditional_weights_path, boost_file_path, 9000, map_resolution, room_lower_limit_voronoi_, room_upper_limit_voronoi_, 12.5, &door_points);

		std::cout << "number of doorpoints: " << door_points.size() << std::endl;
		door_points.clear();
//		cv::imshow("res", maps[i]);
//		cv::waitKey();
	}

//	segmenter.testFunc(maps[0]);
//
//	// Do several training steps and segment different maps to find the best training-result. This is done by checking the complete
//	// crf-potentials.
//
//	double best_potential = 0;
//
//	for(size_t training = 1; training <= 10; ++training)
//	{
//		if(training != 1)
//			segmenter.trainAlgorithms(training_maps, voronoi_maps, voronoi_node_maps, original_maps, possible_labels, conditional_weights_path, boost_file_path);
//
//		double current_potential = 0;
//
//		for(size_t i = 0; i < 6; ++i)
//		{
//			cv::Mat map = maps[i].clone();
//
//			current_potential += segmenter.segmentMap(map, map, 7, 50, 5, possible_labels, 7, true, conditional_weights_path, boost_file_path, 9000, map_resolution, room_lower_limit_voronoi_, room_upper_limit_voronoi_); // 7, 50, 4, 5
//		}
//
//		std::cout << std::endl << "********** Step: " << training << ". current_potential: " << current_potential << std::endl;
//
//		if(current_potential > best_potential)
//		{
//			best_potential = current_potential;
//			segmenter.testFunc(conditional_weights_optimal_path, boost_file_optimal_path);
//		}
//	}

	return 0;
}
