#include <ipa_room_segmentation/adaboost_classifier.h>

#include <ipa_room_segmentation/features.h>
#include <ipa_room_segmentation/wavefront_region_growing.h>
#include <ipa_room_segmentation/contains.h>
#include <ipa_room_segmentation/raycasting.h>

AdaboostClassifier::AdaboostClassifier()
{
	//save the angles between the simulated beams, used in the following algorithm
	for (double angle = 0; angle < 360; angle++)
	{
		angles_for_simulation_.push_back(angle);
	}
	// Set up boosting parameters
	CvBoostParams params(CvBoost::DISCRETE, 350, 0, 2, false, 0);
	params_ = params;
	trained_ = false;
}

void AdaboostClassifier::trainClassifiers(const std::vector<cv::Mat>& room_training_maps, const std::vector<cv::Mat>& hallway_training_maps,
		const std::string& classifier_storage_path)
{
	//**************************Training-Algorithm for the AdaBoost-classifiers*****************************
	//This Alogrithm trains two AdaBoost-classifiers from OpenCV. It takes the given training maps and finds the Points
	//that are labeled as a room/hallway and calculates the features defined in ipa_room_segmentation/features.h.
	//Then these vectors are put in a format that OpenCV expects for the classifiers and then they are trained.
	std::vector<float> labels_for_hallways, labels_for_rooms;
	std::vector<std::vector<float> > hallway_features, room_features;
	std::vector<double> temporary_beams;
	std::vector<float> temporary_features;
	ROS_INFO("Starting to train the algorithm.");
	//Get the labels for every training point. 1.0 means it belongs to a room and -1.0 means it belongs to a hallway
	for(size_t map = 0; map < room_training_maps.size(); ++map)
	{
		for (int y = 0; y < room_training_maps[map].cols; y++)
		{
			for (int x = 0; x < room_training_maps[map].rows; x++)
			{
				if (room_training_maps[map].at<unsigned char>(x, y) != 0)
				{
					//check for label of each Pixel (if it belongs to doors the label is 1, otherwise it is -1)
					if (room_training_maps[map].at<unsigned char>(x, y) > 250)
					{
						labels_for_rooms.push_back(1.0);
					}
					else
					{
						labels_for_rooms.push_back(-1.0);
					}
					//simulate the beams and features for every position and save it
					temporary_beams = raycasting(room_training_maps[map], cv::Point(x, y));
					for (int f = 1; f <= get_feature_count(); f++)
					{
						temporary_features.push_back((float) get_feature(temporary_beams, angles_for_simulation_, cv::Point(x, y), f));
					}
					room_features.push_back(temporary_features);
					temporary_features.clear();

				}
			}
		}
	}

	for(size_t map = 0; map < hallway_training_maps.size(); ++map)
	{
		for (int y = 0; y < hallway_training_maps[map].cols; y++)
		{
			for (int x = 0; x < hallway_training_maps[map].rows; x++)
			{
				if (hallway_training_maps[map].at<unsigned char>(x, y) != 0)
				{
					//check for label of each Pixel (if it belongs to doors the label is 1, otherwise it is -1)
					if (hallway_training_maps[map].at<unsigned char>(x, y) > 250)
					{
						labels_for_hallways.push_back(1.0);
					}
					else
					{
						labels_for_hallways.push_back(-1.0);
					}
					//simulate the beams and features for every position and save it
					temporary_beams = raycasting(hallway_training_maps[map], cv::Point(x, y));
					for (int f = 1; f <= get_feature_count(); f++)
					{
						temporary_features.push_back(get_feature(temporary_beams, angles_for_simulation_, cv::Point(x, y), f));
					}
					hallway_features.push_back(temporary_features);
					temporary_features.clear();

				}
			}
		}
	}

	//*********hallway***************
	//save the found labels and features in Matrices
	cv::Mat hallway_labels_Mat(labels_for_hallways.size(), 1, CV_32FC1);
	cv::Mat hallway_features_Mat(hallway_features.size(), get_feature_count(), CV_32FC1);
	for (int i = 0; i < labels_for_hallways.size(); i++)
	{
		hallway_labels_Mat.at<float>(i, 0) = labels_for_hallways[i];
		for (int f = 0; f < get_feature_count(); f++)
		{
			hallway_features_Mat.at<float>(i, f) = (float) hallway_features[i][f];
		}
	}
	// Train a boost classifier
	hallway_boost_.train(hallway_features_Mat, CV_ROW_SAMPLE, hallway_labels_Mat, cv::Mat(), cv::Mat(), cv::Mat(), cv::Mat(), params_);
	//save the trained booster
	std::string filename_hallway = classifier_storage_path + "trained_hallway_boost.xml";
	hallway_boost_.save(filename_hallway.c_str(), "boost");
	ROS_INFO("Done hallway classifiers.");

	//*************room***************
	//save the found labels and features in Matrices
	cv::Mat room_labels_Mat(labels_for_rooms.size(), 1, CV_32FC1);
	cv::Mat room_features_Mat(room_features.size(), get_feature_count(), CV_32FC1);
	for (int i = 0; i < labels_for_rooms.size(); i++)
	{
		room_labels_Mat.at<float>(i, 0) = labels_for_rooms[i];
		for (int f = 0; f < get_feature_count(); f++)
		{
			room_features_Mat.at<float>(i, f) = (float) room_features[i][f];
		}
	}
	// Train a boost classifier
	room_boost_.train(room_features_Mat, CV_ROW_SAMPLE, room_labels_Mat, cv::Mat(), cv::Mat(), cv::Mat(), cv::Mat(), params_);
	//save the trained booster
	std::string filename_room = classifier_storage_path + "trained_room_boost.xml";
	room_boost_.save(filename_room.c_str(), "boost");
	//set the trained-variabel true, so the labeling-algorithm knows the classifiers have been trained already
	trained_ = true;
	ROS_INFO("Done room classifiers.");
	ROS_INFO("Finished training the algorithm.");
}

void AdaboostClassifier::semanticLabeling(const cv::Mat& map_to_be_labeled, cv::Mat& segmented_map, double map_resolution_from_subscription,
        double room_area_factor_lower_limit, double room_area_factor_upper_limit, const std::string& classifier_storage_path, bool display_results)
{
	//******************Semantic-labeling function based on AdaBoost*****************************
	//This function calculates single-valued features for every white Pixel in the given occupancy-gridmap and classifies it
	//using the AdaBoost-algorithm from OpenCV. It does the following steps:
	//	I. If the classifiers hasn't been trained before they should load the training-results saved in the
	//	   training_results folder
	//	II. Go trough each Pixel of the given map. If this Pixel is white simulate the laser-beams for it and calculate each
	//		of the implemented features.
	//	III. Apply a median-Filter on the labeled map to smooth the output of it.
	//	IV. Find the contours of the segments given by III. by thresholding the map. First set the threshold so high that
	//		only room-areas are shown in the map and find them. Then make these room-areas black and finally set the threshold
	//		a little lower than the hallway-colour. The function only takes contours that are larger than the minimum value
	//		and splits too large hallway-areas into smaller areas by putting random Points into the too large hallway contour
	//		and apply a watershed-algorithm on it. At last the saved room and hallway contours are drawn with a random
	//		colour into the map that hasn't been used already.

	cv::Mat original_Map_to_be_labeled = map_to_be_labeled.clone();
	ROS_INFO("Starting to label the map.");
	//***********************I. check if classifiers has already been trained*****************************
	if (!trained_) //classifiers hasn't been trained before so they should be loaded
	{
		std::string filename_room = classifier_storage_path + "trained_room_boost.xml";
		room_boost_.load(filename_room.c_str());
		std::string filename_hallway = classifier_storage_path + "trained_hallway_boost.xml";
		hallway_boost_.load(filename_hallway.c_str());
		trained_ = true;
		ROS_INFO("Loaded training results.");
	}
	//*************** II. Go trough each Point and label it as room or hallway.**************************
	for (int x = 0; x < original_Map_to_be_labeled.rows; x++)
	{
		for (int y = 0; y < original_Map_to_be_labeled.cols; y++)
		{
			if (original_Map_to_be_labeled.at<unsigned char>(x, y) == 255)
			{
				//TODO: x und y richtig machen
				std::vector<double> temporary_beams = raycasting(original_Map_to_be_labeled, cv::Point(x, y));
				std::vector<float> temporary_features;
				cv::Mat featuresMat(1, get_feature_count(), CV_32FC1); //OpenCV expects a 32-floating-point Matrix as feature input
				for (int f = 1; f <= get_feature_count(); f++)
				{
					//get the features for each room and put it in the featuresMat
					featuresMat.at<float>(0, f - 1) = (float) get_feature(temporary_beams, angles_for_simulation_, cv::Point(x, y), f);
				}
				//classify each Point
				float room_sum = room_boost_.predict(featuresMat, cv::Mat(), cv::Range::all(), false, true);
				float hallway_sum = hallway_boost_.predict(featuresMat, cv::Mat(), cv::Range::all(), false, true);
				//get the certanity-values for each class (it shows the probability that it belongs to the given class)
				double room_certanity = (std::exp((double) room_sum)) / (std::exp(-1 * (double) room_sum) + std::exp((double) room_sum));
				double hallway_certanity = (std::exp((double) hallway_certanity))
				        / (std::exp(-1 * (double) hallway_certanity) + std::exp((double) hallway_certanity));
				//make a decision-list and check which class the Point belongs to
				double probability_for_room = room_certanity;
				double probability_for_hallway = hallway_certanity * (1.0 - probability_for_room);
				if (probability_for_room > probability_for_hallway)
				{
					original_Map_to_be_labeled.at<unsigned char>(x, y) = 150; //label it as room
				}
				else
				{
					original_Map_to_be_labeled.at<unsigned char>(x, y) = 100; //label it as hallway
				}
			}
		}
	}
	std::cout << "labeled all white Pixels" << std::endl;
	//******************** III. Apply a median filter over the image to smooth the results.***************************

	cv::Mat temporary_map = original_Map_to_be_labeled.clone();
	cv::medianBlur(temporary_map, temporary_map, 3);
	std::cout << "blurred image" << std::endl;

	//make regions black, that has been black before
	for (int x = 0; x < original_Map_to_be_labeled.rows; x++)
	{
		for (int y = 0; y < original_Map_to_be_labeled.cols; y++)
		{
			if (original_Map_to_be_labeled.at<unsigned char>(x, y) == 0)
			{
				temporary_map.at<unsigned char>(x, y) = 0;
			}
		}
	}
//	cv::imshow("thresholded", temporary_map);
//	cv::imwrite("/home/rmb/Bilder/semantic_classified.png", temporary_map);
//	cv::waitKey();
	if(display_results)
	{
		cv::imshow("classified", temporary_map);
		cv::waitKey();
	}
	cv::Mat blured_image_for_thresholding = temporary_map.clone();

	//*********** IV. Fill the large enough rooms with a random color and split the hallways into smaller regions*********

	std::vector<std::vector<cv::Point> > contours, temporary_contours, saved_room_contours, saved_hallway_contours;
	//hierarchy saves if the contours are hole-contours:
	//hierarchy[{0,1,2,3}]={next contour (same level), previous contour (same level), child contour, parent contour}
	//child-contour = 1 if it has one, = -1 if not, same for parent_contour
	std::vector < cv::Vec4i > hierarchy;

	std::vector < cv::Scalar > already_used_colors; //saving-vector for the already used coloures

	//find the contours, which are labeled as a room
	cv::threshold(temporary_map, temporary_map, 120, 255, cv::THRESH_BINARY); //find rooms (value = 150)
	cv::findContours(temporary_map, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_NONE);
	cv::drawContours(blured_image_for_thresholding, contours, -1, cv::Scalar(0), CV_FILLED); //make the found regions at the original map black, because they have been looked at already

	//only take rooms that are large enough and that are not a hole-contour
	for (int c = 0; c < contours.size(); c++)
	{
		if (map_resolution_from_subscription * map_resolution_from_subscription * cv::contourArea(contours[c]) > room_area_factor_lower_limit
		        && hierarchy[c][3] != 1)
		{
			saved_room_contours.push_back(contours[c]);
		}
	}
	//find the contours, which are labeled as a hallway

	map_to_be_labeled.convertTo(segmented_map, CV_32SC1, 256, 0);		// rescale to 32 int, 255 --> 255*256 = 65280
	temporary_map = blured_image_for_thresholding.clone();

	cv::threshold(temporary_map, temporary_map, 90, 255, cv::THRESH_BINARY); //find hallways (value = 100)
	cv::findContours(temporary_map, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_NONE);
	//if the hallway-contours are too big split them into smaller regions, also don't take too small regions
	for (int contour_counter = 0; contour_counter < contours.size(); contour_counter++)
	{
		if (map_resolution_from_subscription * map_resolution_from_subscription * cv::contourArea(contours[contour_counter]) > room_area_factor_upper_limit)
		{
			//Generate a black map to draw the hallway-contour in. Then use this map to ckeck if the generated random Points
			// are inside the contour.
			cv::Mat contour_Map = cv::Mat::zeros(temporary_map.rows, temporary_map.cols, CV_8UC1);
			cv::drawContours(contour_Map, contours, contour_counter, cv::Scalar(255), CV_FILLED);
			cv::erode(contour_Map, contour_Map, cv::Mat(), cv::Point(-1,-1), 10);
			//center-counter so enough centers could be found
			int center_counter = 0;
			//saving-vector for watershed centers
			std::vector < cv::Point > temporary_watershed_centers;
			//find enough random watershed centers that are inside the hallway-contour
			do
			{
				int random_x = rand() % temporary_map.rows;
				int random_y = rand() % temporary_map.cols;
				if (contour_Map.at<unsigned char>(random_y, random_x) == 255)
				{
					temporary_watershed_centers.push_back(cv::Point(random_x, random_y));
					center_counter++;
				}
			} while (center_counter <= (map_resolution_from_subscription * map_resolution_from_subscription * cv::contourArea(contours[contour_counter])) / 8);		// todo: parameter
			cv::Mat temporary_Map_to_wavefront;
			contour_Map.convertTo(temporary_Map_to_wavefront, CV_32SC1, 256, 0);
			//draw the centers as white circles into a black map and give the center-map and the contour-map to the opencv watershed-algorithm
			for (int current_center = 0; current_center < temporary_watershed_centers.size(); current_center++)
			{
				bool coloured = false;
				do
				{
					cv::Scalar fill_colour(rand() % 52224 + 13056);
					if (!contains(already_used_colors, fill_colour))
					{
						cv::circle(temporary_Map_to_wavefront, temporary_watershed_centers[current_center], 2, fill_colour, CV_FILLED);
						already_used_colors.push_back(fill_colour);
						coloured = true;
					}
				} while (!coloured);
			}
			//make sure all previously black Pixels are still black
			for (int x = 0; x < map_to_be_labeled.rows; x++)
			{
				for (int y = 0; y < map_to_be_labeled.cols; y++)
				{
					if (map_to_be_labeled.at<unsigned char>(x, y) == 0)
					{
						temporary_Map_to_wavefront.at<unsigned char>(x, y) = 0;
					}
				}
			}
			wavefrontRegionGrowing(temporary_Map_to_wavefront);
			//draw the seperated contour into the map, which should be labeled
			for (int row = 0; row < segmented_map.rows; row++)
			{
				for (int col = 0; col < segmented_map.cols; col++)
				{
					if (temporary_Map_to_wavefront.at<int>(row, col) != 0)
					{
						segmented_map.at<int>(row, col) = temporary_Map_to_wavefront.at<int>(row, col);
					}
				}
			}
		}
		else if (map_resolution_from_subscription * map_resolution_from_subscription * cv::contourArea(contours[contour_counter])
		        > room_area_factor_lower_limit)
		{
			saved_hallway_contours.push_back(contours[contour_counter]);
		}
	}
	std::cout << "finished too big hallway contours" << std::endl;
	//draw every room and lasting hallway contour with a random colour into the map
	for (int room = 0; room < saved_room_contours.size(); room++)
	{
		bool coloured = false;
		do
		{
			cv::Scalar fill_colour(rand() % 52224 + 13056);
			if (!contains(already_used_colors, fill_colour))
			{
				cv::drawContours(segmented_map, saved_room_contours, room, fill_colour, CV_FILLED);
				already_used_colors.push_back(fill_colour);
				coloured = true;
			}
		} while (!coloured);
	}
	std::cout << "finished room contours" << std::endl;
	for (int hallway = 0; hallway < saved_hallway_contours.size(); hallway++)
	{
		bool coloured = false;
		int loop_counter = 0; //loop-counter to exit the loop if it gets a infite loop
		do
		{
			loop_counter++;
			cv::Scalar fill_colour(rand() % 52224 + 13056);
			if (!contains(already_used_colors, fill_colour) || loop_counter > 250)
			{
				cv::drawContours(segmented_map, saved_hallway_contours, hallway, fill_colour, CV_FILLED);
				already_used_colors.push_back(fill_colour);
				coloured = true;
			}
		} while (!coloured);
	}
	std::cout << "finished small hallway contours" << std::endl;
	//spread the coloured regions to regions, which were too small and aren't drawn into the map
	wavefrontRegionGrowing(segmented_map);
	//make sure previously black Pixels are still black
	// todo: why is this necessary? it seems like this should not alter any data.
	for (int v = 0; v < map_to_be_labeled.rows; ++v)
	{
		for (int u = 0; u < map_to_be_labeled.cols; ++u)
		{
			if (map_to_be_labeled.at<unsigned char>(v, u) == 0)
			{
				segmented_map.at<int>(v, u) = 0;
			}
		}
	}
	ROS_INFO("Finished Labeling the map.");
}
