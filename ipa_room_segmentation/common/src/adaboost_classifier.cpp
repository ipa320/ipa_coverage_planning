#include <ipa_room_segmentation/adaboost_classifier.h>

adaboost_classifier::adaboost_classifier(cv::Mat original_map_from_subscription, double map_resolution_from_subscription, double room_area_factor_lower_limit,
        double room_area_factor_upper_limit)
{
	map_resolution_from_subscription_ = map_resolution_from_subscription;
	room_area_factor_lower_limit_ = room_area_factor_lower_limit;
	room_area_factor_upper_limit_ = room_area_factor_upper_limit;
	//save the angles between the simulated beams, used in the following algorithm
	for (double angle = 0; angle < 360; angle++)
	{
		angles_for_simulation_.push_back(angle);
	}
	// Set up boosting parameters
	CvBoostParams params(CvBoost::DISCRETE, 400, 0, 2, false, 0);
	params_ = params;
	//set the initial value for trained-shower
	trained_ = false;
}

void adaboost_classifier::trainClassifiers(cv::Mat first_room_training_map, cv::Mat second_room_training_map, cv::Mat first_hallway_training_map,
        cv::Mat second_hallway_training_map)
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
	//first room training map (this map has only one part of it prelabeled from me):
	for (int y = 82; y < 357; y++)
	{
		for (int x = 84; x < 361; x++)
		{
			if (first_room_training_map.at<unsigned char>(x, y) != 0)
			{
				//check for label of each Pixel (if it belongs to doors the label is 1, otherwise it is -1)
				if (first_room_training_map.at<unsigned char>(x, y) != 255)
				{
					labels_for_rooms.push_back(1.0);
				}
				else
				{
					labels_for_rooms.push_back(-1.0);
				}
				//simulate the beams and features for every position and save it
				temporary_beams = raycasting(first_room_training_map, cv::Point(x, y));
				for (int f = 1; f <= 23; f++)
				{
					temporary_features.push_back((float) get_feature(temporary_beams, angles_for_simulation_, cv::Point(x, y), f));
				}
				room_features.push_back(temporary_features);
				temporary_features.clear();

			}
		}
	}
	//second room training map:
	for (int y = 0; y < second_room_training_map.cols; y++)
	{
		for (int x = 0; x < second_room_training_map.rows; x++)
		{
			if (second_room_training_map.at<unsigned char>(x, y) != 0)
			{
				//check for label of each Pixel (if it belongs to doors the label is 1, otherwise it is -1)
				if (second_room_training_map.at<unsigned char>(x, y) != 255)
				{
					labels_for_rooms.push_back(1.0);
				}
				else
				{
					labels_for_rooms.push_back(-1.0);
				}
				//simulate the beams and features for every position and save it
				temporary_beams = raycasting(second_room_training_map, cv::Point(x, y));
				for (int f = 1; f <= 23; f++)
				{
					temporary_features.push_back((float) get_feature(temporary_beams, angles_for_simulation_, cv::Point(x, y), f));
				}
				room_features.push_back(temporary_features);
				temporary_features.clear();

			}
		}
	}
	//first hallway training map (this map has only one part of it prelabeled from me):
	for (int y = 82; y < 357; y++)
	{
		for (int x = 84; x < 361; x++)
		{
			if (first_hallway_training_map.at<unsigned char>(x, y) != 0)
			{
				//check for label of each Pixel (if it belongs to doors the label is 1, otherwise it is -1)
				if (first_hallway_training_map.at<unsigned char>(x, y) != 255)
				{
					labels_for_hallways.push_back(1.0);
				}
				else
				{
					labels_for_hallways.push_back(-1.0);
				}
				//simulate the beams and features for every position and save it
				temporary_beams = raycasting(first_hallway_training_map, cv::Point(x, y));
				for (int f = 1; f <= 23; f++)
				{
					temporary_features.push_back(get_feature(temporary_beams, angles_for_simulation_, cv::Point(x, y), f));
				}
				hallway_features.push_back(temporary_features);
				temporary_features.clear();

			}
		}
	}
	//second hallway training map:
	for (int y = 0; y < second_hallway_training_map.cols; y++)
	{
		for (int x = 0; x < second_hallway_training_map.rows; x++)
		{
			if (second_hallway_training_map.at<unsigned char>(x, y) != 0)
			{
				//check for label of each Pixel (if it belongs to doors the label is 1, otherwise it is -1)
				if (second_hallway_training_map.at<unsigned char>(x, y) != 255)
				{
					labels_for_hallways.push_back(1.0);
				}
				else
				{
					labels_for_hallways.push_back(-1.0);
				}
				//simulate the beams and features for every position and save it
				temporary_beams = raycasting(second_hallway_training_map, cv::Point(x, y));
				for (int f = 1; f <= 23; f++)
				{
					temporary_features.push_back(get_feature(temporary_beams, angles_for_simulation_, cv::Point(x, y), f));
				}
				hallway_features.push_back(temporary_features);
				temporary_features.clear();

			}
		}
	}

	//*********hallway***************
	//save the found labels and features in Matrices
	cv::Mat hallway_labels_Mat(labels_for_hallways.size(), 1, CV_32FC1);
	cv::Mat hallway_features_Mat(hallway_features.size(), 23, CV_32FC1);
	for (int i = 0; i < labels_for_hallways.size(); i++)
	{
		hallway_labels_Mat.at<float>(i, 0) = labels_for_hallways[i];
		for (int f = 0; f < 23; f++)
		{
			hallway_features_Mat.at<float>(i, f) = (float) hallway_features[i][f];
		}
	}
	// Train a boost classifier
	hallway_boost_.train(hallway_features_Mat, CV_ROW_SAMPLE, hallway_labels_Mat, cv::Mat(), cv::Mat(), cv::Mat(), cv::Mat(), params_);
	//save the trained booster
	// todo: do not use fixed paths! use a relative path --> data will end up in ~/.ros/...
	hallway_boost_.save("src/autopnp/ipa_room_segmentation/training_results/trained_hallway_boost.xml", "boost");
	ROS_INFO("Done hallway classifiers.");

	//*************room***************
	//save the found labels and features in Matrices
	cv::Mat room_labels_Mat(labels_for_rooms.size(), 1, CV_32FC1);
	cv::Mat room_features_Mat(room_features.size(), 23, CV_32FC1);
	for (int i = 0; i < labels_for_rooms.size(); i++)
	{
		room_labels_Mat.at<float>(i, 0) = labels_for_rooms[i];
		for (int f = 0; f < 23; f++)
		{
			room_features_Mat.at<float>(i, f) = (float) room_features[i][f];
		}
	}
	// Train a boost classifier
	room_boost_.train(room_features_Mat, CV_ROW_SAMPLE, room_labels_Mat, cv::Mat(), cv::Mat(), cv::Mat(), cv::Mat(), params_);
	//save the trained booster
	//room_boost.save("/home/rmb-fj/roomsegmentation/src/segmentation/src/training_results/trained_room_boost.xml", "boost");
	room_boost_.save("src/autopnp/ipa_room_segmentation/training_results/trained_room_boost.xml", "boost");
	//set the trained-variabel true, so the labeling-algorithm knows the classifiers have been trained already
	trained_ = true;
	ROS_INFO("Done room classifiers.");
	ROS_INFO("Finsihed training the algorithm.");
}

cv::Mat adaboost_classifier::semanticLabeling(cv::Mat map_to_be_labeled)
{
	//******************Semantic-labeling function based on AdaBoost*****************************
	//This function calculates single-valued features for every white Pixel in the given occupancy-gridmap and classifies it
	//using the AdaBoost-algorithm from OpenCV. It does the following steps:
	//	I. If the classifiers hasn't been trained before they should load the training-results saved in the
	//	   training_results folder
	//	II.
	cv::Mat original_Map_to_be_labeled = map_to_be_labeled.clone();
	ROS_INFO("Starting to label the map.");
	//***********************I. check if classifiers has already been trained*****************************
	if (!trained_) //classifiers hasn't been trained before so they should be loaded
	{
		room_boost_.load("src/autopnp/ipa_room_segmentation/training_results/trained_room_boost.xml");
		hallway_boost_.load("src/autopnp/ipa_room_segmentation/training_results/trained_hallway_boost.xml");
		ROS_INFO("Loaded training results.");
	}
	//*************** II. Go trough each Point and label it as room or hallway.**************************
	for (int x = 0; x < original_Map_to_be_labeled.rows; x++)
	{
		for (int y = 0; y < original_Map_to_be_labeled.cols; y++)
		{
			if (original_Map_to_be_labeled.at<unsigned char>(x, y) == 255)
			{
				std::vector<double> temporary_beams = raycasting(original_Map_to_be_labeled, cv::Point(x, y));
				std::vector<float> temporary_features;
				cv::Mat featuresMat(1, 23, CV_32FC1); //OpenCV expects a 32-floating-point Matrix as feature input
				for (int f = 1; f <= 23; f++)
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
	//apply a median filter over the image to smooth the results
	cv::Mat temporary_map = original_Map_to_be_labeled.clone();
	cv::medianBlur(temporary_map, temporary_map, 3);
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
	cv::Mat labeling_Map = map_to_be_labeled.clone();
	cv::Mat blured_image_for_thresholding = temporary_map.clone();
	// todo: no absolute paths
	cv::imwrite("/home/rmb-fj/Pictures/maps/semantic/opencv/opencvboost.png", temporary_map);

	//fill the large enough rooms with a random color and split the hallways into smaller regions
	std::vector<std::vector<cv::Point> > contours, temporary_contours, saved_room_contours, saved_hallway_contours;
	//hierarchy saves if the contours are hole-contours:
	//hierarchy[{0,1,2,3}]={next contour (same level), previous contour (same level), child contour, parent contour}
	//child-contour = 1 if it has one, = -1 if not, same for parent_contour
	std::vector < cv::Vec4i > hierarchy;

	//find the contours, which are labeled as a room
	cv::threshold(temporary_map, temporary_map, 120, 255, cv::THRESH_BINARY); //find rooms (value = 150)
	cv::findContours(temporary_map, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_NONE);
	cv::drawContours(blured_image_for_thresholding, contours, -1, cv::Scalar(0), CV_FILLED); //make the found regions at the original map black, because they have been looked at already

	//only take rooms that are large enough and that are not a hole-contour
	for (int c = 0; c < contours.size(); c++)
	{
		if (map_resolution_from_subscription_ * map_resolution_from_subscription_ * cv::contourArea(contours[c]) > room_area_factor_lower_limit_
				&& hierarchy[c][3] != 1)
		{
			saved_room_contours.push_back(contours[c]);
		}
	}

	//find the contours, which are labeled as a hallway
	temporary_map = blured_image_for_thresholding.clone();
	cv::threshold(temporary_map, temporary_map, 90, 255, cv::THRESH_BINARY); //find hallways (value = 100)
	cv::findContours(temporary_map, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_NONE);
	//if the hallway-contours are too big split them into smaller regions, also don't take too small regions
	for (int contour_counter = 0; contour_counter < contours.size(); contour_counter++)
	{
		if (map_resolution_from_subscription_ * map_resolution_from_subscription_ * cv::contourArea(contours[contour_counter]) > room_area_factor_upper_limit_)
		{
			//Generate a black map to draw the hallway-contour in. Then use this map to ckeck if the generated random Points
			// are inside the contour.
			cv::Mat contour_Map = cv::Mat::zeros(temporary_map.cols, temporary_map.rows, CV_8UC1);
			cv::drawContours(contour_Map, contours, contour_counter, cv::Scalar(255), CV_FILLED);
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
			} while (center_counter <= (map_resolution_from_subscription_ * map_resolution_from_subscription_ * cv::contourArea(contours[contour_counter])) / 8);
			//draw the centers as white circles into a black map and give the center-map and the contour-map to the opencv watershed-algorithm
			for (int current_center = 0; current_center < temporary_watershed_centers.size(); current_center++)
			{
				bool coloured = false;
				do
				{
					cv::Scalar fill_colour(rand() % 200 + 53);
					if (!contains(already_used_colours_, fill_colour))
					{
						cv::circle(contour_Map, temporary_watershed_centers[current_center], 2, fill_colour, CV_FILLED);
						already_used_colours_.push_back(fill_colour);
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
						contour_Map.at<unsigned char>(x, y) = 0;
					}
				}
			}
			temporary_map = watershed_region_spreading(contour_Map);
			cv::imwrite("/home/rmb-fj/Pictures/maps/semantic/spreading.png", temporary_map);
			//draw the seperated contour into the map, which should be labeled
			for (int x = 0; x < labeling_Map.rows; x++)
			{
				for (int y = 0; y < labeling_Map.cols; y++)
				{
					if (temporary_map.at<unsigned char>(x, y) != 0)
					{
						labeling_Map.at<unsigned char>(x, y) = temporary_map.at<unsigned char>(x, y);
					}
				}
			}
		}
		else if (map_resolution_from_subscription_ * map_resolution_from_subscription_ * cv::contourArea(contours[contour_counter]) > room_area_factor_lower_limit_)
		{
			saved_hallway_contours.push_back(contours[contour_counter]);
		}
	}
	//draw every room and lasting hallway contour with a random colour into the map
	for (int room = 0; room < saved_room_contours.size(); room++)
	{
		bool coloured = false;
		do
		{
			cv::Scalar fill_colour(rand() % 200 + 53);
			if (!contains(already_used_colours_, fill_colour))
			{
				cv::drawContours(labeling_Map, saved_room_contours, room, fill_colour, CV_FILLED);
				already_used_colours_.push_back(fill_colour);
				coloured = true;
			}
		} while (!coloured);
	}
	for (int hallway = 0; hallway < saved_hallway_contours.size(); hallway++)
	{
		bool coloured = false;
		do
		{
			cv::Scalar fill_colour(rand() % 200 + 53);
			if (!contains(already_used_colours_, fill_colour))
			{
				cv::drawContours(labeling_Map, saved_hallway_contours, hallway, fill_colour, CV_FILLED);
				already_used_colours_.push_back(fill_colour);
				coloured = true;
			}
		} while (!coloured);
	}
	//spread the coloured regions to regions, which were too small and aren't drawn into the map
	labeling_Map = watershed_region_spreading(labeling_Map);
	//make sure previously black Pixels are still black
	for (int x = 0; x < map_to_be_labeled.rows; x++)
	{
		for (int y = 0; y < map_to_be_labeled.cols; y++)
		{
			if (map_to_be_labeled.at<unsigned char>(x, y) == 0)
			{
				labeling_Map.at<unsigned char>(x, y) = 0;
			}
		}
	}
	ROS_INFO("Finsihed Labeling the map.");
	return labeling_Map;
}
