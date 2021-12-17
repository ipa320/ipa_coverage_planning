#include <ipa_room_segmentation/adaboost_classifier.h>

#include <ipa_room_segmentation/wavefront_region_growing.h>
#include <ipa_room_segmentation/contains.h>
#include <ipa_room_segmentation/cv_boost_loader.h>

#include <ipa_room_segmentation/timer.h>

#include <boost/filesystem.hpp>

AdaboostClassifier::AdaboostClassifier()
{
	//save the angles between the simulated beams, used in the following algorithm
	for (double angle = 0; angle < 360; angle++)
	{
		angles_for_simulation_.push_back(angle);
	}
#if CV_MAJOR_VERSION == 2
	// Set up boosting parameters
	CvBoostParams params(CvBoost::DISCRETE, 350, 0, 2, false, 0);
	params_ = params;
#endif
	trained_ = false;
}

void AdaboostClassifier::trainClassifiers(const std::vector<cv::Mat>& room_training_maps, const std::vector<cv::Mat>& hallway_training_maps,
		const std::string& classifier_storage_path, bool load_features)
{
	//**************************Training-Algorithm for the AdaBoost-classifiers*****************************
	//This Alogrithm trains two AdaBoost-classifiers from OpenCV. It takes the given training maps and finds the Points
	//that are labeled as a room/hallway and calculates the features defined in ipa_room_segmentation/features.h.
	//Then these vectors are put in a format that OpenCV expects for the classifiers and then they are trained.
	std::vector<int> labels_for_hallways, labels_for_rooms;
	std::vector<std::vector<float> > hallway_features, room_features;
	std::vector<double> temporary_beams;
	std::vector<float> temporary_features;
	cv::Mat hallway_labels_mat, room_labels_mat;
	cv::Mat hallway_features_mat, room_features_mat;
	std::cout << "Starting to train the algorithm." << std::endl;
	std::cout << "number of room training maps: " << room_training_maps.size() << std::endl;
	std::cout << "number of hallway training maps: " << hallway_training_maps.size() << std::endl;
	//Get the labels for every training point. 1.0 means it belongs to a room and -1.0 means it belongs to a hallway
	// if not loading precomputed features, compute them here
	if(load_features==false)
	{
		LaserScannerFeatures lsf;
		for(size_t map = 0; map < room_training_maps.size(); ++map)
		{
			for (int y = 0; y < room_training_maps[map].rows; y++)
			{
				for (int x = 0; x < room_training_maps[map].cols; x++)
				{
					if (room_training_maps[map].at<unsigned char>(y, x) != 0)
					{
						//check for label of each Pixel (if it belongs to rooms the label is 1, otherwise it is -1)
						if (room_training_maps[map].at<unsigned char>(y, x) > 250)
						{
							labels_for_rooms.push_back(-1);
						}
						else
						{
							labels_for_rooms.push_back(1);
						}
						//simulate the beams and features for every position and save it
						raycasting_.raycasting(room_training_maps[map], cv::Point(x, y), temporary_beams);
						cv::Mat features;
						lsf.get_features(temporary_beams, angles_for_simulation_, cv::Point(x, y), features);
						temporary_features.resize(features.cols);
						for (int i=0; i<features.cols; ++i)
							temporary_features[i] = features.at<float>(0,i);
						room_features.push_back(temporary_features);
						temporary_features.clear();
					}
				}
			}
			std::cout << "done one room map" << std::endl;
		}

		for(size_t map = 0; map < hallway_training_maps.size(); ++map)
		{
			for (int y = 0; y < hallway_training_maps[map].rows; y++)
			{
				for (int x = 0; x < hallway_training_maps[map].cols; x++)
				{
					if (hallway_training_maps[map].at<unsigned char>(y, x) != 0)
					{
						//check for label of each Pixel (if it belongs to hallways the label is 1, otherwise it is -1)
						if (hallway_training_maps[map].at<unsigned char>(y, x) > 250)
						{
							labels_for_hallways.push_back(-1);
						}
						else
						{
							labels_for_hallways.push_back(1);
						}
						//simulate the beams and features for every position and save it
						raycasting_.raycasting(hallway_training_maps[map], cv::Point(x, y), temporary_beams);
						cv::Mat features;
						lsf.get_features(temporary_beams, angles_for_simulation_, cv::Point(x, y), features);
						temporary_features.resize(features.cols);
						for (int i=0; i<features.cols; ++i)
							temporary_features[i] = features.at<float>(0,i);
						hallway_features.push_back(temporary_features);
						temporary_features.clear();
					}
				}
			}
			std::cout << "done one hallway map" << std::endl;
		}

		//save the found labels and features in Matrices --> hallway
		hallway_labels_mat = cv::Mat(labels_for_hallways);
		hallway_features_mat = cv::Mat(hallway_features.size(), lsf.get_feature_count(), CV_32FC1);
		for (int i = 0; i < labels_for_hallways.size(); i++)
		{
	//		hallway_labels_mat.at<float>(i, 0) = labels_for_hallways[i];
			for (int f = 0; f < hallway_features[i].size(); f++)
			{
				hallway_features_mat.at<float>(i, f) = (float) hallway_features[i][f];
			}
		}
		//save the found labels and features in Matrices --> rooms
		room_labels_mat = cv::Mat(labels_for_rooms);
		room_features_mat = cv::Mat(room_features.size(), lsf.get_feature_count(), CV_32FC1);
		for (int i = 0; i < labels_for_rooms.size(); i++)
		{
	//		room_labels_mat.at<float>(i, 0) = labels_for_rooms[i];
			for (int f = 0; f < room_features[i].size(); f++)
			{
				room_features_mat.at<float>(i, f) = (float) room_features[i][f];
			}
		}

	//	// save feature data to file
		cv::FileStorage fs(classifier_storage_path+"_features.yml", cv::FileStorage::WRITE);
		if (fs.isOpened())
		{
			fs << "hallway_features_mat" << hallway_features_mat;
			fs << "hallway_labels_mat" << hallway_labels_mat;
			fs << "room_features_mat" << room_features_mat;
			fs << "room_labels_mat" << room_labels_mat;
		}
		fs.release();
	}
	else
	{
		// load the feature data from file
		cv::FileStorage fs(classifier_storage_path+"_features.yml", cv::FileStorage::READ);
		std::cout << "Loading feature data from file: " << classifier_storage_path+"_features.yml" << std::endl;
		if (fs.isOpened())
		{
			fs["hallway_features_mat"] >> hallway_features_mat;
			fs["hallway_labels_mat"] >> hallway_labels_mat;
			fs["room_features_mat"] >> room_features_mat;
			fs["room_labels_mat"] >> room_labels_mat;
		}
		std::cout << "Loaded features data" << std::endl;
		fs.release();
	}

	// check if path for storing classifier models exists
	boost::filesystem::path storage_path(classifier_storage_path);
	if (boost::filesystem::exists(storage_path) == false)
	{
		if (boost::filesystem::create_directories(storage_path) == false && boost::filesystem::exists(storage_path) == false)
		{
			std::cout << "Error: AdaboostClassifier::trainClassifiers: Could not create directory " << storage_path << std::endl;
			return;
		}
	}

	//*********hallway***************
	std::string filename_hallway = classifier_storage_path + "semantic_hallway_boost.xml";
#if CV_MAJOR_VERSION == 2
	// Train a boost classifier
	hallway_boost_.train(hallway_features_mat, CV_ROW_SAMPLE, hallway_labels_mat, cv::Mat(), cv::Mat(), cv::Mat(), cv::Mat(), params_);
	//save the trained booster
	hallway_boost_.save(filename_hallway.c_str(), "boost");
#else
	// Train a boost classifier
	hallway_boost_ = cv::ml::Boost::create();
	hallway_boost_->setBoostType(cv::ml::Boost::REAL);
	hallway_boost_->setWeakCount(350);
	hallway_boost_->setWeightTrimRate(0);
	hallway_boost_->setMaxDepth(2);
	hallway_boost_->setUseSurrogates(false);
	hallway_boost_->setPriors(cv::Mat());
	hallway_boost_->train(hallway_features_mat, cv::ml::ROW_SAMPLE, hallway_labels_mat);
	//save the trained booster
	hallway_boost_->save(filename_hallway.c_str());
#endif
	ROS_INFO("Done hallway classifiers.");

	//*************room***************
	std::string filename_room = classifier_storage_path + "semantic_room_boost.xml";
#if CV_MAJOR_VERSION == 2
	// Train a boost classifier
	room_boost_.train(room_features_mat, CV_ROW_SAMPLE, room_labels_mat, cv::Mat(), cv::Mat(), cv::Mat(), cv::Mat(), params_);
	//save the trained booster
	room_boost_.save(filename_room.c_str(), "boost");
#else
	// Train a boost classifier
	room_boost_ = cv::ml::Boost::create();
	room_boost_->setBoostType(cv::ml::Boost::REAL);
	room_boost_->setWeakCount(350);
	room_boost_->setWeightTrimRate(0);
	room_boost_->setMaxDepth(2);
	room_boost_->setUseSurrogates(false);
	room_boost_->setPriors(cv::Mat());
	room_boost_->train(room_features_mat, cv::ml::ROW_SAMPLE, room_labels_mat);
	//save the trained booster
	room_boost_->save(filename_room.c_str());
#endif

	//set the trained-variabel true, so the labeling-algorithm knows the classifiers have been trained already
	trained_ = true;
	ROS_INFO("Done room classifiers.");
	ROS_INFO("Finished training the algorithm.");
}

void AdaboostClassifier::segmentMap(const cv::Mat& map_to_be_labeled, cv::Mat& segmented_map, double map_resolution_from_subscription,
        double room_area_factor_lower_limit, double room_area_factor_upper_limit, const std::string& classifier_storage_path,
        const std::string& classifier_default_path, bool display_results)
{
	//******************Semantic-labeling function based on AdaBoost*****************************
	//This function calculates single-valued features for every white Pixel in the given occupancy-gridmap and classifies it
	//using the AdaBoost-algorithm from OpenCV. It does the following steps:
	//	I. If the classifiers hasn't been trained before they should load the training-results saved in the
	//	   classifier_models folder
	//	II. Go trough each Pixel of the given map. If this Pixel is white simulate the laser-beams for it and calculate each
	//		of the implemented features.
	//	III. Apply a median-Filter on the labeled map to smooth the output of it.
	//	IV. Find the contours of the segments given by III. by thresholding the map. First set the threshold so high that
	//		only room-areas are shown in the map and find them. Then make these room-areas black and finally set the threshold
	//		a little lower than the hallway-colour. The function only takes contours that are larger than the minimum value
	//		and splits too large hallway-areas into smaller areas by putting random Points into the too large hallway contour
	//		and apply a watershed-algorithm on it. At last the saved room and hallway contours are drawn with a random
	//		colour into the map that hasn't been used already.

	cv::Mat original_map_to_be_labeled = map_to_be_labeled.clone();
	ROS_INFO("Starting to label the map.");
	//***********************I. check if classifiers has already been trained*****************************
	if (!trained_) //classifiers hasn't been trained before so they should be loaded
	{
		// check if path for storing classifier models exists
		boost::filesystem::path storage_path(classifier_storage_path);
		if (boost::filesystem::exists(storage_path) == false)
		{
			if (boost::filesystem::create_directories(storage_path) == false && boost::filesystem::exists(storage_path) == false)
			{
				std::cout << "Error: AdaboostClassifier::segmentMap: Could not create directory " << storage_path << std::endl;
				return;
			}
		}

		std::string filename_room = classifier_storage_path + "semantic_room_boost.xml";
		std::string filename_room_default = classifier_default_path + "semantic_room_boost.xml";
		if (boost::filesystem::exists(boost::filesystem::path(filename_room)) == false)
			boost::filesystem::copy_file(filename_room_default, filename_room);
		loadBoost(room_boost_, filename_room);

		std::string filename_hallway = classifier_storage_path + "semantic_hallway_boost.xml";
		std::string filename_hallway_default = classifier_default_path + "semantic_hallway_boost.xml";
		if (boost::filesystem::exists(boost::filesystem::path(filename_hallway)) == false)
			boost::filesystem::copy_file(filename_hallway_default, filename_hallway);
		loadBoost(hallway_boost_,filename_hallway);

		trained_ = true;
		ROS_INFO("Loaded training results.");
	}

	//*************** II. Go trough each Point and label it as room or hallway.**************************
#pragma omp parallel for
	for (int y = 0; y < original_map_to_be_labeled.rows; y++)
	{
		LaserScannerFeatures lsf;
		for (int x = 0; x < original_map_to_be_labeled.cols; x++)
		{
			if (original_map_to_be_labeled.at<unsigned char>(y, x) == 255)
			{
				std::vector<double> temporary_beams;
				raycasting_.raycasting(original_map_to_be_labeled, cv::Point(x, y), temporary_beams);
				std::vector<float> temporary_features;
				cv::Mat features_mat; //OpenCV expects a 32-floating-point Matrix as feature input
				lsf.get_features(temporary_beams, angles_for_simulation_, cv::Point(x, y), features_mat);
				//classify each Point
#if CV_MAJOR_VERSION == 2
				float room_sum = room_boost_.predict(features_mat, cv::Mat(), cv::Range::all(), false, true);
				float hallway_sum = hallway_boost_.predict(features_mat, cv::Mat(), cv::Range::all(), false, true);
#else
				float room_sum = room_boost_->predict(features_mat, cv::noArray(), cv::ml::Boost::PREDICT_SUM);
				float hallway_sum = hallway_boost_->predict(features_mat, cv::noArray(), cv::ml::Boost::PREDICT_SUM);
#endif
				//get the certanity-values for each class (it shows the probability that it belongs to the given class)
				double room_certanity = (std::exp((double) room_sum)) / (std::exp(-1 * (double) room_sum) + std::exp((double) room_sum));
				double hallway_certanity = (std::exp((double) hallway_certanity))
				        / (std::exp(-1 * (double) hallway_certanity) + std::exp((double) hallway_certanity));
				//make a decision-list and check which class the Point belongs to
				double probability_for_room = room_certanity;
				double probability_for_hallway = hallway_certanity * (1.0 - probability_for_room);
				if (probability_for_room > probability_for_hallway)
				{
					original_map_to_be_labeled.at<unsigned char>(y, x) = 150; //label it as room
				}
				else
				{
					original_map_to_be_labeled.at<unsigned char>(y, x) = 100; //label it as hallway
				}
			}
		}
	}
	std::cout << "labeled all white pixels: " << std::endl;
	//******************** III. Apply a median filter over the image to smooth the results.***************************
	cv::Mat temporary_map = original_map_to_be_labeled.clone();
	cv::medianBlur(temporary_map, temporary_map, 3);
	std::cout << "blurred image" << std::endl;

	//make regions black, that have been black before
	for (int x = 0; x < original_map_to_be_labeled.rows; x++)
	{
		for (int y = 0; y < original_map_to_be_labeled.cols; y++)
		{
			if (original_map_to_be_labeled.at<unsigned char>(x, y) == 0)
			{
				temporary_map.at<unsigned char>(x, y) = 0;
			}
		}
	}
//	cv::imshow("thresholded", temporary_map);
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
#if CV_MAJOR_VERSION<=3
	cv::findContours(temporary_map, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_NONE);
	cv::drawContours(blured_image_for_thresholding, contours, -1, cv::Scalar(0), CV_FILLED); //make the found regions at the original map black, because they have been looked at already
#else
	cv::findContours(temporary_map, contours, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_NONE);
	cv::drawContours(blured_image_for_thresholding, contours, -1, cv::Scalar(0), cv::FILLED); //make the found regions at the original map black, because they have been looked at already
#endif

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
#if CV_MAJOR_VERSION<=3
	cv::findContours(temporary_map, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_NONE);
#else
	cv::findContours(temporary_map, contours, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_NONE);
#endif
	//if the hallway-contours are too big split them into smaller regions, also don't take too small regions
	for (int contour_counter = 0; contour_counter < contours.size(); contour_counter++)
	{
		if (map_resolution_from_subscription * map_resolution_from_subscription * cv::contourArea(contours[contour_counter]) > room_area_factor_upper_limit)
		{
			//Generate a black map to draw the hallway-contour in. Then use this map to ckeck if the generated random Points
			// are inside the contour.
			cv::Mat contour_Map = cv::Mat::zeros(temporary_map.rows, temporary_map.cols, CV_8UC1);
#if CV_MAJOR_VERSION<=3
			cv::drawContours(contour_Map, contours, contour_counter, cv::Scalar(255), CV_FILLED);
#else
			cv::drawContours(contour_Map, contours, contour_counter, cv::Scalar(255), cv::FILLED);
#endif
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
			} while (center_counter <= (map_resolution_from_subscription * map_resolution_from_subscription * cv::contourArea(contours[contour_counter])) / 8);
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
#if CV_MAJOR_VERSION<=3
						cv::circle(temporary_Map_to_wavefront, temporary_watershed_centers[current_center], 2, fill_colour, CV_FILLED);
#else
						cv::circle(temporary_Map_to_wavefront, temporary_watershed_centers[current_center], 2, fill_colour, cv::FILLED);
#endif
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
#if CV_MAJOR_VERSION<=3
				cv::drawContours(segmented_map, saved_room_contours, room, fill_colour, CV_FILLED);
#else
				cv::drawContours(segmented_map, saved_room_contours, room, fill_colour, cv::FILLED);
#endif
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
#if CV_MAJOR_VERSION<=3
				cv::drawContours(segmented_map, saved_hallway_contours, hallway, fill_colour, CV_FILLED);
#else
				cv::drawContours(segmented_map, saved_hallway_contours, hallway, fill_colour, cv::FILLED);
#endif
				already_used_colors.push_back(fill_colour);
				coloured = true;
			}
		} while (!coloured);
	}
	std::cout << "finished small hallway contours" << std::endl;
	//spread the coloured regions to regions, which were too small and aren't drawn into the map
	wavefrontRegionGrowing(segmented_map);
	//make sure previously black pixels are still black
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
