#include <ros/ros.h>
#include <ros/package.h>

#include <string>
#include <vector>
#include <iomanip>
#include <iostream>
#include <fstream>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <cv_bridge/cv_bridge.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <ipa_room_segmentation/MapSegmentationAction.h>
#include <ipa_building_navigation/FindRoomSequenceWithCheckpointsAction.h>
#include <ipa_building_navigation/A_star_pathplanner.h>

#include <geometry_msgs/Pose.h>
#include <sensor_msgs/image_encodings.h>


struct EvaluationConfig
{
	int room_segmentation_algorithm_;	// this variable selects the algorithm for room segmentation
											// 1 = morphological segmentation
											// 2 = distance segmentation
											// 3 = Voronoi segmentation
											// 4 = semantic segmentation
	double max_clique_path_length_;		// max A* path length between two rooms that are assigned to the same clique, in [m]

	EvaluationConfig()
	{
		room_segmentation_algorithm_ = 1;
		max_clique_path_length_ = 12.0;
	}

	EvaluationConfig(const int room_segmentation_algorithm, const double max_clique_path_length)
	{
		room_segmentation_algorithm_ = room_segmentation_algorithm;
		max_clique_path_length_ = max_clique_path_length;
	}

	std::string generateConfigurationFolderString() const
	{
		std::stringstream ss;
		ss << "seg" << room_segmentation_algorithm_ << "mcl" << std::setprecision(4) <<  max_clique_path_length_;

		return ss.str();
	}

	std::string getConfigurationString() const
	{
		std::stringstream ss;
		ss << "\n==================================================================================================\n" <<
				"Configuration " << generateConfigurationFolderString() << "\n" <<
				"==================================================================================================\n" <<
				"room_segmentation_algorithm: " << roomSegmentationAlgorithmToString() << " (" << room_segmentation_algorithm_ << ")\n" <<
				"max_clique_path_length: " << max_clique_path_length_ <<
				"\n" << std::endl;

		return ss.str();
	}

	std::string roomSegmentationAlgorithmToString() const
	{
		std::string s = "";
		if (room_segmentation_algorithm_ == 1)
			s = "morphological segmentation";
		else if (room_segmentation_algorithm_ == 2)
			s = "distance segmentation";
		else if (room_segmentation_algorithm_ == 3)
			s = "Voronoi segmentation";
		else if (room_segmentation_algorithm_ == 4)
			s = "semantic segmentation";
		return s;
	}
};

struct EvaluationData
{
	std::string map_name_;		// without file type
	cv::Mat floor_plan_;
	double map_downsampling_factor_;
	float map_resolution_;
	geometry_msgs::Pose map_origin_;
	geometry_msgs::Pose robot_start_position_;
	double robot_radius_;
	std::vector< cv::Point > trash_bin_locations_;

	EvaluationData()
	{
		map_name_ = "";
		floor_plan_ = cv::Mat();
		map_downsampling_factor_ = 0.25;
		map_resolution_ = 0.05;
		robot_radius_ = 0.3;
	}

	EvaluationData(const std::string& map_name, const cv::Mat& floor_plan, const float map_resolution, const double map_downsampling_factor,
			const double robot_radius, const std::vector< cv::Point >& trash_bin_locations)
	{
		map_name_ = map_name;
		floor_plan_ = floor_plan;
		map_downsampling_factor_ = map_downsampling_factor;
		map_resolution_ = map_resolution;
		map_origin_.position.x = 0;
		map_origin_.position.y = 0;
		robot_radius_ = robot_radius;
		trash_bin_locations_ = trash_bin_locations;
		cv::Mat map_eroded;
		cv::erode(floor_plan_, map_eroded, cv::Mat(), cv::Point(-1,-1), robot_radius_/map_resolution_+2);
		cv::Mat distance_map;	//variable for the distance-transformed map, type: CV_32FC1
		cv::distanceTransform(map_eroded, distance_map, CV_DIST_L2, 5);
		cv::convertScaleAbs(distance_map, distance_map);	// conversion to 8 bit image
		bool robot_start_coordinate_set = false;
		for (int v=0; v<map_eroded.rows && robot_start_coordinate_set==false; ++v)
			for (int u=0; u<map_eroded.cols && robot_start_coordinate_set==false; ++u)
				if (map_eroded.at<uchar>(v,u) != 0 && distance_map.at<uchar>(v,u) > 15)
				{
					robot_start_position_.position.x = u*map_resolution_ + map_origin_.position.x;
					robot_start_position_.position.y = v*map_resolution_ + map_origin_.position.y;
					robot_start_coordinate_set = true;
				}
	}
};

class Evaluation
{
public:

	Evaluation(const std::string& test_map_path, const std::string& data_storage_path, const double robot_radius)
	: robot_radius_(robot_radius)
	{
		// prepare relevant floor map data
		std::vector<std::string> map_names;
		map_names.push_back("lab_ipa");
	//	map_names.push_back("freiburg_building101");
	//	map_names.push_back("freiburg_building52");
	//	map_names.push_back("freiburg_building79");
	//	map_names.push_back("intel_map");
	//	map_names.push_back("lab_a");
	//	map_names.push_back("lab_b");
	//	map_names.push_back("lab_c");
	//	map_names.push_back("lab_d");
	//	map_names.push_back("lab_e");
		for (size_t image_index = 0; image_index<map_names.size(); ++image_index)
		{
			std::string image_filename = test_map_path + map_names[image_index] + ".png";
			std::cout << "loading image: " << image_filename << std::endl;
			cv::Mat map = cv::imread(image_filename.c_str(), 0);
			//make non-white pixels black
			for (int y = 0; y < map.rows; y++)
			{
				for (int x = 0; x < map.cols; x++)
				{
					//find not reachable regions and make them black
					if (map.at<unsigned char>(y, x) != 255)
					{
						map.at<unsigned char>(y, x) = 0;
					}
				}
			}

			// read in trash bin locations
			image_filename = test_map_path + map_names[image_index] + "_trashbins.png";
			cv::Mat temp = cv::imread(image_filename.c_str());
			cv::Vec3b blue(255, 0, 0);
			std::vector< cv::Point > trash_bin_locations;
			for (int y = 0; y < temp.rows; y++)
			{
				for (int x = 0; x < temp.cols; x++)
				{
					//find not reachable regions and make them black
					if (temp.at<cv::Vec3b>(y, x) == blue)
					{
						trash_bin_locations.push_back(cv::Point(x,y));
						std::cout << "trash: " << x << ", " << y << std::endl;
					}
				}
			}

			evaluation_data_.push_back(EvaluationData(map_names[image_index], map, 0.05, 0.25, robot_radius_, trash_bin_locations));
		}

		// set configurations
		std::vector< EvaluationConfig > evaluation_configurations;
		setConfigurations(evaluation_configurations);

		// do the evaluation
		for (size_t i=0; i<evaluation_configurations.size(); ++i)
		{
			if (evaluateAllMaps(evaluation_configurations[i], evaluation_data_, data_storage_path) == false)
				return;
		}
	}

	void setConfigurations(std::vector< EvaluationConfig >& evaluation_configurations)
	{
		evaluation_configurations.clear();

		for (int room_segmentation_algorithm=1; room_segmentation_algorithm<=4; ++room_segmentation_algorithm)
		{
			for (double max_clique_path_length = 6.0; max_clique_path_length <= 20.; max_clique_path_length += 2.0)
				evaluation_configurations.push_back(EvaluationConfig(room_segmentation_algorithm, max_clique_path_length));
		}
	}

	bool evaluateAllMaps(const EvaluationConfig& evaluation_configuration, const std::vector< EvaluationData >& evaluation_data_vector, const std::string& data_storage_path)
	{
		// prepare folders for storing results
		const std::string folder_name = evaluation_configuration.generateConfigurationFolderString() + "/";
		const std::string path = data_storage_path + folder_name;
		const std::string command = "mkdir -p " + path;
		int return_value = system(command.c_str());

		AStarPlanner planner;
		for (size_t map_index = 0; map_index<evaluation_data_vector.size(); ++map_index)
		{
			const EvaluationData& evaluation_data = evaluation_data_vector[map_index];

			// 1. retrieve segmentation
			ipa_room_segmentation::MapSegmentationResultConstPtr result_seg;
			if (segmentFloorPlan(evaluation_data, evaluation_configuration, result_seg) == false)
				return false;

			// 2. solve sequence problem
			ipa_building_navigation::FindRoomSequenceWithCheckpointsResultConstPtr result_seq;
			if (computeRoomSequence(evaluation_data, evaluation_configuration, result_seg, result_seq) == false)
				return false;

			// 3. assign trash bins to rooms of the respective segmentation
			std::vector< std::vector<cv::Point> > room_trash_bins(result_seg->room_information_in_pixel.size());
			// converting the map msg in cv format
			cv_bridge::CvImagePtr cv_ptr_obj;
			cv_ptr_obj = cv_bridge::toCvCopy(result_seg->segmented_map, sensor_msgs::image_encodings::TYPE_32SC1);
			cv::Mat segmented_map = cv_ptr_obj->image;
			for (size_t i=0; i<evaluation_data.trash_bin_locations_.size(); ++i)
			{
				int room_index = segmented_map.at<int>(evaluation_data.trash_bin_locations_[i]);
				room_trash_bins[room_index].push_back(evaluation_data.trash_bin_locations_[i]);
			}

			// 4. do the movements
			// get the cliques of roomcenters as cliques
			std::vector<cv::Point> room_centers(result_seg->room_information_in_pixel.size());
			for(size_t i = 0; i < result_seg->room_information_in_pixel.size(); ++i)
			{
				room_centers[i] = cv::Point(result_seg->room_information_in_pixel[i].room_center.x, result_seg->room_information_in_pixel[i].room_center.y);
			}
			cv::Mat downsampled_map;
			planner.downsampleMap(evaluation_data.floor_plan_, downsampled_map, evaluation_data.map_downsampling_factor_, evaluation_data.robot_radius_, evaluation_data.map_resolution_);
			double path_length = 0.;
			const double max_clique_path_length_in_pixel = evaluation_configuration.max_clique_path_length_ / evaluation_data.map_resolution_;
			cv::Point robot_start_position((evaluation_data.robot_start_position_.position.x - evaluation_data.map_origin_.position.x)/evaluation_data.map_resolution_,
								(evaluation_data.robot_start_position_.position.y - evaluation_data.map_origin_.position.y)/evaluation_data.map_resolution_);
			cv::Point robot_position = robot_start_position;
			cv::Point trolley_position((evaluation_data.robot_start_position_.position.x - evaluation_data.map_origin_.position.x)/evaluation_data.map_resolution_,
					(evaluation_data.robot_start_position_.position.y - evaluation_data.map_origin_.position.y)/evaluation_data.map_resolution_);
			for (size_t clique_index = 0; clique_index<result_seq->checkpoints.size(); ++clique_index)
			{
				// move trolley
				//		i) robot to trolley
				path_length += planner.planPath(downsampled_map, evaluation_data.map_downsampling_factor_*robot_position, evaluation_data.map_downsampling_factor_*trolley_position, 1., 0., evaluation_data.map_resolution_);
				// 		ii) trolley to next trolley goal
				cv::Point trolley_goal_position(result_seq->checkpoints[clique_index].checkpoint_position_in_pixel.x, result_seq->checkpoints[clique_index].checkpoint_position_in_pixel.y);
				path_length += planner.planPath(downsampled_map, evaluation_data.map_downsampling_factor_*trolley_position, evaluation_data.map_downsampling_factor_*trolley_goal_position, 1., 0., evaluation_data.map_resolution_);
				trolley_position = trolley_goal_position;
				robot_position = trolley_goal_position;

				// move robot to rooms
				for(size_t room = 0; room < result_seq->checkpoints[clique_index].room_indices.size(); ++room)
				{
					// get next room in sequence
					const int room_index = result_seq->checkpoints[clique_index].room_indices[room];
					cv::Point current_roomcenter = room_centers[room_index];
					// drive to next room
					path_length += planner.planPath(downsampled_map, evaluation_data.map_downsampling_factor_*robot_position, evaluation_data.map_downsampling_factor_*current_roomcenter, 1., 0., evaluation_data.map_resolution_);
					robot_position = current_roomcenter;
					// clear all trash bins: go to trash bin, go back to trolley to empty trash and then drive back to trash bin
					for (size_t t=0; t<room_trash_bins[room_index].size(); ++t)
					{
						// drive robot to trash bin
						path_length += planner.planPath(downsampled_map, evaluation_data.map_downsampling_factor_*robot_position, evaluation_data.map_downsampling_factor_*room_trash_bins[room_index][t], 1., 0., evaluation_data.map_resolution_);
						// drive trash bin to trolley
						path_length += 2. * planner.planPath(downsampled_map, evaluation_data.map_downsampling_factor_*room_trash_bins[room_index][t], evaluation_data.map_downsampling_factor_*trolley_position, 1., 0., evaluation_data.map_resolution_);
						robot_position = room_trash_bins[room_index][t];
					}
				}
			}
			// finally go back to trolley
			path_length += planner.planPath(downsampled_map, evaluation_data.map_downsampling_factor_*robot_position, evaluation_data.map_downsampling_factor_*trolley_position, 1., 0., evaluation_data.map_resolution_);
			// and back to start position
			path_length += planner.planPath(downsampled_map, evaluation_data.map_downsampling_factor_*trolley_position, evaluation_data.map_downsampling_factor_*robot_start_position, 1., 0., evaluation_data.map_resolution_);

			// evaluation
			double path_length_in_meter = path_length * evaluation_data.map_resolution_;
			double robot_speed = 0.25;		// [m/s]
			double time_for_trashbin_manipulation = 150;	// [s], without driving
			double time_for_trolley_manipulation = 90;		// [s], without driving
			double time = path_length_in_meter / robot_speed
						+ time_for_trashbin_manipulation * evaluation_data.trash_bin_locations_.size()
						+ time_for_trolley_manipulation * (result_seq->checkpoints.size()+1);

			// write log file
			std::stringstream output;
			// header
			output << evaluation_configuration.getConfigurationString();
			output << robot_speed << "\t" << time_for_trashbin_manipulation << "\t" << evaluation_data.trash_bin_locations_.size() << "\t"
					<< time_for_trolley_manipulation << "\t" << (result_seq->checkpoints.size()+1) << "\t" << path_length_in_meter << "\t"
					<< time;

			std::string log_filename = path + evaluation_data.map_name_ + "_results.txt";
			std::ofstream file(log_filename.c_str(), std::ios::out);
			if (file.is_open()==true)
				file << output.str();
			file.close();

			// images: segmented_map, sequence_map
			std::string segmented_map_filename = path + evaluation_data.map_name_ + "_segmented.png";
			cv::imwrite(segmented_map_filename.c_str(), segmented_map);
		}

		return true;
	}

	bool segmentFloorPlan(const EvaluationData& evaluation_data, const EvaluationConfig& evaluation_configuration,
			ipa_room_segmentation::MapSegmentationResultConstPtr& result_seg)
	{
		sensor_msgs::Image map_msg;
		cv_bridge::CvImage cv_image;
		cv_image.encoding = "mono8";
		cv_image.image = evaluation_data.floor_plan_;
		cv_image.toImageMsg(map_msg);
		actionlib::SimpleActionClient<ipa_room_segmentation::MapSegmentationAction> ac_seg("/room_segmentation/room_segmentation_server", true);
		ROS_INFO("Waiting for action server '/room_segmentation/room_segmentation_server' to start.");
		ac_seg.waitForServer(); // wait for the action server to start, will wait for infinite time

		ROS_INFO("Action server started, sending goal.");
		// send a goal to the action
		ipa_room_segmentation::MapSegmentationGoal goal_seg;
		goal_seg.input_map = map_msg;
		goal_seg.map_origin = evaluation_data.map_origin_;
		goal_seg.map_resolution = evaluation_data.map_resolution_;
		goal_seg.return_format_in_meter = false;
		goal_seg.return_format_in_pixel = true;
		goal_seg.room_segmentation_algorithm = evaluation_configuration.room_segmentation_algorithm_;
		ac_seg.sendGoal(goal_seg);

		//wait for the action to return
		bool finished_before_timeout = ac_seg.waitForResult(ros::Duration(600.0));
		if (finished_before_timeout == false)
		{
			ROS_ERROR("Timeout on room segmentation.");
			return false;
		}
		result_seg = ac_seg.getResult();
		ROS_INFO("Finished segmentation successfully!");

		return true;
	}

	bool computeRoomSequence(const EvaluationData& evaluation_data, const EvaluationConfig& evaluation_configuration,
			ipa_room_segmentation::MapSegmentationResultConstPtr& result_seg,
			ipa_building_navigation::FindRoomSequenceWithCheckpointsResultConstPtr& result_seq)
	{
		sensor_msgs::Image map_msg;
		cv_bridge::CvImage cv_image;
		cv_image.encoding = "mono8";
		cv_image.image = evaluation_data.floor_plan_;
		cv_image.toImageMsg(map_msg);

		actionlib::SimpleActionClient<ipa_building_navigation::FindRoomSequenceWithCheckpointsAction> ac_seq("/room_sequence_planning/room_sequence_planning_server", true);
		ROS_INFO("Waiting for action server '/room_sequence_planning/room_sequence_planning_server' to start.");
		// wait for the action server to start
		ac_seq.waitForServer(); //will wait for infinite time

		ROS_INFO("Action server started, sending goal_seg.");
		// send a goal_seg to the action
		ipa_building_navigation::FindRoomSequenceWithCheckpointsGoal goal_seq;
		goal_seq.input_map = map_msg;
		goal_seq.map_resolution = evaluation_data.map_resolution_;
		goal_seq.map_origin = evaluation_data.map_origin_;
		goal_seq.room_information_in_pixel = result_seg->room_information_in_pixel;
		goal_seq.max_clique_path_length = evaluation_configuration.max_clique_path_length_;
		goal_seq.map_downsampling_factor = evaluation_data.map_downsampling_factor_;
		goal_seq.robot_radius = robot_radius_;
		goal_seq.robot_start_coordinate = evaluation_data.robot_start_position_;
		ac_seq.sendGoal(goal_seq);

		//wait for the action to return
		bool finished_before_timeout = ac_seq.waitForResult(ros::Duration(600.0));
		if (finished_before_timeout == false)
		{
			ROS_ERROR("Timeout on room sequence planning.");
			return false;
		}
		result_seq = ac_seq.getResult();
		ROS_INFO("Finished sequence planning successfully!");

		return true;
	}

private:

	std::vector< EvaluationData > evaluation_data_;

	const double robot_radius_;
};



int main(int argc, char **argv)
{
	ros::init(argc, argv, "room_sequence_planning_client");

	const std::string test_map_path = ros::package::getPath("ipa_room_segmentation") + "/common/files/test_maps/";
	const std::string data_storage_path = "room_sequence_planning/";
	Evaluation ev(test_map_path, data_storage_path, 0.3);

	//exit
	return 0;
}
