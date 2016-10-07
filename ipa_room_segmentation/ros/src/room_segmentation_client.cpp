#include <ros/ros.h>
#include <ros/package.h>

#include <string>
#include <vector>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <cv_bridge/cv_bridge.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <ipa_room_segmentation/MapSegmentationAction.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "room_segmentation_client");

	std::vector< std::string > map_names;
	map_names.push_back("lab_ipa");
	map_names.push_back("lab_c_scan");
	map_names.push_back("Freiburg52_scan");
	map_names.push_back("Freiburg79_scan");
	map_names.push_back("lab_b_scan");
	map_names.push_back("lab_intel");
	map_names.push_back("Freiburg101_scan");
	map_names.push_back("lab_d_scan");
	map_names.push_back("lab_f_scan");
	map_names.push_back("lab_a_scan");
	map_names.push_back("NLB");
	map_names.push_back("office_a");
	map_names.push_back("office_b");
	map_names.push_back("office_c");
	map_names.push_back("office_d");
	map_names.push_back("office_e");
	map_names.push_back("office_f");
	map_names.push_back("office_g");
	map_names.push_back("office_h");
	map_names.push_back("office_i");
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

	for (size_t image_index = 0; image_index<map_names.size(); ++image_index)
	{
		std::string image_filename = ros::package::getPath("ipa_room_segmentation") + "/common/files/test_maps/" + map_names[image_index] + ".png";
		cv::Mat map = cv::imread(image_filename.c_str(), 0);
		//make non-white pixels black
		for (int y = 0; y < map.rows; y++)
		{
			for (int x = 0; x < map.cols; x++)
			{
				//find not reachable regions and make them black
				if (map.at<unsigned char>(y, x) < 250)
				{
					map.at<unsigned char>(y, x) = 0;
				}
				//else make it white
				else
				{
					map.at<unsigned char>(y, x) = 255;
				}
			}
		}
//		cv::imshow("map", map);
//		cv::waitKey();
		sensor_msgs::Image labeling;

		cv_bridge::CvImage cv_image;
	//	cv_image.header.stamp = ros::Time::now();
		cv_image.encoding = "mono8";
		cv_image.image = map;
		cv_image.toImageMsg(labeling);
		// create the action client --> "name of server"
		// true causes the client to spin its own thread
		actionlib::SimpleActionClient<ipa_room_segmentation::MapSegmentationAction> ac("/room_segmentation/room_segmentation_server", true);

		ROS_INFO("Waiting for action server to start.");
		// wait for the action server to start
		ac.waitForServer(); //will wait for infinite time

		ROS_INFO("Action server started, sending goal.");
		// send a goal to the action
		ipa_room_segmentation::MapSegmentationGoal goal;
		goal.input_map = labeling;
		goal.map_origin.position.x = 0;
		goal.map_origin.position.y = 0;
		goal.map_resolution = 0.05;
		goal.return_format_in_meter = false;
		goal.return_format_in_pixel = true;
		goal.room_segmentation_algorithm = 5;
		goal.robot_radius = 0.4;
		ac.sendGoal(goal);

		//wait for the action to return
		bool finished_before_timeout = ac.waitForResult(ros::Duration());

		if (finished_before_timeout)
		{
			ROS_INFO("Finished successfully!");
			ipa_room_segmentation::MapSegmentationResultConstPtr result_seg = ac.getResult();

			// display
			cv_bridge::CvImagePtr cv_ptr_obj;
			cv_ptr_obj = cv_bridge::toCvCopy(result_seg->segmented_map, sensor_msgs::image_encodings::TYPE_32SC1);
			cv::Mat segmented_map = cv_ptr_obj->image;
			cv::Mat colour_segmented_map = segmented_map.clone();
			colour_segmented_map.convertTo(colour_segmented_map, CV_8U);
			cv::cvtColor(colour_segmented_map, colour_segmented_map, CV_GRAY2BGR);
			for(size_t i = 1; i <= result_seg->room_information_in_pixel.size(); ++i)
			{
				//choose random color for each room
				int blue = (rand() % 250) + 1;
				int green = (rand() % 250) + 1;
				int red = (rand() % 250) + 1;
				for(size_t u = 0; u < segmented_map.rows; ++u)
				{
					for(size_t v = 0; v < segmented_map.cols; ++v)
					{
						if(segmented_map.at<int>(u,v) == i)
						{
							colour_segmented_map.at<cv::Vec3b>(u,v)[0] = blue;
							colour_segmented_map.at<cv::Vec3b>(u,v)[1] = green;
							colour_segmented_map.at<cv::Vec3b>(u,v)[2] = red;
						}
					}
				}
			}
			//draw the room centers into the map
			for(size_t i = 0; i < result_seg->room_information_in_pixel.size(); ++i)
			{
				cv::Point current_center (result_seg->room_information_in_pixel[i].room_center.x, result_seg->room_information_in_pixel[i].room_center.y);
				cv::circle(colour_segmented_map, current_center, 2, CV_RGB(0,0,255), CV_FILLED);
			}

			cv::imshow("segementation", colour_segmented_map);
			cv::waitKey();
		}
	}

	//exit
	return 0;
}
