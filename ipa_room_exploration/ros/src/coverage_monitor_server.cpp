/*!
 *****************************************************************
 * \file
 *
 * \note
 * Copyright (c) 2016 \n
 * Fraunhofer Institute for Manufacturing Engineering
 * and Automation (IPA) \n\n
 *
 *****************************************************************
 *
 * \note
 * Project name: Care-O-bot
 * \note
 * ROS stack name: autopnp
 * \note
 * ROS package name: ipa_room_exploration
 *
 * \author
 * Author: Richard Bormann
 * \author
 * Supervised by:
 *
 * \date Date of creation: 01.2018
 *
 * \brief
 *
 *
 *****************************************************************
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer. \n
 * - Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution. \n
 * - Neither the name of the Fraunhofer Institute for Manufacturing
 * Engineering and Automation (IPA) nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission. \n
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License LGPL for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License LGPL along with this program.
 * If not, see <http://www.gnu.org/licenses/>.
 *
 ****************************************************************/


#include <vector>
#include <string>

#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>
#include <dynamic_reconfigure/BoolParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>
#include <ipa_room_exploration/CoverageMonitorConfig.h>
#include <ipa_room_exploration/coverage_check_server.h>
#include <ipa_building_msgs/CheckCoverage.h>

#include <visualization_msgs/Marker.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Point.h>
#include <std_srvs/Trigger.h>

#include <tf/transform_listener.h>

#include <boost/thread/mutex.hpp>

#include <opencv2/opencv.hpp>


class CoverageMonitor
{
public:
	CoverageMonitor(ros::NodeHandle nh) :
		node_handle_(nh)
	{
		// dynamic reconfigure
		coverage_monitor_dynamic_reconfigure_server_.setCallback(boost::bind(&CoverageMonitor::dynamicReconfigureCallback, this, _1, _2));

		// Parameters
		std::cout << "\n--------------------------\nCoverage Monitor Parameters:\n--------------------------\n";
		node_handle_.param<std::string>("map_frame", map_frame_, "map");
		std::cout << "coverage_monitor/map_frame = " << map_frame_ << std::endl;
		node_handle_.param<std::string>("robot_frame", robot_frame_, "base_link");
		std::cout << "coverage_monitor/robot_frame = " << robot_frame_ << std::endl;
		node_handle_.param("coverage_radius", coverage_radius_, 0.25);
		std::cout << "coverage_monitor/coverage_radius = " << coverage_radius_ << std::endl;
		coverage_circle_offset_transform_.setIdentity();
		std::vector<double> temp;
		node_handle_.getParam("coverage_circle_offset_translation", temp);
		if (temp.size() == 3)
			coverage_circle_offset_transform_.setOrigin(tf::Vector3(temp[0], temp[1], temp[2]));
		else
			coverage_circle_offset_transform_.setOrigin(tf::Vector3(0.29035, -0.114, 0.));
		node_handle_.param("robot_trajectory_recording_active", robot_trajectory_recording_active_, false);
		std::cout << "coverage_monitor/robot_trajectory_recording_active = " << robot_trajectory_recording_active_ << std::endl;

		// setup publishers and subscribers
		coverage_marker_pub_ = node_handle_.advertise<visualization_msgs::Marker>("coverage_marker", 1);
		computed_trajectory_marker_pub_ = node_handle_.advertise<visualization_msgs::Marker>("computed_trajectory_marker", 1);
		commanded_trajectory_marker_pub_ = node_handle_.advertise<visualization_msgs::Marker>("commanded_trajectory_marker", 1);

		computed_trajectory_sub_ = node_handle_.subscribe<geometry_msgs::TransformStamped>("computed_target_trajectory_monitor", 500, &CoverageMonitor::computedTrajectoryCallback, this);
		commanded_trajectory_sub_ = node_handle_.subscribe<geometry_msgs::TransformStamped>("commanded_target_trajectory_monitor", 500, &CoverageMonitor::commandedTrajectoryCallback, this);

		// setup services
		start_coverage_monitoring_srv_ = node_handle_.advertiseService("start_coverage_monitoring", &CoverageMonitor::startCoverageMonitoringCallback, this);
		stop_coverage_monitoring_srv_ = node_handle_.advertiseService("stop_coverage_monitoring", &CoverageMonitor::stopCoverageMonitoringCallback, this);
		get_coverage_image_srv_ = node_handle_.advertiseService("get_coverage_image", &CoverageMonitor::getCoverageImageCallback, this);

		// prepare coverage_marker_msg message
		visualization_msgs::Marker coverage_marker_msg;
		// Set the frame ID and timestamp.  See the TF tutorials for information on these.
		coverage_marker_msg.header.frame_id = "/map";
		coverage_marker_msg.header.stamp = ros::Time::now();
		// Set the namespace and id for this marker.  This serves to create a unique ID
		// Any marker sent with the same namespace and id will overwrite the old one
		coverage_marker_msg.ns = "coverage_marker";
		coverage_marker_msg.id = 0;
		// Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
		coverage_marker_msg.type = visualization_msgs::Marker::LINE_STRIP;
		// Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
		coverage_marker_msg.action = visualization_msgs::Marker::ADD;
		// Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
		coverage_marker_msg.pose.position.x = 0;
		coverage_marker_msg.pose.position.y = 0;
		coverage_marker_msg.pose.position.z = 0;
		coverage_marker_msg.pose.orientation.x = 0.0;
		coverage_marker_msg.pose.orientation.y = 0.0;
		coverage_marker_msg.pose.orientation.z = 0.0;
		coverage_marker_msg.pose.orientation.w = 1.0;
		// Set the scale of the marker -- 1x1x1 here means 1m on a side
		coverage_marker_msg.scale.x = 2*coverage_radius_;		// this is the line width
		coverage_marker_msg.scale.y = 1.0;
		coverage_marker_msg.scale.z = 1.0;
		// Set the color -- be sure to set alpha to something non-zero!
		coverage_marker_msg.color.r = 0.0f;
		coverage_marker_msg.color.g = 1.0f;
		coverage_marker_msg.color.b = 0.0f;
		coverage_marker_msg.color.a = 0.33;
		coverage_marker_msg.lifetime = ros::Duration();
		geometry_msgs::Point p; p.x=0; p.y=0; p.z=0;
		coverage_marker_msg.points.push_back(p);

		// prepare computed_trajectory_marker_msg message
		visualization_msgs::Marker computed_trajectory_marker_msg;
		// Set the frame ID and timestamp.  See the TF tutorials for information on these.
		computed_trajectory_marker_msg.header.frame_id = "/map";
		computed_trajectory_marker_msg.header.stamp = ros::Time::now();
		// Set the namespace and id for this marker.  This serves to create a unique ID
		// Any marker sent with the same namespace and id will overwrite the old one
		computed_trajectory_marker_msg.ns = "computed_trajectory_marker";
		computed_trajectory_marker_msg.id = 0;
		// Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
		computed_trajectory_marker_msg.type = visualization_msgs::Marker::LINE_STRIP;
		// Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
		computed_trajectory_marker_msg.action = visualization_msgs::Marker::ADD;
		// Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
		computed_trajectory_marker_msg.pose.position.x = 0;
		computed_trajectory_marker_msg.pose.position.y = 0;
		computed_trajectory_marker_msg.pose.position.z = 0.2;
		computed_trajectory_marker_msg.pose.orientation.x = 0.0;
		computed_trajectory_marker_msg.pose.orientation.y = 0.0;
		computed_trajectory_marker_msg.pose.orientation.z = 0.0;
		computed_trajectory_marker_msg.pose.orientation.w = 1.0;
		// Set the scale of the marker -- 1x1x1 here means 1m on a side
		computed_trajectory_marker_msg.scale.x = 0.06;		// this is the line width
		computed_trajectory_marker_msg.scale.y = 1.0;
		computed_trajectory_marker_msg.scale.z = 1.0;
		// Set the color -- be sure to set alpha to something non-zero!
		computed_trajectory_marker_msg.color.r = 1.0f;
		computed_trajectory_marker_msg.color.g = 0.0f;
		computed_trajectory_marker_msg.color.b = 0.0f;
		computed_trajectory_marker_msg.color.a = 0.8;
		computed_trajectory_marker_msg.lifetime = ros::Duration();
		computed_trajectory_marker_msg.points.push_back(p);

		// prepare commanded_trajectory_marker_msg message
		visualization_msgs::Marker commanded_trajectory_marker_msg;
		// Set the frame ID and timestamp.  See the TF tutorials for information on these.
		commanded_trajectory_marker_msg.header.frame_id = "/map";
		commanded_trajectory_marker_msg.header.stamp = ros::Time::now();
		// Set the namespace and id for this marker.  This serves to create a unique ID
		// Any marker sent with the same namespace and id will overwrite the old one
		commanded_trajectory_marker_msg.ns = "commanded_trajectory_marker";
		commanded_trajectory_marker_msg.id = 0;
		// Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
		commanded_trajectory_marker_msg.type = visualization_msgs::Marker::LINE_STRIP;
		// Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
		commanded_trajectory_marker_msg.action = visualization_msgs::Marker::ADD;
		// Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
		commanded_trajectory_marker_msg.pose.position.x = 0;
		commanded_trajectory_marker_msg.pose.position.y = 0;
		commanded_trajectory_marker_msg.pose.position.z = 0.4;
		commanded_trajectory_marker_msg.pose.orientation.x = 0.0;
		commanded_trajectory_marker_msg.pose.orientation.y = 0.0;
		commanded_trajectory_marker_msg.pose.orientation.z = 0.0;
		commanded_trajectory_marker_msg.pose.orientation.w = 1.0;
		// Set the scale of the marker -- 1x1x1 here means 1m on a side
		commanded_trajectory_marker_msg.scale.x = 0.03;		// this is the line width
		commanded_trajectory_marker_msg.scale.y = 1.0;
		commanded_trajectory_marker_msg.scale.z = 1.0;
		// Set the color -- be sure to set alpha to something non-zero!
		commanded_trajectory_marker_msg.color.r = 0.0f;
		commanded_trajectory_marker_msg.color.g = 0.0f;
		commanded_trajectory_marker_msg.color.b = 1.0f;
		commanded_trajectory_marker_msg.color.a = 0.8;
		commanded_trajectory_marker_msg.lifetime = ros::Duration();
		commanded_trajectory_marker_msg.points.push_back(p);

		// cyclically publish marker messages
		ros::AsyncSpinner spinner(2);	// asynch. spinner (2) is needed to call dynamic reconfigure from this node without blocking the node
		spinner.start();
		ros::Rate r(5);
//		int index = 0;
		while (ros::ok())
		{
			// receive the current robot pose
			if (robot_trajectory_recording_active_ == true)
			{
				ros::Time time = ros::Time(0);
				bool transform_available = transform_listener_.waitForTransform(map_frame_, robot_frame_, time, ros::Duration(0.5));
				if (transform_available == true)
				{
					tf::StampedTransform transform;
					transform_listener_.lookupTransform(map_frame_, robot_frame_, time, transform);
					{
						boost::mutex::scoped_lock lock(robot_trajectory_vector_mutex_);
						robot_trajectory_vector_.push_back(transform);
					}
				}
//				// this can be used for testing if no data is available
//				tf::StampedTransform transform(tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.1*index, 0., 0.)), ros::Time::now(), map_frame_, robot_frame_);
//				robot_trajectory_vector_.push_back(transform);
//				robot_computed_trajectory_vector_.push_back(transform);
//				robot_commanded_trajectory_vector_.push_back(transform);
//				++index;
			}

			// update and publish coverage_marker_msg
			coverage_marker_msg.header.stamp = ros::Time::now();
			coverage_marker_msg.points.resize(robot_trajectory_vector_.size());
			for (size_t i=0; i<robot_trajectory_vector_.size(); ++i)
				tf::pointTFToMsg((robot_trajectory_vector_[i]*coverage_circle_offset_transform_).getOrigin(), coverage_marker_msg.points[i]);
			coverage_marker_pub_.publish(coverage_marker_msg);

			// update and publish computed_trajectory_marker_msg
			{
				// secure this access with a mutex
				boost::mutex::scoped_lock lock(robot_computed_trajectory_vector_mutex_);

				computed_trajectory_marker_msg.header.stamp = ros::Time::now();
				computed_trajectory_marker_msg.points.resize(robot_computed_trajectory_vector_.size());
				for (size_t i=0; i<robot_computed_trajectory_vector_.size(); ++i)
					tf::pointTFToMsg((robot_computed_trajectory_vector_[i]*coverage_circle_offset_transform_).getOrigin(), computed_trajectory_marker_msg.points[i]);
			}
			computed_trajectory_marker_pub_.publish(computed_trajectory_marker_msg);

			// update and publish commanded_trajectory_marker_msg
			{
				// secure this access with a mutex
				boost::mutex::scoped_lock lock(robot_commanded_trajectory_vector_mutex_);

				commanded_trajectory_marker_msg.header.stamp = ros::Time::now();
				commanded_trajectory_marker_msg.points.resize(robot_commanded_trajectory_vector_.size());
				for (size_t i=0; i<robot_commanded_trajectory_vector_.size(); ++i)
					tf::pointTFToMsg((robot_commanded_trajectory_vector_[i]*coverage_circle_offset_transform_).getOrigin(), commanded_trajectory_marker_msg.points[i]);
			}
			commanded_trajectory_marker_pub_.publish(commanded_trajectory_marker_msg);

			r.sleep();
		}
	}

	// receive computed trajectory targets
	void computedTrajectoryCallback(const geometry_msgs::TransformStamped::ConstPtr& trajectory_msg)
	{
		tf::StampedTransform transform;
		tf::transformStampedMsgToTF(*trajectory_msg, transform);
		{
			// secure this access with a mutex
			boost::mutex::scoped_lock lock(robot_computed_trajectory_vector_mutex_);
			robot_computed_trajectory_vector_.push_back(transform);
		}
	}

	// receive commanded trajectory targets
	void commandedTrajectoryCallback(const geometry_msgs::TransformStamped::ConstPtr& trajectory_msg)
	{
		tf::StampedTransform transform;
		tf::transformStampedMsgToTF(*trajectory_msg, transform);
		{
			// secure this access with a mutex
			boost::mutex::scoped_lock lock(robot_commanded_trajectory_vector_mutex_);
			robot_commanded_trajectory_vector_.push_back(transform);
		}
	}

	bool startCoverageMonitoringCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
	{
		std::cout << "CoverageMonitor::startCoverageMonitoringCallback." << std::endl;
		robot_trajectory_recording_active_ = true;
		internalDynamicReconfigureUpdate();		// update dynamic reconfigure
		res.success = true;
		return true;
	}

	bool stopCoverageMonitoringCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
	{
		std::cout << "CoverageMonitor::stopCoverageMonitoringCallback." << std::endl;
		robot_trajectory_recording_active_ = false;
		internalDynamicReconfigureUpdate();		// update dynamic reconfigure
		res.success = true;
		return true;
	}

	void internalDynamicReconfigureUpdate()
	{
		// update dynamic reconfigure
		dynamic_reconfigure::ReconfigureRequest srv_req;
		dynamic_reconfigure::ReconfigureResponse srv_resp;
		dynamic_reconfigure::BoolParameter enable_param;
		enable_param.name = "robot_trajectory_recording_active";
		enable_param.value = robot_trajectory_recording_active_;
		srv_req.config.bools.push_back(enable_param);
		if (ros::service::call("~/set_parameters", srv_req, srv_resp))
			ROS_INFO("Update of dynamic reconfigure parameters succeeded");
		else
			ROS_INFO("Update of dynamic reconfigure parameters failed");
	}

	// callback function for dynamic reconfigure
	void dynamicReconfigureCallback(ipa_room_exploration::CoverageMonitorConfig &config, uint32_t level)
	{
		std::cout << "######################################################################################" << std::endl;
		std::cout << "Dynamic reconfigure request:" << std::endl;


		map_frame_ = config.map_frame;
		std::cout << "coverage_monitor/map_frame_ = " << map_frame_ << std::endl;
		robot_frame_ = config.robot_frame;
		std::cout << "coverage_monitor/robot_frame_ = " << robot_frame_ << std::endl;

		coverage_radius_ = config.coverage_radius;
		std::cout << "coverage_monitor/coverage_radius_ = " << coverage_radius_ << std::endl;
		double temp[3];
		temp[0] = config.coverage_circle_offset_transform_x;
		temp[1] = config.coverage_circle_offset_transform_y;
		temp[2] = config.coverage_circle_offset_transform_z;
		coverage_circle_offset_transform_.setOrigin(tf::Vector3(temp[0], temp[1], temp[2]));
		std::cout << "coverage_monitor/coverage_circle_offset_transform_ = (" << temp[0] << ", " << temp[1] << ", " << temp[2] << ")" << std::endl;

		robot_trajectory_recording_active_ = config.robot_trajectory_recording_active;
		std::cout << "coverage_monitor/robot_trajectory_recording_active_ = " << robot_trajectory_recording_active_ << std::endl;

		std::cout << "######################################################################################" << std::endl;
	}

	bool getCoverageImageCallback(ipa_building_msgs::CheckCoverage::Request &req, ipa_building_msgs::CheckCoverage::Response &res)
	{
		std::cout << "req.input_map.encoding:" << req.input_map.encoding << std::endl;
		std::cout << "CoverageMonitor::getCoverageImageCallback." << std::endl;

		// insert path to request message
		{
			boost::mutex::scoped_lock lock(robot_trajectory_vector_mutex_);
			req.path.resize(robot_trajectory_vector_.size());
			for (size_t i=0; i<robot_trajectory_vector_.size(); ++i)
			{
				req.path[i].x = robot_trajectory_vector_[i].getOrigin().getX();
				req.path[i].y = robot_trajectory_vector_[i].getOrigin().getY();
				req.path[i].theta = tf::getYaw(robot_trajectory_vector_[i].getRotation());
			}
		}

		// call coverage check server
		CoverageCheckServer coverage_checker;
		coverage_checker.checkCoverage(req, res);

		std::cout << "res.coverage_map.encoding=" << res.coverage_map.encoding << std::endl;
		// simplify returned coverage_map (remove room pixels [255] and remap the covered pixels from 127 to 255)
		cv_bridge::CvImagePtr cv_ptr_obj;
		cv_ptr_obj = cv_bridge::toCvCopy(res.coverage_map, res.coverage_map.encoding);	//sensor_msgs::image_encodings::MONO8);
		cv::Mat coverage_map = cv_ptr_obj->image;

		for (int v=0; v<coverage_map.rows; ++v)
		{
			for (int u=0; u<coverage_map.cols; ++u)
			{
				if (coverage_map.at<uchar>(v,u) == 255)
					coverage_map.at<uchar>(v,u) = 0;
				else if (coverage_map.at<uchar>(v,u) == 127)
					coverage_map.at<uchar>(v,u) = 255;
			}
		}
		cv_bridge::CvImage cv_image;
		cv_image.header.stamp = res.coverage_map.header.stamp;
		cv_image.encoding = res.coverage_map.encoding;
		cv_image.image = coverage_map;
		cv_image.toImageMsg(res.coverage_map);

		return true;
	}

protected:
	ros::NodeHandle node_handle_;

	ros::Publisher coverage_marker_pub_;				// visualization of the coverage trajectory
	ros::Publisher computed_trajectory_marker_pub_;		// visualization of the computed target trajectory
	ros::Publisher commanded_trajectory_marker_pub_;	// visualization of the commanded target trajectory

	tf::TransformListener transform_listener_;
	ros::Subscriber computed_trajectory_sub_;			// receives messages with StampedTransforms of the computed target trajectory
	ros::Subscriber commanded_trajectory_sub_;			// receives messages with StampedTransforms of the commanded target trajectory

	ros::ServiceServer start_coverage_monitoring_srv_;	// service for starting monitoring the robot trajectory
	ros::ServiceServer stop_coverage_monitoring_srv_;	// service for stopping monitoring the robot trajectory
	ros::ServiceServer get_coverage_image_srv_;			// service for returning an image of the covered area

	dynamic_reconfigure::Server<ipa_room_exploration::CoverageMonitorConfig> coverage_monitor_dynamic_reconfigure_server_;

	tf::Transform coverage_circle_offset_transform_;		// the offset of the coverage circle from the robot center
	double coverage_radius_;			// radius of the circular coverage device

	std::string map_frame_;			// name of the map coordinate system
	std::string robot_frame_;		// name of the robot base frame

	bool robot_trajectory_recording_active_;		// the robot trajectory is only recorded if this is true (can be set from outside)

	boost::mutex robot_trajectory_vector_mutex_;				// secures read and write operations on robot_trajectory_vector_
	std::vector<tf::StampedTransform> robot_trajectory_vector_;				// vector of actual robot trajectory
	std::vector<tf::StampedTransform> robot_computed_trajectory_vector_;	// vector of computed target robot trajectory
	boost::mutex robot_computed_trajectory_vector_mutex_;		// secures read and write operations on robot_computed_trajectory_vector_
	std::vector<tf::StampedTransform> robot_commanded_trajectory_vector_;	// vector of commanded target robot trajectory
	boost::mutex robot_commanded_trajectory_vector_mutex_;		// secures read and write operations on robot_commanded_trajectory_vector_
};


int main(int argc, char **argv)
{
	ros::init(argc, argv, "coverage_monitor");
	ros::NodeHandle nh("~");

	CoverageMonitor cm(nh);

	return 0;
}
