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

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Point.h>

#include <tf/transform_listener.h>

#include <boost/thread/mutex.hpp>


class CoverageMonitor
{
public:
	CoverageMonitor(ros::NodeHandle nh)
	{
		// todo: parameters
		map_frame_ = "map";
		robot_frame_ = "base_link";
		robot_trajectory_recording_active_ = true;	// todo: make a service for activating/deactivating

		coverage_marker_pub_ = nh.advertise<visualization_msgs::MarkerArray>("coverage_marker", 1);
		target_trajectory_marker_pub_ = nh.advertise<visualization_msgs::MarkerArray>("target_trajectory_marker", 1);

		target_trajectory_sub_ = nh.subscribe<geometry_msgs::TransformStamped>("target_trajectory_monitor", 1, &CoverageMonitor::targetTrajectoryCallback, this);

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
		coverage_marker_msg.scale.x = 0.3;		// this is the line width	// todo: take coverage_radius
		coverage_marker_msg.scale.y = 1.0;
		coverage_marker_msg.scale.z = 1.0;
		// Set the color -- be sure to set alpha to something non-zero!
		coverage_marker_msg.color.r = 0.0f;
		coverage_marker_msg.color.g = 1.0f;
		coverage_marker_msg.color.b = 0.0f;
		coverage_marker_msg.color.a = 1.0;
		coverage_marker_msg.lifetime = ros::Duration();
		geometry_msgs::Point p; p.x=0; p.y=0; p.z=0;
		coverage_marker_msg.points.push_back(p);

		// prepare target_trajectory_marker_msg message
		visualization_msgs::Marker target_trajectory_marker_msg;
		// Set the frame ID and timestamp.  See the TF tutorials for information on these.
		target_trajectory_marker_msg.header.frame_id = "/map";
		target_trajectory_marker_msg.header.stamp = ros::Time::now();
		// Set the namespace and id for this marker.  This serves to create a unique ID
		// Any marker sent with the same namespace and id will overwrite the old one
		target_trajectory_marker_msg.ns = "target_trajectory_marker";
		target_trajectory_marker_msg.id = 0;
		// Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
		target_trajectory_marker_msg.type = visualization_msgs::Marker::LINE_STRIP;
		// Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
		target_trajectory_marker_msg.action = visualization_msgs::Marker::ADD;
		// Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
		target_trajectory_marker_msg.pose.position.x = 0;
		target_trajectory_marker_msg.pose.position.y = 0;
		target_trajectory_marker_msg.pose.position.z = 0;
		target_trajectory_marker_msg.pose.orientation.x = 0.0;
		target_trajectory_marker_msg.pose.orientation.y = 0.0;
		target_trajectory_marker_msg.pose.orientation.z = 0.0;
		target_trajectory_marker_msg.pose.orientation.w = 1.0;
		// Set the scale of the marker -- 1x1x1 here means 1m on a side
		target_trajectory_marker_msg.scale.x = 0.1;		// this is the line width	// todo: take coverage_radius
		target_trajectory_marker_msg.scale.y = 1.0;
		target_trajectory_marker_msg.scale.z = 1.0;
		// Set the color -- be sure to set alpha to something non-zero!
		target_trajectory_marker_msg.color.r = 0.0f;
		target_trajectory_marker_msg.color.g = 0.0f;
		target_trajectory_marker_msg.color.b = 1.0f;
		target_trajectory_marker_msg.color.a = 1.0;
		target_trajectory_marker_msg.lifetime = ros::Duration();
		target_trajectory_marker_msg.points.push_back(p);

		// cyclically publish marker messages
		ros::Rate r(10);
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
					robot_trajectory_vector_.push_back(transform);
				}
			}

			// update and publish coverage_marker_msg
			coverage_marker_msg.header.stamp = ros::Time::now();
			geometry_msgs::Point p; p.x=coverage_marker_msg.points.back().x+0.02; p.y=0; p.z=0;
			coverage_marker_msg.points.push_back(p);
			coverage_marker_pub_.publish(coverage_marker_msg);

			// update and publish target_trajectory_marker_msg
			{
				// secure this access with a mutex
				boost::mutex::scoped_lock lock(robot_target_trajectory_vector_mutex_);

				target_trajectory_marker_msg.header.stamp = ros::Time::now();
				p.x=target_trajectory_marker_msg.points.back().x+0.02; p.y=0; p.z=0;
				target_trajectory_marker_msg.points.push_back(p);
				target_trajectory_marker_pub_.publish(target_trajectory_marker_msg);
			}

			r.sleep();
			ros::spinOnce();
		}
	}

	// receive another trajectory target
	void targetTrajectoryCallback(const geometry_msgs::TransformStamped::ConstPtr& trajectory_msg)
	{
		// secure this access with a mutex
		boost::mutex::scoped_lock lock(robot_target_trajectory_vector_mutex_);

		tf::StampedTransform transform;
		tf::transformStampedMsgToTF(*trajectory_msg, transform);
		robot_target_trajectory_vector_.push_back(transform);
	}

protected:
	ros::Publisher coverage_marker_pub_;				// visualization of the coverage trajectory
	ros::Publisher target_trajectory_marker_pub_;		// visualization of the target trajectory

	tf::TransformListener transform_listener_;
	ros::Subscriber target_trajectory_sub_;			// receives messages with StampedTransforms of the target trajectory

	cv::Point2d coverage_circle_offset_px_;			// the offset of the coverage circle from the robot center

	std::string map_frame_;
	std::string robot_frame_;

	bool robot_trajectory_recording_active_;		// the robot trajectory is only recorded if this is true (can be set from outside)

	std::vector<tf::StampedTransform> robot_trajectory_vector_;				// vector of actual robot trajectory
	std::vector<tf::StampedTransform> robot_target_trajectory_vector_;		// vector of target robot trajectory
	boost::mutex robot_target_trajectory_vector_mutex_; // secures read and write operations on robot_target_trajectory_vector_
};


int main(int argc, char **argv)
{
	ros::init(argc, argv, "coverage_monitor");
	ros::NodeHandle nh;

	CoverageMonitor cm(nh);

	ros::spin();

	return 0;
}
