#!/usr/bin/env python
#
# Copyright (c) 2017 Fraunhofer Institute for Manufacturing Engineering and Automation (IPA)
# All rights reserved.
# Unauthorized copying of this file, via any medium is strictly prohibited.
# Proprietary and confidential.

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from nav_msgs.srv import GetPlan, GetPlanResponse


def distance(p1, p2):
    dx = p1.pose.position.x - p2.pose.position.x
    dy = p1.pose.position.y - p2.pose.position.y
    return dx**2 + dy**2


class GlobalPlanProvider(object):

    def __init__(self):
        super(GlobalPlanProvider, self).__init__()
        self.coverage_plan = None
        rospy.loginfo("Starting {}".format(self))
        self.plan_service = rospy.Service("/move_base/global_service_planner", GetPlan, self.get_plan_callback)
        rospy.Subscriber("/room_exploration/room_exploration_client/coverage_path", Path, self.coverage_plan_callback)
        self.goal_publisher = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)

    def coverage_plan_callback(self, msg):
        self.coverage_plan = msg;
        self.goal_publisher.publish(msg.poses[-1])

    def get_plan_callback(self, request):
        rospy.loginfo("Received request")
        response = GetPlanResponse()

        min_index = min(range(len(self.coverage_plan.poses)),
                        key=lambda i: distance(self.coverage_plan.poses[i], request.start))
        self.coverage_plan.poses = self.coverage_plan.poses[min_index:]
        response.plan = self.coverage_plan
        rospy.loginfo("Transmitted plan with {} poses".format(len(response.plan.poses)))

        return response


    def __del__(self):
        self.plan_service.shutdown("The state is over")


def main():
    rospy.init_node("global_plan_provider");
    gpp = GlobalPlanProvider()
    rospy.spin()
    del gpp


if __name__ == '__main__':
    main()
