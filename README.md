# ipa_coverage_planning
Algorithms for floor plan segmentation and systematic coverage driving patterns

## ROS Distro Support

|         | Indigo | Jade | Kinetic | Melodic | Noetic |
|:-------:|:------:|:----:|:-------:|:-------:|:-------:|
| Branch  | [`indigo_dev`](https://github.com/ipa320/ipa_coverage_planning/tree/indigo_dev) | [`indigo_dev`](https://github.com/ipa320/ipa_coverage_planning/tree/indigo_dev) | [`indigo_dev`](https://github.com/ipa320/ipa_coverage_planning/tree/indigo_dev) | [`melodic_dev`](https://github.com/ipa320/ipa_coverage_planning/tree/melodic_dev) |[`noetic_dev`](https://github.com/ipa320/ipa_coverage_planning/tree/noetic_dev) |
| Status  |  not supported | not supported |  EOL | supported | supported |
| Version | [version](http://repositories.ros.org/status_page/ros_indigo_default.html?q=ipa_coverage_planning) | [version](http://repositories.ros.org/status_page/ros_jade_default.html?q=ipa_coverage_planning) | [version](http://repositories.ros.org/status_page/ros_kinetic_default.html?q=ipa_coverage_planning) | [version](http://repositories.ros.org/status_page/ros_melodic_default.html?q=ipa_coverage_planning) | [version](http://repositories.ros.org/status_page/ros_noetic_default.html?q=ipa_coverage_planning)

## Travis - Continuous Integration

Status: [![Build Status](https://travis-ci.org/ipa320/ipa_coverage_planning.svg?branch=indigo_dev)](https://travis-ci.org/ipa320/ipa_coverage_planning)


## Quick start

1. Bring up your robot or launch the turtlebot3 simulation

1. Start the room exploration action server:

    ```
    roslaunch ipa_room_exploration room_exploration_action_server.launch
    ```


1. Start the room exploration action client:

    ```
    roslaunch ipa_room_exploration room_exploration_client.launch  robot_env:=your-robot-env use_test_maps:=false
    ```

