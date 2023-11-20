# ipa_coverage_planning
Algorithms for floor plan segmentation and systematic coverage driving patterns.

If you find this software useful in your work, please cite our corresponding papers:
- R. Bormann, F. Jordan, W. Li, J. Hampp, and M. Hägele. Room Segmentation: Survey, Implementation, and Analysis. In Proceedings of the IEEE International Conference on Robotics and Automation (ICRA), 2016. https://ieeexplore.ieee.org/abstract/document/7487234 , https://publica.fraunhofer.de/entities/publication/0bf23149-75d5-4601-bfce-992d91698862/details
- R. Bormann, F. Jordan, J. Hampp, and M. Hägele. Indoor coverage path planning: Survey, implementation, analysis. In Proceedings of the IEEE International Conference on Robotics and Automation (ICRA), pages 1718–1725, May 2018. https://ieeexplore.ieee.org/abstract/document/8460566 , https://publica.fraunhofer.de/entities/publication/f537c15d-4cbe-4672-9d86-6e756a9ce71b/details

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

