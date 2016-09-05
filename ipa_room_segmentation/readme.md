########### General Procedure ###########

1. Change the algorithm parameters in ros/launch/room_segmentation_action_server_params.yaml in ros/launch to the wanted algorithms and settings.
	1.1 The parameter room_segmentation_algorithm directly changes the used segmentation algorithm directly.
	1.2 The parameter display_segmented_map is used to specify if you want a window to pop up after the given map has been segmented, showing the results. To continue the program press any Button.
	1.3 Train_semantic and train_vrf are Booleans showing if the semantic or voronoi random field segmentation should be trained. For details of this training method see the further document.
2. Start the action server using the file /ros/launch/room_segmentation_action_server.launch, which executes the /ros/src/room_segmentation_server.cpp file
3. Start an action client, which sends a goal to the segmentation action server, corresponding to the MapSegmentation.action message, which lies in ipa_building_msgs/action.

The algorithms are implemented in common/src, using the headers in common/include/ipa_room_segmentation. Each algorithm has its own class, so if you want to use it alone you have to include the header for the algorithm you want to use and make a new object. Then you have to execute the segmentationAlgorithm (semanticLabeling for adaboost_classifier.cpp), which segments the map.



########### Training ###########

In this package two algorithms are implemented that use machine-learning techniques. These are the semantic segmentation and the voronoi random field segmentation.

1. The semantic segmentation uses two AdaBoost classifiers, one to detect hallways and one to detect rooms, based on simulated laserscannner-data [1]. For the training of this algorithm 


########### References ###########

[1] Martinez Mozos, O., Rottmann, A., Triebel, R., Jensfelt, P., Burgard, W., et al. Semantic labeling of places using information extracted from laser and vision sensor data (2006)
