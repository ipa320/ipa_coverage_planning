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
Important: The algorithms assume that the maps are given as 8-Bit image, with 0 as occupied space and 256 as free space.

1. The semantic segmentation uses two AdaBoost classifiers, one to detect hallways and one to detect rooms, based on simulated laserscannner-data [1]. 
For the training of this algorithm hand-labelled data showing where rooms and where hallways are. To do so see the given files in common/files/training_maps. You can see that for different maps some regions are gray and not white or black. These gray regions show where a hallway/room is, the color itself is not important. 
After you have created some training files yourself you have to specify where they lay on your file-system in the room_segmentation_action_server.launch file. When you set the train Boolean to true these given maps are read in and given to the training routine.

2. The voronoi random field segmentation also uses AdaBoost algorithms to check for rooms/hallways/doorways. Additionally the weak classifier results from these algorithms are used as Features in a conditional random field, resulting in two different algorithms that need to be trained [2].
Examples for training data for this algorithm can be seen in common/files/training_maps/voronoi_random_field_training. The Data consist of the original maps, with only white and black regions and the labeled maps (training_maps) with colors showing if the class of the current point. The colors have the following meaning:
	- 77: room
	- 115: hallway
	- 179: doorway
Optionally one can also provide additional data to save computation time, the precomputed maps with a voronoi graph drawn in (color 127) and maps that show where nodes are in the given voronoi graph. If you provide one of it you also need to provide the other.
After you created your personal training files you also have to specify the path to them in room_segmentation_action_server.launch.




########### References ###########

[1] Martinez Mozos, O., Rottmann, A., Triebel, R., Jensfelt, P., Burgard, W., et al. Semantic labeling of places using information extracted from laser and vision sensor data (2006)

[2] Friedman, Stephen, Hanna Pasula, and Dieter Fox. "Voronoi Random Fields: Extracting Topological Structure of Indoor Environments via Place Labeling." IJCAI. Vol. 7. 2007
