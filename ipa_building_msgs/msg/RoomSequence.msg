uint32[] room_indices		# indices of those rooms that belong to one group (connected to a checkpoint,
							# i.e. the clique of rooms close [in terms of robot driving distance] to each other),
							# indices should be ordered in optimized traveling salesman sequence
geometry_msgs/Point32 checkpoint_position_in_pixel			# provides the (x,y)-coordinates of the checkpoint location for this group of rooms [in pixel]
geometry_msgs/Point32 checkpoint_position_in_meter			# provides the (x,y)-coordinates of the checkpoint location for this group of rooms [in meter]