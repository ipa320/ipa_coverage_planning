#include <ipa_room_exploration/coverage_check_server.h>


int main(int argc, char **argv)
{
	ros::init(argc, argv, "coverage_check_server");
	ros::NodeHandle nh("~");

	CoverageCheckServer coverage_checker(nh);

	ros::spin();
	return 0;
}
