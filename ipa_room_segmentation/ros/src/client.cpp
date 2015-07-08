#include "ros/ros.h"
#include "ipa_room_segmentation/seg.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "client");

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<ipa_room_segmentation::seg>("roomsegmentation");
  ipa_room_segmentation::seg srv;
  srv.request.a = 1;
  client.call(srv);

  return 0;
}
