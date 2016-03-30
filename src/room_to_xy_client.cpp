#include "ros/ros.h"
#include "coffeebot/room_to_xy.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "room_to_xy_client");
  if(argc != 2)
  {
    ROS_INFO("usage: room_to_xy_client room");
    return 1;
  }

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<coffeebot::room_to_xy>("room_to_xy");
  coffeebot::room_to_xy srv;
  srv.request.room = atoll(argv[1]);
  if(client.call(srv))
  {
    //ROS_INFO("Location (x,y,z) = (%f, %f, %f)", res.position.x, res.position.y, res.position.z);
    ROS_INFO("Location received");
  }
  else
  {
    ROS_ERROR("Failed to call service room_to_xy");
    return 1;
  }
  return 0;
}
