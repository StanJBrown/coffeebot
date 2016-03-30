#include "ros/ros.h"
#include "coffeebot/room_to_goal.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "room_to_goal_client");
  if(argc != 2)
  {
    ROS_INFO("usage: room_to_goal_client room");
    return 1;
  }

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<coffeebot::room_to_goal>("room_to_goal");
  coffeebot::room_to_goal srv;
  srv.request.room = atoll(argv[1]);
  if(client.call(srv))
  {
    //ROS_INFO("Location (x,y,z) = (%f, %f, %f)", res.position.x, res.position.y, res.position.z);
    ROS_INFO("Location received");
  }
  else
  {
    ROS_ERROR("Failed to call service room_to_goal");
    return 1;
  }
  return 0;
}
