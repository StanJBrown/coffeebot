#include "ros/ros.h"
#include "coffeebot/room_to_xy.h"

bool room2xy(coffeebot::room_to_xy::Request &req,
             coffeebot::room_to_xy::Response &res)
{
  res.position.x = 1.0;
  res.position.y = 2.0;
  res.position.z = 3.0;
  ROS_INFO("request room: %d ", req.room);
  ROS_INFO("Location x=%d, y=%d, z=%d", (int)res.position.x, (int)res.position.y, (int)res.position.z);
  return true;
}

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "room_to_xy_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("room_to_xy", room2xy);
  ROS_INFO("Enter the room number");
  ros::spin();

  return 0;
}
