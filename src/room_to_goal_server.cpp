#include <ros/ros.h>
#include "coffeebot/room_to_goal.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/Header.h"
#include <iostream>
#include <map>

/*
bool room2goal(coffeebot::room_to_goal::Request &req,
             coffeebot::room_to_goal::Response &res)
{
  res.goal.pose.position.x = 1.0;
  res.goal.pose.position.y = 2.0;
  res.goal.pose.position.z = 3.0;
  res.goal.pose.orientation.x = 0.0;
  res.goal.pose.orientation.y = 0.0;
  res.goal.pose.orientation.z = 0.0;
  res.goal.pose.orientation.w = 1.0;

  ROS_INFO("request room: %d ", req.room);
  //ROS_INFO("Location x=%d, y=%d, z=%d", (int)res.position.x, (int)res.position.y, (int)res.position.z);
  ros::NodeHandle nh;
  ros::Publisher goal_pub = nh.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal",10);
  geometry_msgs::PoseStamped msg;
  msg.pose.position.x = 5.0;
  msg.pose.position.y = 5.0;
  msg.pose.position.z = 0.0;
  msg.pose.orientation.x = 0.0;
  msg.pose.orientation.y = 0.0;
  msg.pose.orientation.z = 0.0;
  msg.pose.orientation.w = 1.0;
  msg.header.frame_id = "odom";
  ros::Time current_time;
  current_time = ros::Time::now();
  msg.header.stamp = current_time;
  goal_pub.publish(msg);
  ROS_INFO("Published goal");
  return true;
}
*/


using namespace std;


struct Goal
{
    double xPosition;
    double yPosition;
};

map<int, Goal> goals;

void setupGoals()
{
    //Goal goal1 = {0.0, 1.0}
    goals[3001] = {31.5, 19.2};
    goals[3002] = {-15.5, 9.2};

    goals[5001] = {-6.4, 8.4};
    goals[5002] = {-3.6, -6.5};
}

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "room_to_goal_server");
    ros::NodeHandle n;

    ros::Publisher goal_pub = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal",100);

    // Location declaration
    setupGoals();


    //map<double,double> mapx;
    //map<double,double> mapy;
    int query;
    cout << "Enter room\n";
    cin >> query;
    //mapx[1] = 0.0;
    //mapy[1] = 5.0;
    //mapx[2] = 2.0;
    //mapy[2] = -5.0;

    // Find the coordinates of the room


    //ros::Rate max_loop_rate(100);
    //max_loop_rate.sleep();
    //double room_x, room_y;
    /*
    if(mapx.find(query) != mapx.end()){
    room_x = mapx[query];
    };
    if(mapy.find(query) != mapy.end()){
    room_y = mapy[query];
    };
    */
    //ros::Duration(1).sleep();
    //ros::Rate loop_rate(10);
    /*
    if(query==3012){
    room_x = 19.815;
    room_y = -3.938;


}*/

    int i=1;
    while(i<25){
        geometry_msgs::PoseStamped msg;
        ros::Time current_time;
        current_time = ros::Time::now();
        msg.header.stamp = current_time;
        msg.header.frame_id = "map";
        msg.pose.position.x = goals[query].xPosition;
        msg.pose.position.y = goals[query].yPosition;
        msg.pose.position.z = 0.0;
        msg.pose.orientation.x = 0.0;
        msg.pose.orientation.y = 0.0;
        msg.pose.orientation.z = 0.43;
        msg.pose.orientation.w = 0.903;
        goal_pub.publish(msg);
        ROS_INFO("Published goal %d at %f, %f", query, goals[query].xPosition, goals[query].yPosition);
        //loop_rate.sleep();
        i=i+1;
    }

    /*
    ros::ServiceServer service = n.advertiseService("room_to_goal", room2goal);
    ROS_INFO("Enter the room number");
    ROS_INFO("1");
    ros::spin();
    */
    return 0;
}
