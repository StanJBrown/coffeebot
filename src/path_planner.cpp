#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>


int main(int argc, char** argv)
{
	ros::init(argc, argv, "path_planner");
	ros::NodeHandle n;

	tf::TransformBroadcaster broadcaster;

	ros::Rate loop_rate(10);

	while (ros::ok())
	{
		broadcaster.sendTransform(
			tf::StampedTransform(
				tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(-0.5, 0.0, 0.5)),
				ros::Time::now(), "base_link", "scan"));




		// Chill until the next loop
		loop_rate.sleep();
	}

	return 0;
}
