#include "ros/ros.h"
#include "std_msgs/String.h"
#include "stdlib.h"

#include "kobuki_msgs/BumperEvent.h"
#include "tf/tf.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/OccupancyGrid.h"

#include <sstream>

#include "rosbert_lab4/a_star.hpp"

int main(int argc, char **argv) {
	ros::init(argc, argv, "talker");

	ros::NodeHandle n;

	ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

	ros::Rate loop_rate(10);

	int count = 0;
	while (ros::ok()) {

		std_msgs::String msg;

		std::stringstream ss;
		ss << "hello world " << count;
		msg.data = ss.str();

		ROS_INFO("%s", msg.data.c_str());

		chatter_pub.publish(msg);

		ros::spinOnce();

		loop_rate.sleep();
		++count;
	}

	return 0;
}
