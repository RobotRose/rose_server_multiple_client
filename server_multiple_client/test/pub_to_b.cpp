/***********************************************************************************
* Copyright: Rose B.V. (2014)
*
* Revision History:
*	Author: Okke Hendriks
*	Date  : 2015/01/30
* 		- File created.
*
* Description:
*	Publishes cancel messages to smc_b
* 
***********************************************************************************/

#include <ros/ros.h> 
#include <std_msgs/Bool.h> 

#define NAME "CANCEL_B_PUBLISHER"

int main(int argc, char *argv[])
{
	ros::init(argc, argv, NAME);
	ros::NodeHandle n;

	ROS_INFO("%s", ((std::string)NAME + " node started.").c_str());
	
	ros::Publisher pub = n.advertise<std_msgs::Bool>("cancel_smc_b", 1);

	ros::Rate rate(20);
	while(n.ok())
	{
		ros::spinOnce();

		float sleep_time = (float)(std::rand() % 500)/1000.0;
		ROS_INFO("[%s] Sleep time is: %.4fs.", ((std::string)NAME).c_str(), sleep_time);
		ros::Duration(sleep_time).sleep();

		std_msgs::Bool bool_msg;
		bool_msg.data = (std::rand() % 2 == 0) ? true : false;
		pub.publish(bool_msg);

		ROS_INFO_THROTTLE(1.0, "%s", ((std::string)NAME + " spinning.").c_str());
	}

	ROS_INFO("%s", ((std::string)NAME + " node stopping.").c_str());
}
