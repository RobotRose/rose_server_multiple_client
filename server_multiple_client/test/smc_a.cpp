/***********************************************************************************
* Copyright: Rose B.V. (2014)
*
* Revision History:
*	Author: Okke Hendriks
*	Date  : 2015/01/19
* 		- File created.
*
* Description:
*	Test for node for the server multiple client
* 
***********************************************************************************/

#include <ros/ros.h> 
#include "server_multiple_client/server_multiple_client.hpp"

#include "server_multiple_client_msgs/smc_testAction.h"
#include "server_multiple_client_msgs/smc_testActionGoal.h"
#include "server_multiple_client_msgs/smc_testActionResult.h"
#include "server_multiple_client_msgs/smc_testActionFeedback.h"

#define NAME 	"TEST_SMC_A"
#define CLIENT1 "TEST_SMC_B"

typedef ServerMultipleClient<server_multiple_client_msgs::smc_testAction> SMC;

int main(int argc, char *argv[])
{
	ros::init(argc, argv, NAME);
	ros::NodeHandle n;

	ROS_INFO("%s", ((std::string)NAME + " node started.").c_str());

	SMC smc(n, NAME);

	smc.addClient<server_multiple_client_msgs::smc_testAction>(CLIENT1);

	smc.startServer();

	server_multiple_client_msgs::smc_testGoal goal;

	while(n.ok())
	{
		ros::spinOnce();

		if(smc.sendGoal<server_multiple_client_msgs::smc_testAction>(goal, CLIENT1, 3.0))
			ROS_INFO("%s", ((std::string)NAME + " succesfully send goal to " + (std::string)CLIENT1).c_str());
		else
			ROS_INFO("%s", ((std::string)NAME + " unable to send goal to " + (std::string)CLIENT1).c_str());
		
		float execute_time = 0.0001 + (float)(std::rand() % 2000)/1000.0;
		ROS_INFO("[%s] Premitted execute time is: %.4fs.", ((std::string)NAME).c_str(), execute_time);

		if(smc.waitForResult(CLIENT1, ros::Duration(execute_time)))
			ROS_INFO("RESULT: %d", smc.getResult<server_multiple_client_msgs::smc_testAction>(CLIENT1)->result);
		else
			ROS_WARN("NO RESULT, execute time has been exceeded.");
	}

	ROS_INFO("%s", ((std::string)NAME + " node stopping.").c_str());

}
