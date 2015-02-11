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

#include "rose_conversions/conversions.hpp"
#include "server_multiple_client/server_multiple_client.hpp"

#include "server_multiple_client_msgs/smc_testAction.h"
#include "server_multiple_client_msgs/smc_testActionGoal.h"
#include "server_multiple_client_msgs/smc_testActionResult.h"
#include "server_multiple_client_msgs/smc_testActionFeedback.h"

#define NAME 	"TEST_SMC_C"
#define CLIENT1 "TEST_SMC_D"

typedef ServerMultipleClient<server_multiple_client_msgs::smc_testAction> SMC;

int cnt = 0;

void CB_goal(const server_multiple_client_msgs::smc_testGoalConstPtr& goal, SMC* smc)
{
	cnt++;
	ROS_INFO("CB_goal: %s, ---------------start! %d", NAME, cnt);
	server_multiple_client_msgs::smc_testResult result;
	result.result = std::rand() % 10;
	
	if(result.result == 0)
	{
		// Let it timeout
		ROS_INFO("CB_goal: %s, Making it timeout!", NAME);

		//! @todo OH [QUESTION]: What if it hangs here??!
		ros::Duration(2.0).sleep();

		smc->sendServerResult(true, result);
	}
	else if(result.result == 1)
	{
		ROS_INFO("CB_goal: %s, sending result 0 (FAILED)", NAME);
		smc->sendServerResult(false, result);
	}
	else
	{
		ROS_INFO("CB_goal: %s, sending result 0 (SUCCESS)", NAME);
		smc->sendServerResult(true, result);
	}
	
	ROS_INFO("CB_goal: %s, ---------------%d FINISHED! %d", NAME, result.result, cnt);
}

void CB_preempt(SMC* smc)
{
	ROS_INFO("CB_preempt: %s", NAME);
}

void CB_feedback(const server_multiple_client_msgs::smc_testFeedbackConstPtr& feedback)
{
	ROS_INFO("CB_feedback: %s", NAME);
}


int main(int argc, char *argv[])
{
	ros::init(argc, argv, NAME);
	ros::NodeHandle n;

	ROS_INFO("%s", ((std::string)NAME + " node started.").c_str());

	SMC smc(n, NAME, CB_goal, CB_preempt);

	smc.startServer();

	ros::Rate rate(5);
	while(n.ok() and not (rose_conversions::kbhit() and getchar() == 'x'))
	{
		ros::spinOnce();

		rate.sleep();
		ROS_INFO_THROTTLE(1.0, "%s", ((std::string)NAME + " spinning.").c_str());
	}

	ROS_INFO("%s", ((std::string)NAME + " node stopping.").c_str());
	
}
