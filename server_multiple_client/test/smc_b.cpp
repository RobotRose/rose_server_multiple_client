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

#include <ros/ros.h> 
#include <std_msgs/Bool.h> 
#include "rose20_common/server_multiple_client/server_multiple_client.hpp"

#include "rose20_common/smc_testAction.h"
#include "rose20_common/smc_testActionGoal.h"
#include "rose20_common/smc_testActionResult.h"
#include "rose20_common/smc_testActionFeedback.h"

#define NAME 	"TEST_SMC_B"
#define CLIENT1 "TEST_SMC_C"
#define CLIENT2 "TEST_SMC_D"

typedef ServerMultipleClient<rose20_common::smc_testAction> SMC;

SMC* smc_;

void CB_goal(const rose20_common::smc_testGoalConstPtr& goal, SMC* smc)
{
	ROS_INFO("CB_goal: %s, forwarding.", NAME);

	// Send a goal to SMC C
	if( not smc->sendGoal<rose20_common::smc_testAction>(*goal, CLIENT1, 2.0) )
	{
		ROS_WARN("%s", ((std::string)NAME + " unable to send goal to " + (std::string)CLIENT1 + ", aborting goal.").c_str());
		smc->abort();
		return;
	}
	else
		ROS_INFO("%s", ((std::string)NAME + " succesfully send goal to " + (std::string)CLIENT1).c_str());

	// Send a goal to SMC D
	if( not smc->sendGoal<rose20_common::smc_testAction>(*goal, CLIENT2, 2.0) )
	{
		ROS_WARN("%s", ((std::string)NAME + " unable to send goal to " + (std::string)CLIENT2 + ", aborting goal.").c_str());
		smc->abort();
		return;
	}
	else
		ROS_INFO("%s", ((std::string)NAME + " succesfully send goal to " + (std::string)CLIENT2).c_str());
	
	// Wait for result of SMC C, abort on timeout
	if( not smc->waitForSuccess(CLIENT1, ros::Duration(1.0)) )
	{
		smc_->abort();
		return;
	}

	// Wait for result of SMC D, abort on timeout
	if( not smc->waitForSuccess(CLIENT2, ros::Duration(1.0)) )
	{
		smc->abort();
		return;
	}
	else
	{
		// Success! Add results and send to client SMC A
		auto result_c = smc->getResult<rose20_common::smc_testAction>(CLIENT1);
		auto result_d = smc->getResult<rose20_common::smc_testAction>(CLIENT2);
		int temp = result_c.result;
		result_c.result += result_d.result;

		smc->sendServerResult(true, result_c);
		ROS_INFO("%s succesfully completed, result C: %d, result D: %d, result C+D: %d.", ((std::string)NAME).c_str(), temp, result_d.result, result_c.result);
	}
	
}

void CB_preempt(SMC* smc)
{
	ROS_INFO("CB_preempt: %s", NAME);
}

void CB_success(const actionlib::SimpleClientGoalState& state, const rose20_common::smc_testResultConstPtr& client_result)
{
	ROS_INFO("CB_success: %s, state: %s", CLIENT1,  state.toString().c_str());

}

void CB_fail(const actionlib::SimpleClientGoalState& state, const rose20_common::smc_testResultConstPtr& client_result)
{
	ROS_INFO("CB_fail: %s, state: %s", CLIENT1,  state.toString().c_str());
	
	smc_->abort();
}

void CB_active()
{
	ROS_INFO("CB_active: %s", CLIENT1);
}

void CB_feedback(const rose20_common::smc_testFeedbackConstPtr& feedback)
{
	ROS_INFO("CB_feedback: %s", CLIENT1);
}

void CB_cancel_smc_b(const std_msgs::BoolConstPtr& bool_msg)
{
	ROS_INFO("CB_cancel_smc_b: %s", CLIENT1);
	if( bool_msg->data )
	{
		ROS_INFO("CB_cancel_smc_b: canceling SMC");
		smc_->abort();
	}	
	else
		ROS_INFO("CB_cancel_smc_b: not canceling SMC");
}





int main(int argc, char *argv[])
{
	ros::init(argc, argv, NAME);
	ros::NodeHandle n;

	ROS_INFO("%s", ((std::string)NAME + " node started.").c_str());

	ros::Subscriber sub = n.subscribe("cancel_smc_b", 1, CB_cancel_smc_b);

	
	smc_ = new SMC(n, NAME, 
		boost::bind(&CB_goal, _1, _2), 
		boost::bind(&CB_preempt, _1));

	smc_->addClient<rose20_common::smc_testAction>(CLIENT1, 
	   boost::bind(&CB_success, _1, _2),
	   boost::bind(&CB_fail, _1, _2),
	   boost::bind(&CB_active),
	   boost::bind(&CB_feedback, _1));

	smc_->addClient<rose20_common::smc_testAction>(CLIENT2);

	smc_->startServer();

	rose20_common::smc_testGoal goal;

	ros::Rate rate(100);
	while(n.ok())
	{
		ros::spinOnce();
		rate.sleep();
		ROS_INFO_THROTTLE(1.0, "%s", ((std::string)NAME + " spinning.").c_str());
	}

	ROS_INFO("%s", ((std::string)NAME + " node stopping.").c_str());
}
