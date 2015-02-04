/***********************************************************************************
* Copyright: Rose B.V. (2013)
*
* Revision History:
*  Author: Okke Hendriks
*  Date  : 2014/01/30
*     - File created.
*
* Description:
*  ComplexActionClient class and base class, header only due to use of templates
* 
***********************************************************************************/

#ifndef COMPLEX_CLIENT_HPP
#define COMPLEX_CLIENT_HPP

#include <map>
#include <vector>
#include <typeinfo>
#include <boost/foreach.hpp>

#include "ros/ros.h"
#include "ros/callback_queue.h"
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/action_client.h>

#include "rose_common/common.hpp"
#include "rose20_common/ros_name.hpp"

#define ROS_NAME_CC     (ROS_NAME + "|CC|" + client_name_)

#define SMC_DEFAULT_CANCEL_TIMEOUT  1.0  // [s]

using namespace std;
using namespace actionlib;

class ComplexClientBase
{
public: 
    virtual ~ComplexClientBase() {}; // You need one ComplexClientBase virtual function in the ComplexClientBase class
    virtual bool cancelGoal(const ros::Duration& timeout = ros::Duration(SMC_DEFAULT_CANCEL_TIMEOUT)) = 0;
    virtual bool isBusy() = 0;
    virtual bool waitForResult(const ros::Duration& timeout = ros::Duration(0.0)) = 0;
    virtual string getName() = 0;
};


template <class ClientActionType> class ComplexClient : public ComplexClientBase
{
  public:
    ACTION_DEFINITION(ClientActionType)
    typedef boost::function<void (const SimpleClientGoalState& state,  const ResultConstPtr& result) > SimpleFailCallback;
    typedef boost::function<void (const SimpleClientGoalState& state,  const ResultConstPtr& result) > SimpleDoneCallback;
    typedef boost::function<void () > SimpleActiveCallback;
    typedef boost::function<void (const FeedbackConstPtr& feedback) > SimpleFeedbackCallback;
    
    ComplexClient(string client_name,   SimpleDoneCallback      custom_done_cb      = NULL,
                                        SimpleFailCallback      custom_fail_cb      = NULL,
                                        SimpleActiveCallback    custom_active_cb    = NULL, 
                                        SimpleFeedbackCallback  custom_feedback_cb  = NULL)
        : custom_succes_cb_(custom_done_cb)
        , custom_fail_cb_(custom_fail_cb)
        , custom_active_cb_(custom_active_cb)
        , custom_feedback_cb_(custom_feedback_cb)
        , goal_outstanding_(false)
        , client_name_(client_name)
        , last_result_(new Result)
        , last_goal_succes_(false)
    {
        simple_client_ = new SimpleActionClient<ClientActionType>(client_name_, true);
    }

    ~ComplexClient()
    {
        cancelGoal();       

        // Wait for result?
        delete simple_client_;
    }


    bool sendComplexGoal(const Goal& goal)
    {
        if(!waitForServer(ros::Duration(10.0)))
        {
            ROS_ERROR_NAMED(ROS_NAME_CC, "Client '%s' could not send goal. Is it up and running?", client_name_.c_str());       
            return false;
        }

        // Call standard SimpleActionClient 
        simple_client_->sendGoal(goal,  boost::bind(&ComplexClient::CB_clientDone, this, _1, _2),
                                        boost::bind(&ComplexClient::CB_clientActive, this),
                                        boost::bind(&ComplexClient::CB_clientFeedback, this, _1)
                                        );
        // Store that we did send a goal
        goal_outstanding_ = true;

        return true;
    }

    // Returns false if nothing had to be canceled, returns true if cancled or if canceling timedout
    bool cancelGoal(const ros::Duration& timeout = ros::Duration(SMC_DEFAULT_CANCEL_TIMEOUT))
    {
        // Check if we have a goal at the moment, otherwise we cannot get its state and dont have to cancel
        if(!goal_outstanding_)
            return false;

        // Get the state of the goal
        const SimpleClientGoalState& state = simple_client_->getState();        

        // Check if we are actually persuing a goal, otherwise we should not cancel         
        if( state == actionlib::SimpleClientGoalState::PENDING ||
            state == actionlib::SimpleClientGoalState::ACTIVE)
        {
            ROS_DEBUG_NAMED(ROS_NAME_CC, "Canceling goal of client '%s' with state: %s\n", client_name_.c_str(), state.toString().c_str());

            // Cancel the goal
            simple_client_->cancelGoal();

            // Wait for canceled
            if(simple_client_->waitForResult(ros::Duration(timeout)))    //! @todo OH: duration fixed??
                ROS_DEBUG_NAMED(ROS_NAME_CC, "Goal of client '%s' canceled", client_name_.c_str());
            else
                ROS_WARN_NAMED(ROS_NAME_CC, "Canceling goal of client '%s' timed out!", client_name_.c_str()); 
        }   

        last_goal_succes_ = false;

        return true;    
    }

    void CB_clientDone(const SimpleClientGoalState& state,  const ResultConstPtr& result)
    {
        ROS_DEBUG_NAMED(ROS_NAME_CC, "CB_clientDone() state: %s", state.toString().c_str());

        last_result_ = result;

        // Store that we are finished with a goal, we have none outstanding
        goal_outstanding_ = false;  

        // Call custom callbacks
        if(state == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            last_goal_succes_ = true;

            if(custom_succes_cb_ != NULL)
                custom_succes_cb_(state, result);
        }            
        else if (state == actionlib::SimpleClientGoalState::ABORTED)
        {
            last_goal_succes_ = false;
            
            if(custom_fail_cb_ != NULL)
                custom_fail_cb_(state, result); 
        }
    }

    void CB_clientActive()
    {
        ROS_DEBUG_NAMED(ROS_NAME_CC, "CB_clientActive()");
        if(custom_active_cb_ != NULL)
            custom_active_cb_();
    }

    void CB_clientFeedback(const FeedbackConstPtr& feedback)
    {
        ROS_DEBUG_NAMED(ROS_NAME_CC, "CB_clientFeedback()");
        if(custom_feedback_cb_ != NULL)
            custom_feedback_cb_(feedback);
    }

    string getName()
    {
        return client_name_;
    }

    bool isBusy()
    {
        return goal_outstanding_;
    }

    bool waitForResult(const ros::Duration& timeout = ros::Duration(0.0))
    {
        if(timeout.toSec() == 0.0)
            ROS_WARN_NAMED(ROS_NAME_CC, "A timeout of 0 implies an infinite timeout, this is probably not smart to use.");

        ROS_INFO_NAMED(ROS_NAME_CC, "Waiting on result of client '%s' with timeout %.3f", client_name_.c_str(), timeout.toSec());;

        // If waiting times out, cancel the goal
        if(!simple_client_->waitForResult(timeout))
        {
            ROS_WARN_NAMED(ROS_NAME_CC, "waitForResult(%.3fs) of client '%s' timed out!", timeout.toSec(), client_name_.c_str());  
            cancelGoal();       // Sets last_goal_succes_ to false
        }

        return last_goal_succes_;
    }

    bool waitForServer(const ros::Duration& timeout = ros::Duration(0.0))
    {
        if(timeout.toSec() == 0.0)
            ROS_WARN_NAMED(ROS_NAME_CC, "A timeout of 0 implies an infinite timeout, this is probably not smart to use.");

        // If waiting times out, return false
        int number = 10;
        int i = 0;
        for(i = 0; i < number; i++)
        {
            if(simple_client_->waitForServer(ros::Duration(timeout.toSec()/(float)number)))
                return true;
            else
                ROS_WARN_THROTTLE_NAMED(0.9, ROS_NAME_CC, "Waiting for server '%s'", client_name_.c_str()); 
        }

        return false;
    }

    const ResultConstPtr& getLastResult()
    {
        return last_result_;
    }

    const bool& getLastGoalSucces()
    {
        return last_goal_succes_;
    }

  private: 
    string                  client_name_;
    bool                    goal_outstanding_;
    SimpleActionClient<ClientActionType>* simple_client_;
    ResultConstPtr          last_result_;
    bool                    last_goal_succes_;

    SimpleDoneCallback      custom_succes_cb_;  
    SimpleFailCallback      custom_fail_cb_;
    SimpleActiveCallback    custom_active_cb_;      
    SimpleFeedbackCallback  custom_feedback_cb_;

};

#endif // COMPLEX_CLIENT_HPP
