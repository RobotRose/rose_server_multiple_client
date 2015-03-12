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
#include <thread>
#include <mutex>
#include <atomic>
#include <condition_variable>
#include <boost/foreach.hpp>

#include "ros/ros.h"
#include "ros/callback_queue.h"
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/action_client.h>

#include "rose_common/common.hpp"
#include "ros_name/ros_name.hpp"

#define ROS_NAME_CC     (ROS_NAME + "|CC|" + client_name_)

#define SMC_DEFAULT_CANCEL_TIMEOUT  1.0  // [s]

class ComplexClientBase
{
public: 
    virtual ~ComplexClientBase() {}; // You need one ComplexClientBase virtual function in the ComplexClientBase class
    //! @todo OH [IMPR]: Do we need the other virtual functions? Do we need te add all the functions? Only non templated functions can be added /used here!
    virtual bool cancelGoal(const ros::Duration& timeout = ros::Duration(SMC_DEFAULT_CANCEL_TIMEOUT)) = 0;
    virtual bool hasGoalOutstanding() = 0;
    virtual bool waitForResult(const ros::Duration& timeout = ros::Duration(0.0)) = 0;
    virtual bool waitForSuccess(const ros::Duration& timeout = ros::Duration(0.0)) = 0;
    virtual bool waitForFailed(const ros::Duration& timeout = ros::Duration(0.0)) = 0;
    virtual bool getLastGoalSucces() = 0;
    virtual std::string getName() = 0;
};


template <class ClientActionType> class ComplexClient : public ComplexClientBase
{
  public:
    ACTION_DEFINITION(ClientActionType)
    typedef boost::function<void (const actionlib::SimpleClientGoalState& state,  const ResultConstPtr& result) > SimpleFailCallback;
    typedef boost::function<void (const actionlib::SimpleClientGoalState& state,  const ResultConstPtr& result) > SimpleDoneCallback;
    typedef boost::function<void () > SimpleActiveCallback;
    typedef boost::function<void (const FeedbackConstPtr& feedback) > SimpleFeedbackCallback;
    
    ComplexClient(const std::string& client_name,   SimpleDoneCallback      custom_done_cb      = NULL,
                                                    SimpleFailCallback      custom_fail_cb      = NULL,
                                                    SimpleActiveCallback    custom_active_cb    = NULL, 
                                                    SimpleFeedbackCallback  custom_feedback_cb  = NULL)
        : custom_succes_cb_(custom_done_cb)
        , custom_fail_cb_(custom_fail_cb)
        , custom_active_cb_(custom_active_cb)
        , custom_feedback_cb_(custom_feedback_cb)
        , client_name_(client_name)
        , goal_outstanding_(false)
        , last_goal_succes_(false)
        , last_result_(ResultConstPtr())
        , server_connected_(false)
        , stop_wait_for_server_thread_(false)
    {
        ROS_DEBUG_NAMED(client_name_, "Constructor client '%s'.", client_name_.c_str());
        simple_client_ = new actionlib::SimpleActionClient<ClientActionType>(client_name_, true);

        // Start waitForServer thread
        startWaitForServerThread();

        // Start looking for the server
        checkForServer();
    }

    ~ComplexClient()
    {
        ROS_DEBUG_NAMED(ROS_NAME_CC, "Destructor client '%s'.", client_name_.c_str());
        stopWaitForServerThread();

        if( hasGoalOutstanding() and isServerConnected() )
            cancelGoal();       

        // Delete the contained simpleActionClient
        delete simple_client_;
    }

    bool isServerConnected()
    {
        return server_connected_;
    }

    // Send a goal to the server.
    // canceling_timeout specifies the time to wait before canceling a goal which was send in the past.
    // A canceling_timeout of 0 specifies an infinite timeout.
    bool sendComplexGoal(const Goal& goal, float canceling_timeout)
    {
        ROS_DEBUG_NAMED(ROS_NAME_CC, "Sending goal to client '%s'.", client_name_.c_str());

         // Check if connected to server
        if( not isServerConnected() )
        {
            ROS_WARN_NAMED(ROS_NAME_CC, "Not connected to server, cannot send goal to client '%s'.", client_name_.c_str());            
            return false;
        }

        std::lock_guard<std::recursive_mutex> lock(cancel_mutex_);
        // Try to lock the mutex, if succesfull we are not persuing a goal currently thus we can send the new one.
        // If not succesfull we have to cancel our previous goal first.
        if( hasGoalOutstanding() )
        {
            ROS_DEBUG_NAMED(ROS_NAME_CC, "There is an outstanding goal of '%s', giving it %.4fs to finish.", client_name_.c_str(), canceling_timeout);
            
            if( not waitForResult(ros::Duration(canceling_timeout)) )
            {
                ROS_DEBUG_NAMED(ROS_NAME_CC, "The outstanding goal for '%s' did not finish in time, canceling.", client_name_.c_str());
                
                if( not cancelGoal() )
                {
                    ROS_WARN_NAMED(ROS_NAME_CC, "Canceling of outstanding goal failed, '%s', arborting sending of goal.", client_name_.c_str());
                    return false;
                }
            }
        }

        goal_outstanding_ = true;

        // Call standard SimpleActionClient 
        simple_client_->sendGoal(goal,  boost::bind(&ComplexClient::CB_clientDone, this, _1, _2),
                                        boost::bind(&ComplexClient::CB_clientActive, this),
                                        boost::bind(&ComplexClient::CB_clientFeedback, this, _1)
                                        );

        ROS_DEBUG_NAMED(ROS_NAME_CC, "Goal send to server of client '%s'.", client_name_.c_str());
        return true;
    }

    // Returns false if canceling timed out, returns true if canceled or if goal finished any other way
    bool cancelGoal(const ros::Duration& timeout = ros::Duration(SMC_DEFAULT_CANCEL_TIMEOUT))
    {
        ROS_DEBUG_NAMED(ROS_NAME_CC, "cancelGoal '%s'.", client_name_.c_str());
        if( not hasGoalOutstanding() )
        {
            ROS_DEBUG_NAMED(ROS_NAME_CC, "cancelGoal '%s', no goal outstanding.", client_name_.c_str());
            return true;
        }

        std::lock_guard<std::recursive_mutex> lock(cancel_mutex_);

        // Check if we are actually persuing a goal, otherwise we should not cancel
        const actionlib::SimpleClientGoalState& goal_state = simple_client_->getState();

        switch ( goal_state.state_ )
        {
            case actionlib::SimpleClientGoalState::PENDING:
            case actionlib::SimpleClientGoalState::ACTIVE:         
                ROS_DEBUG_NAMED(ROS_NAME_CC, "Goal state of client '%s' is '%s', sending cancel to server.", client_name_.c_str(), goal_state.toString().c_str());

                // Send cancel of the goal to the server
                simple_client_->cancelGoal();   
                break;

            case actionlib::SimpleClientGoalState::LOST:
                ROS_ERROR_NAMED(ROS_NAME_CC, "Goal state of client '%s' is '%s', forgetting this goal.", client_name_.c_str(), goal_state.toString().c_str()); 
                simple_client_->stopTrackingGoal();
                goal_outstanding_ = false;
                return true;
                break;

            case actionlib::SimpleClientGoalState::RECALLED:
            case actionlib::SimpleClientGoalState::REJECTED:   
            case actionlib::SimpleClientGoalState::PREEMPTED:   
            case actionlib::SimpleClientGoalState::ABORTED:   
            case actionlib::SimpleClientGoalState::SUCCEEDED:   
                ROS_DEBUG_NAMED(ROS_NAME_CC, "Goal state of client '%s' is '%s', not PENDING, ACTIVE or LOST, not sending cancel.", client_name_.c_str(), goal_state.toString().c_str()); 
                break;
            
            default:
                ROS_ERROR_NAMED(ROS_NAME_CC, "Unknown terminal state [%s].", goal_state.toString().c_str());
                simple_client_->stopTrackingGoal();
                goal_outstanding_ = false;

                return false;
                break;
        }

        return waitForResult(timeout);
    }

    void CB_clientDone(const actionlib::SimpleClientGoalState& state,  const ResultConstPtr& result)
    {
        ROS_DEBUG_NAMED(ROS_NAME_CC, "CB_clientDone() client '%s', goal state: %s", client_name_.c_str(), state.toString().c_str());
        goal_outstanding_ = false;

        // Call custom callbacks if goal state was set to succeeded or aborted by the server
        if(state == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            setLastResult(result);
            last_goal_succes_ = true;

            if(custom_succes_cb_ != NULL)
                custom_succes_cb_(state, result);
        }            
        else if (state == actionlib::SimpleClientGoalState::ABORTED)
        {
            setLastResult(result);
            last_goal_succes_ = false; 
            
            if(custom_fail_cb_ != NULL)
                custom_fail_cb_(state, result); 
        }

        ROS_DEBUG_NAMED(ROS_NAME_CC, "CB_clientDone() client '%s' finished", client_name_.c_str());
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

    std::string getName()
    {
        return client_name_;
    }

    bool hasGoalOutstanding()
    {
        return goal_outstanding_;
    }

    ResultConstPtr getLastResult()
    {
        std::lock_guard<std::mutex> last_result_lock(last_result_mutex_);
        
        if(last_result_ == NULL)
            return ResultConstPtr(new Result);

        return last_result_;
    }

    bool getLastGoalSucces()
    {
        return last_goal_succes_;
    }

  private:

    void setLastResult(ResultConstPtr result)
    {
        std::lock_guard<std::mutex> last_result_lock(last_result_mutex_);
        last_result_ = result;
    }

    // SMC internal wait for result function
    bool waitForResult(const ros::Duration& timeout = ros::Duration(0.0))
    {
        if( not hasGoalOutstanding() )
        {
            ROS_INFO_NAMED(ROS_NAME_CC, "Client '%s' has no outstanding goal, not wait for result required.", client_name_.c_str());
            return true;
        }

        ROS_DEBUG_NAMED(ROS_NAME_CC, "Waiting on result of client '%s' after specified timeout of %.4fs.", client_name_.c_str(), timeout.toSec());

        // Tell the user it is not smart to block forever
        if(timeout.toSec() == 0.0)
            ROS_WARN_ONCE_NAMED(ROS_NAME_CC, "A timeout of 0 implies an infinite timeout, this is probably not smart to use! This warning will only be displayed once.");

        // Did we get a result or a timeout
        if( simple_client_->waitForResult(ros::Duration(timeout)) )
        {
            ROS_DEBUG_NAMED(ROS_NAME_CC, "Got result of client '%s', goal state: %s", client_name_.c_str(), simple_client_->getState().toString().c_str());
            return true;
        }
        // Recheck if we have an goal outstanding because it could be canceled in the meantime.
        else if( hasGoalOutstanding() )
        {
            ROS_WARN_THROTTLE_NAMED(0.5, ROS_NAME_CC, "Waiting on result of client '%s' timed out after %.4fs!", client_name_.c_str(), timeout.toSec()); 

            //! @todo OH [CHECK]: Should the goal at this point indeed be forgotten?.
            simple_client_->stopTrackingGoal();
            goal_outstanding_ = false;

            // Probably disconnected, start wait for connection loop again, this also sets the connected state to false.
            checkForServer();
        }

        return false;
    }

    bool waitForSuccess(const ros::Duration& timeout = ros::Duration(0.0))
    {
        return waitForResult(timeout) && last_goal_succes_;
    }

    bool waitForFailed(const ros::Duration& timeout = ros::Duration(0.0))
    {
        return not waitForSuccess(timeout);
    }
    

    // Check if server is connected, sets server_connected_ to false
    void checkForServer()
    {
        ROS_DEBUG_NAMED(ROS_NAME_CC, "Client '%s' will check connection to server.", client_name_.c_str()); 
        cv_.notify_all();
    }

    void startWaitForServerThread()
    {
        ROS_DEBUG_NAMED(ROS_NAME_CC, "Starting wait for server thread.");

        if(wait_for_server_thread_.joinable())
        {
            ROS_ERROR_NAMED(ROS_NAME_CC, "Cannot start wait for server thread, stop and join first.");
            return;
        }

        wait_for_server_thread_ = std::thread(&ComplexClient<ClientActionType>::waitForServerThread, this);
    }

    void stopWaitForServerThread()
    {
        ROS_INFO_NAMED(ROS_NAME_CC, "Client '%s' will stop trying to connect to server.", client_name_.c_str()); 
        stop_wait_for_server_thread_ = true;
        cv_.notify_all();
        wait_for_server_thread_.join();
    }

    void waitForServerThread()
    {
        ros::NodeHandle n;
        while( ( not stop_wait_for_server_thread_ ) and n.ok() )
        {
            bool prev_server_connected_ = server_connected_;

            if( simple_client_->waitForServer(ros::Duration(1.0)) )
                server_connected_ = true;
            else
            {
                if(server_connected_)
                    ROS_WARN_NAMED(ROS_NAME_CC, "Server not connected, will wait for server of client '%s'.", client_name_.c_str()); 

                server_connected_ = false;
                ros::Duration(1.0).sleep();
                continue;
            }
        
            // Only show once when state actually did change from false to true
            if(server_connected_ != prev_server_connected_)
                ROS_INFO_NAMED(ROS_NAME_CC, "Server of client '%s' connected.", client_name_.c_str());      
            else
                ROS_DEBUG_NAMED(ROS_NAME_CC, "Server of client '%s' (still) connected.", client_name_.c_str());      

            // Wait for reactivation
            std::unique_lock<std::mutex> lock(cv_mutex_);
            cv_.wait(lock);

        }
    }

  private: 
    std::string                                         client_name_;
    actionlib::SimpleActionClient<ClientActionType>*    simple_client_;

    SimpleDoneCallback                      custom_succes_cb_;  
    SimpleFailCallback                      custom_fail_cb_;
    SimpleActiveCallback                    custom_active_cb_;      
    SimpleFeedbackCallback                  custom_feedback_cb_;

    std::atomic_bool                goal_outstanding_;
    std::atomic_bool                last_goal_succes_;
    std::atomic_bool                server_connected_;

    ResultConstPtr                  last_result_;
    std::mutex                      last_result_mutex_; 

    std::recursive_mutex            cancel_mutex_;
    
    std::thread                     wait_for_server_thread_;
    std::atomic_bool                stop_wait_for_server_thread_;
    std::condition_variable         cv_;
    std::mutex                      cv_mutex_; 
};

#endif // COMPLEX_CLIENT_HPP
