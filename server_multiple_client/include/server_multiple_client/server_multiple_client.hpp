/***********************************************************************************
* Copyright: Rose B.V. (2013)
*
* Revision History:
*  Author: Okke Hendriks
*  Date  : 2014/01/30
*     - File created.
*
* Description:
*  ServerMultipleClient class, header only due to use of templates
* 
***********************************************************************************/

//! @todo OH: Move SMC into seperate library

#ifndef SERVER_MULTIPLE_CLIENT_HPP
#define SERVER_MULTIPLE_CLIENT_HPP

#include <map>
#include <vector>
#include <typeinfo>

#include <boost/foreach.hpp>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/action_client.h>

#include "rose_common/common.hpp"
#include "ros_name/ros_name.hpp"

#include "server_multiple_client/complex_client.hpp"

#include "server_multiple_client_msgs/smc_dummy_serverAction.h"
#include "server_multiple_client_msgs/smc_dummy_serverActionGoal.h"
#include "server_multiple_client_msgs/smc_dummy_serverActionResult.h"
#include "server_multiple_client_msgs/smc_dummy_serverActionFeedback.h"

#include "thread_safe_stl_containers/thread_safe_map.h"

#define ROS_NAME_SMC    (ROS_NAME + "|SMC")

using namespace std;
using namespace actionlib;

template <class ServerActionType = server_multiple_client_msgs::smc_dummy_serverAction> class ServerMultipleClient
{
  public:
    ACTION_DEFINITION(ServerActionType)
    typedef boost::function<void (const GoalConstPtr& goal, ServerMultipleClient<ServerActionType>* smc) > ServerWorkCallback;
    typedef boost::function<void (ServerMultipleClient<ServerActionType>* smc) > ServerPreemptCallback;

    ServerMultipleClient(ros::NodeHandle n, const std::string& server_name, ServerWorkCallback      server_work_cb = NULL, 
                                                                            ServerPreemptCallback   server_preempt_cb = NULL)
        : server_name_(server_name)
        , n_(n)
        , server_work_cb_(server_work_cb)
        , server_preempt_cb_(server_preempt_cb)
        , server_started_(false)
    {
        // Create server
        server_ = new SimpleActionServer<ServerActionType>(n_, server_name_, boost::bind(&ServerMultipleClient<ServerActionType>::CB_serverGoalReceived, this, _1) , false);    
        
        // Register preempt received callback
        server_->registerPreemptCallback(boost::bind(&ServerMultipleClient<ServerActionType>::CB_serverPreempt, this));  

        ROS_INFO_NAMED(ROS_NAME_SMC, "Created server: %s", server_name_.c_str());
    }

    ~ServerMultipleClient()
    {
        ROS_INFO("~ServerMultipleClient()");

        cancelAllClients();
        server_started_ = false;
        
        pair<string, ComplexClientBase*> a_pair; 
        BOOST_FOREACH(a_pair, clients_) 
        {
            delete a_pair.second;
            ROS_INFO_NAMED(ROS_NAME_SMC, "Removed client: %s", a_pair.first.c_str());
        }

        delete server_;

    }

    void startServer()
    {
        // Check if a server work callback has been registered.
        if(server_work_cb_ == NULL)
        {
            ROS_ERROR_NAMED(ROS_NAME_SMC, "Unable to start server '%s' because no server work callback has been registered.", server_name_.c_str());
            return;
        }            

        // Start the server
        server_->start();
        server_started_ = true; 

        ROS_INFO_NAMED(ROS_NAME_SMC, "Started server: %s", server_name_.c_str());
    }

    void stopServer()
    {
        // Stop the SimpleServer
        server_->stop();
        server_started_ = false; 

        ROS_INFO_NAMED(ROS_NAME_SMC, "Stopped server: %s", server_name_.c_str());
    }

    SimpleActionServer<ServerActionType>* getSimpleServer()
    {
        return server_;
    }

    template <class ClientActionType>
    bool addClient(const std::string& client_name,  typename ComplexClient<ClientActionType>::SimpleDoneCallback        CB_custom_done      = NULL,
                                                    typename ComplexClient<ClientActionType>::SimpleFailCallback        CB_custom_fail      = NULL,
                                                    typename ComplexClient<ClientActionType>::SimpleActiveCallback      CB_custom_active    = NULL,                                 
                                                    typename ComplexClient<ClientActionType>::SimpleFeedbackCallback    CB_custom_feedback  = NULL) 

    {
        if(server_started_)
        {
            ROS_ERROR_NAMED(ROS_NAME_SMC, "Error while trying to add client '%s' to server '%s', because the server is already started.", client_name.c_str(), server_name_.c_str());
            return false;
        }

        if(clients_.count(client_name) > 0)
        {
            ROS_WARN_NAMED(ROS_NAME_SMC, "Client '%s' was not added to server '%s', because it already exists.", client_name.c_str(), server_name_.c_str());
            return false;
        }

        ComplexClient<ClientActionType>* complex_client = new ComplexClient<ClientActionType>(  client_name, 
                                                                                                CB_custom_done,
                                                                                                CB_custom_fail,
                                                                                                CB_custom_active,
                                                                                                CB_custom_feedback);
        
        clients_.insert( pair<string, ComplexClientBase*>(client_name, complex_client));

        ROS_INFO_NAMED(ROS_NAME_SMC, " Added client: %s", client_name.c_str());
    }

    //! @todo OH [IMPR]: Add send and wait functionality.
    //! @todo OH [IMPR]: Add send and fail send fail result on not completing.

    template <class ClientActionType>
    bool sendGoal(const typename ComplexClient<ClientActionType>::Goal& goal, const float& send_cancel_wait_time = 0.0)
    {
        if(clients_.empty())
        {
            ROS_ERROR_NAMED(ROS_NAME_SMC, "sendGoal: Cannot sendGoal because there are no clients.");
            return false;
        }
            
        if(clients_.size() > 1)
        {
            ROS_ERROR_NAMED(ROS_NAME_SMC, "sendGoal: Specify a client name when a SMC has more than one client.");
            return false;
        }
        else
        {
            // Select the only client 
            return sendGoal<ClientActionType>(goal, clients_.begin()->first, send_cancel_wait_time);
        }
    }

    // send_cancel_wait_time specifies the time to wait, before canceling a goal which was send in the past.
    // A send_cancel_wait_time of 0 specifies an infinite timeout.
    template <class ClientActionType>
    bool sendGoal(const typename ComplexClient<ClientActionType>::Goal& goal, const std::string& client_name, const float& send_cancel_wait_time = 0.0)
    {
        ROS_DEBUG_NAMED(ROS_NAME_SMC, "sendGoal -> %s", client_name.c_str());

        if(clients_.count(client_name) == 0)
        {
            ROS_ERROR_NAMED(ROS_NAME_SMC, "Error while trying to send goal via client '%s', 'client does not exist.", client_name.c_str());
            return false;
        }

        if(!castComplexClientBaseToComplexClientPtr<ClientActionType>(clients_[client_name])->sendComplexGoal(goal, send_cancel_wait_time))
            return false;

        latest_client_ = client_name;

        return true;
    }

    template <class ClientActionType>
    bool sendGoals(const typename ComplexClient<ClientActionType>::Goal& goal, const std::vector<string>& client_names, const float& send_cancel_wait_time = 0.0)
    {
        if( hasBusyClients() )
        {
            ROS_ERROR_NAMED(ROS_NAME_SMC, "Error while trying to send goal, 'sendGoals()' already has outstanding goal(s).");
            return false;
        }

        for(const auto& client_name : client_names)
        {
            if( not sendGoal<ClientActionType>(goal, client_name, send_cancel_wait_time) )
            {
                cancelAllClients();
                return false;
            }
        }

        return true;
    }

    template <class ClientActionType>
    const typename ComplexClient<ClientActionType>::Result getResult(const std::string& client_name)
    {
        return castComplexClientBaseToComplexClientPtr<ClientActionType>(getClient(client_name))->getLastResult();
    }

    template <class ClientActionType>
    typename ComplexClient<ClientActionType>::ResultConstPtr getResultLastestClient()
    {
        ROS_DEBUG_NAMED(ROS_NAME_SMC, "getResultLastestClient()");
        return castComplexClientBaseToComplexClientPtr<ClientActionType>(clients_[latest_client_])->getLastResult();
    }

    bool waitForResult(const ros::Duration& timeout = ros::Duration(0.0))
    {
        if(clients_.empty())
        {
            ROS_ERROR_NAMED(ROS_NAME_SMC, "waitForResult: Cannot wait for result because there are no clients.");
            return false;
        }

        if(clients_.size() > 1)
        {
            ROS_ERROR_NAMED(ROS_NAME_SMC, "waitForResult: Specify a client name when a SMC has more than one client.");
            return false;
        }
        else
        {
            // Select the only client 
            return waitForResult(clients_.begin()->first, timeout);
        }
    }

    bool waitForResult(const std::string& client_name, const ros::Duration& timeout = ros::Duration(0.0))
    {
        return getClient(client_name)->waitForResult(timeout);
    }

    //! @todo OH [IMPR]: Difficult because of race conditions? how do we do this?.

    bool waitForAllResults(const ros::Duration& timeout = ros::Duration(0.0))
    {
        ROS_DEBUG_NAMED(ROS_NAME_SMC, "waitForAllResults()");
      
        bool all_success = true;
        
        // Wait for all, busy, clients. NOTE this is not in parallel!                           //! @todo OH [IMPR]: Do this in parallel.
        for(auto& client : clients_) 
            all_success = all_success && waitForResult(client.second->getName(), timeout);      //! @todo OH [IMPR]: Race conditions mutex the waitForResult functions with send goal functions?

        // Return false if one of the clients dit not return a result within the timeout, true otherwise
        return all_success; 
    }

    bool waitForSuccess(const std::string& client_name, const ros::Duration& timeout = ros::Duration(0.0))
    {
        return getClient(client_name)->waitForSuccess(timeout);
    }

    bool waitForAllSuccess(const ros::Duration& timeout = ros::Duration(0.0))
    {
        ROS_DEBUG_NAMED(ROS_NAME_SMC, "waitForAllSuccess()");
      
        // Wait for all, busy, clients. NOTE this is not in parallel!           //! @todo OH [IMPR]: Do this in parallel.
        for(auto& client : clients_) 
            if( not waitForSuccess(client.second->getName(), timeout) )         //! @todo OH [IMPR]: Race conditions mutex the waitForSuccess functions with send goal functions?
                return false;

        return true; 
    }

    bool waitForFailed(const std::string& client_name, const ros::Duration& timeout = ros::Duration(0.0))
    {
        return not waitForSuccess(client_name, timeout);
    }

    //! @todo OH [CHECK]: Is this logically correct?
    bool waitForAllFailed(const ros::Duration& timeout = ros::Duration(0.0))
    {
        // cannot simply use 'return not waitForAllSuccess(timeout);' because we need to wait for the failed result of each client'
        ROS_DEBUG_NAMED(ROS_NAME_SMC, "waitForAllSuccess()");
      
        // Wait for all, busy, clients. NOTE this is not in parallel!           //! @todo OH [IMPR]: Do this in parallel.
        for(auto& client : clients_) 
            if( waitForSuccess(client.second->getName(), timeout) )             //! @todo OH [IMPR]: Race conditions mutex the waitForAllFailed functions with send goal functions?
                return false;

        return true; 
    }

    void abort()
    {
        sendServerResult(false);
    }

    void sendServerResult( bool succes, const Result result = Result(), const ros::Duration& timeout = ros::Duration(0,0))
    {   
        ROS_DEBUG_NAMED(ROS_NAME_SMC, "sendServerResult");

        // If a timeout is specified, give the clients some time to finish
        if( timeout.toSec() != 0.0 )
            waitForAllResults(timeout);

        // All clients will be canceled, if they are still active
        cancelAllClients();

        // Check if the CB_serverPreempt or another thread is already setting the state of the current goal.
        if( set_state_mutex_.try_lock() )
        {
            ROS_DEBUG_NAMED(ROS_NAME_SMC, "sendServerResult, got LOCK");

            // Check if the server is active
            if(server_->isActive() and not server_->isPreemptRequested() )
            {
                ROS_DEBUG_NAMED(ROS_NAME_SMC, "sendServerResult, server active");

                if (succes)
                {
                    ROS_DEBUG_NAMED(ROS_NAME_SMC, "sendServerResult, set succeeded.");
                    server_->setSucceeded(result, "SMC set this goal succeeded.");
                }
                else
                {
                    ROS_DEBUG_NAMED(ROS_NAME_SMC, "sendServerResult, set aborted.");
                    server_->setAborted(result, "SMC set this goal to aborted.");
                }
            }
            else
                ROS_DEBUG_NAMED(ROS_NAME_SMC, "sendServerResult, server NOT active");

            set_state_mutex_.unlock();
        }
        else
            ROS_DEBUG_NAMED(ROS_NAME_SMC, "sendServerResult, NOT LOCK");

        ROS_DEBUG_NAMED(ROS_NAME_SMC, "sendServerResult, done.");
    }
    
    void sendServerFeedback( const Feedback& feedback )
    {
        ROS_DEBUG_NAMED(ROS_NAME_SMC, "sendServerFeedback");

        //! @todo OH [IMPR]: Mutex? Because what if already canceled? Not here anyways
        server_->publishFeedback(feedback);
    }

    // Returns true if client is succesfully canceled or was not busy, return false if timed-out or if client does not exist.
    bool cancelClient(const std::string& client_name, const ros::Duration& timeout = ros::Duration(SMC_DEFAULT_CANCEL_TIMEOUT))
    {
        ROS_INFO_NAMED(ROS_NAME_SMC, "Canceling client '%s'.", client_name.c_str());
        // Check if this client name is actualy registred with this SMC.
        if(!hasClient(client_name))
        {
            ROS_WARN_NAMED(ROS_NAME_SMC, "Trying to cancel non-registred client '%s'.", client_name.c_str());
            return false;
        }

        if( not clients_[client_name]->hasGoalOutstanding() )
            return true;

        if(clients_[client_name]->cancelGoal(timeout))
        {
            ROS_INFO_NAMED(ROS_NAME_SMC, "Canceled client: %s", client_name.c_str());
            return true;
        }

        return false;
    }

    // True if all ACTIVE clients have been canceled, false otherwise
    bool cancelAllClients(const ros::Duration& timeout = ros::Duration(SMC_DEFAULT_CANCEL_TIMEOUT))
    {
        ROS_INFO_NAMED(ROS_NAME_SMC, "Canceling %d clients.", (int)clients_.size());
        
        unsigned int canceledCount  = 0;
        unsigned int activeCount    = 0;

        for( const auto& a_pair : clients_ ) 
        {
            if( a_pair.second->hasGoalOutstanding() )
            {
                activeCount++;
                if( a_pair.second->cancelGoal(timeout) )
                {
                    canceledCount++;
                    ROS_INFO_NAMED(ROS_NAME_SMC, "Canceled client: %s", a_pair.first.c_str());
                }
            }
        }
        
        ROS_INFO_NAMED(ROS_NAME_SMC, "%d/%d active clients canceled, %d clients where inactive.", canceledCount, activeCount, (unsigned int)clients_.size() - activeCount);

        // Return true if all active clients have been canceled
        if(activeCount == canceledCount)
            return true;

        return false;
    }

    template <class ClientActionType>
    ComplexClient<ClientActionType>* castComplexClientBaseToComplexClientPtr(ComplexClientBase* ComplexClientBase)
    {
        ComplexClient<ClientActionType>* complex_client;
        try
        {
            complex_client = dynamic_cast<ComplexClient<ClientActionType>*>(ComplexClientBase);
        }
        catch (const std::bad_cast& e)
        {
            ROS_ERROR("Dynamic cast error: %s", e.what());
            throw e;
        }

        return complex_client;
    }

    bool hasClient(const std::string& client_name)
    {
        if(clients_.find(client_name) == clients_.end())
            return false;

        return true;
    }

    bool hasActiveGoal()
    { 
        return server_->isActive();
    }

    bool isClientBusy(const std::string& client_name)
    {
        // Check if this client name is actualy registred with this SMC.
        if(!hasClient(client_name))
        {
            ROS_WARN_NAMED(ROS_NAME_SMC, "Trying to check if non-registred client '%s' is busy.", client_name.c_str());
            return false;
        }

        return clients_[client_name]->hasGoalOutstanding();
    }

    bool hasBusyClients()
    {
        for(const auto& a_pair : clients_) 
        {
            if(a_pair.second->hasGoalOutstanding())
                return true;
        }

        return false;
    }

    int getNumberOfBusyClients()
    {
        int count = 0;
        for(const auto& a_pair : clients_) 
        {
            if(a_pair.second->hasGoalOutstanding())
                count++;
        }

        return count;
    }

    std::vector<ComplexClientBase*> getBusyClients()
    {
        std::vector<ComplexClientBase*> busy_clients;
        for(const auto& a_pair : clients_) 
        {
            if(a_pair.second->hasGoalOutstanding())
                busy_clients.push_back(a_pair.second);
        }

        return busy_clients;
    }

    GoalConstPtr getLastGoal()
    {
        std::lock_guard<std::mutex> lock(last_goal_mutex_);
        return last_goal_;
    }

    std::string getName()
    {
        return server_name_;
    }

  private:

    ComplexClientBase* getClient(const std::string& client_name)
    {
        const auto& found = clients_.find(client_name);
        
        if(found == clients_.end())
            throw std::out_of_range("Client does not exist.");

        return found->second;
    }

    void CB_serverGoalReceived(const GoalConstPtr& goal)
    {

        ROS_DEBUG_NAMED(ROS_NAME_SMC, "CB_serverGoalReceived: start");
        
        // Check if the goal got canceled in the mean time
        if( server_->isPreemptRequested() )
        {
            ROS_DEBUG_NAMED(ROS_NAME_SMC, "CB_serverGoalReceived, Preempted! NOT calling user callback.");
            // server_->setPreempted(Result(), "Preempted by SMC");
            // CB_serverPreempt();
        }
        else
        {
            // Call custom work callback
            ROS_DEBUG_NAMED(ROS_NAME_SMC, "CB_serverGoalReceived, no preempt calling user callback.");

            server_work_cb_(goal, this);

            // Store previous goal
            last_goal_mutex_.lock();
            last_goal_ = goal;
            last_goal_mutex_.unlock();

            // Wait until server result has been set or the node is shutdown
            while( n_.ok() and server_->isActive() )
            {
                ROS_DEBUG_THROTTLE_NAMED(0.1, ROS_NAME_SMC, "CB_serverGoalReceived, waiting for server result to be set.");
            }
        }

        ROS_DEBUG_NAMED(ROS_NAME_SMC, "CB_serverGoalReceived, done.");
    }

        // Only gets called if for current goal
    void CB_serverPreempt()
    {
        ROS_DEBUG_NAMED(ROS_NAME_SMC, "CB_serverPreempt, thread id: %s", boost::lexical_cast<std::string>(boost::this_thread::get_id()).c_str());
        
        // Check if the sendServerResult is already setting the state of the current goal.
        if(set_state_mutex_.try_lock())
        {
            ROS_DEBUG_NAMED(ROS_NAME_SMC, "CB_serverPreempt, got set_state_mutex_.");

            // Check if the server is in an active state
            if(server_->isActive())
            {
                ROS_DEBUG_NAMED(ROS_NAME_SMC, "CB_serverPreempt, server is active.");

                // All clients will be canceled if deleted  
                ROS_INFO_NAMED(ROS_NAME_SMC, "The server received a preempt request, canceling clients.");
                cancelAllClients();

                // Call custom preempt callback
                if(server_preempt_cb_ != NULL)
                    server_preempt_cb_(this);

                // When all clients are canceled set server as preempted
                ROS_INFO_NAMED(ROS_NAME_SMC, "Setting server as being preempted.");
                server_->setPreempted(Result(), "Preempted by SMC");

                ROS_INFO_NAMED(ROS_NAME_SMC, "Server '%s' preempted.", server_name_.c_str());
            }
            else
                ROS_DEBUG_NAMED(ROS_NAME_SMC, "CB_serverPreempt, server is NOT active.");


            set_state_mutex_.unlock();
        }
        else
            ROS_DEBUG_NAMED(ROS_NAME_SMC, "CB_serverPreempt, did NOT get set_state_mutex_.");
        
        ROS_DEBUG_NAMED(ROS_NAME_SMC, "CB_serverPreempt, done.");
    }



  private:
    ros::NodeHandle                                 n_;
    
    SimpleActionServer<ServerActionType>*           server_;
    string                                          server_name_;
    bool                                            server_started_;
    ServerWorkCallback                              server_work_cb_;
    ServerPreemptCallback                           server_preempt_cb_;

    thread_safe::map<string, ComplexClientBase*>    clients_;   
    std::string                                     latest_client_;

    std::mutex                                      set_state_mutex_;

    GoalConstPtr                                    last_goal_;
    std::mutex                                      last_goal_mutex_;
};

#endif // SERVER_MULTIPLE_CLIENT_HPP
