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

#include "rose20_common/common.hpp"
#include "rose20_common/ros_name.hpp"
#include "rose20_common/server_multiple_client/complex_client.hpp"

#include "rose20_common/smc_dummy_serverAction.h"
#include "rose20_common/smc_dummy_serverActionGoal.h"
#include "rose20_common/smc_dummy_serverActionResult.h"
#include "rose20_common/smc_dummy_serverActionFeedback.h"

#include "thread_safe_stl_containers/thread_safe_map.h"

#define ROS_NAME_SMC    (ROS_NAME + "|SMC")

using namespace std;
using namespace actionlib;

template <class ServerActionType = rose20_common::smc_dummy_serverAction> class ServerMultipleClient
{
  public:
    ACTION_DEFINITION(ServerActionType)
    typedef boost::function<void (const GoalConstPtr& goal, ServerMultipleClient<ServerActionType>* smc) > ServerWorkCallback;
    typedef boost::function<void (ServerMultipleClient<ServerActionType>* smc) > ServerPreemptCallback;

    ServerMultipleClient(ros::NodeHandle n, string server_name, ServerWorkCallback      server_work_cb = NULL, 
                                                                ServerPreemptCallback   server_preempt_cb = NULL)
        : server_name_(server_name)
        , n_(n)
        , server_work_cb_(server_work_cb)
        , server_preempt_cb_(server_preempt_cb)
        , server_started_(false)
        , has_active_goal_(false)
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

        lock_for_result_.try_lock();
        lock_for_result_.unlock();

        cancelAllGoals();
        
        pair<string, ComplexClientBase*> a_pair; 
        BOOST_FOREACH(a_pair, clients_) 
        {
            delete a_pair.second;
            ROS_INFO_NAMED(ROS_NAME_SMC, "Removed client: %s", a_pair.first.c_str());
        }

        delete server_;
        server_started_ = false;

        lock_for_result_.try_lock();
        lock_for_result_.unlock();
    }

    void startServer()
    {
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
    bool addClient(string client_name,  typename ComplexClient<ClientActionType>::SimpleDoneCallback        CB_custom_done      = NULL,
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

    template <class ClientActionType>
    bool sendGoal(const typename ComplexClient<ClientActionType>::Goal& goal)
    {
        if(clients_.size() != 1)
        {
            ROS_ERROR_NAMED(ROS_NAME_SMC, "Error while trying to send goal to '%d' clients, 'sendGoal()' can only be used to send this goal to one client.", (int)clients_.size());
            return false;
        }

        pair<string, ComplexClientBase*> a_pair;
        BOOST_FOREACH(a_pair, clients_) 
        {
            if(!castComplexClientBaseToComplexClientPtr<ClientActionType>(clients_[a_pair.first])->sendComplexGoal(goal))
                return false;                           //! @todo cancel or do something else with the goals that did get send out?
        }

        last_active_client_ = a_pair.first;
        
        return true;
    }   

    template <class ClientActionType>
    bool sendGoal(const typename ComplexClient<ClientActionType>::Goal& goal, string client_name)
    {
        ROS_DEBUG_NAMED(ROS_NAME_SMC, "sendGoal -> %s", client_name.c_str());

        if(clients_.count(client_name) == 0)
        {
            ROS_ERROR_NAMED(ROS_NAME_SMC, "Error while trying to send goal via client '%s', 'client does not exist.", client_name.c_str());
            return false;
        }

        if(!castComplexClientBaseToComplexClientPtr<ClientActionType>(clients_[client_name])->sendComplexGoal(goal))
            return false;

        last_active_client_ = client_name;

        return true;
    }

    template <class ClientActionType>
    bool sendGoals(const typename ComplexClient<ClientActionType>::Goal& goal, vector<string> client_names)
    {
        if(hasBusyClients())
        {
            ROS_ERROR_NAMED(ROS_NAME_SMC, "Error while trying to send goal, 'sendGoal()' already has outstanding goal(s).");
            return false;
        }

        string client_name;
        BOOST_FOREACH(client_name, client_names)
        {
            if(!sendGoal<ClientActionType>(goal, client_name))
                return false;                                           //! @todo cancel or do something else with the goals that did get send out?
        }

        return true;
    }

    template <class ClientActionType>
    typename ComplexClient<ClientActionType>::ResultConstPtr getResultLastActiveClient()
    {
        ROS_DEBUG_NAMED(ROS_NAME_SMC, "getResultLastActiveClient()");
        return castComplexClientBaseToComplexClientPtr<ClientActionType>(clients_[last_active_client_])->getLastResult();
    }

    bool waitForResult(const ros::Duration& timeout = ros::Duration(0.0))
    {
        ROS_DEBUG_NAMED(ROS_NAME_SMC, "waitForResult()");
        int busy_clients_count = getNumberOfBusyClients();
        if(busy_clients_count == 0)
        {
            ROS_DEBUG_NAMED(ROS_NAME_SMC, "Waiting on clients while nobody active.");
            return true;
        }
        else
        {
            bool all_succes = true;
            // Wait for all clients, NOTE this is not in parallel!          //! @todo OH: Is this correct?
            vector<ComplexClientBase*> busy_clients = getBusyClients();
            ComplexClientBase* client; 
            BOOST_FOREACH(client, busy_clients) 
                all_succes = all_succes && client->waitForResult(timeout);

            // Return false if at least one of the clients dit not return a result within the timeout, true otherwise
            return all_succes; 
        }
    }

    void CB_serverGoalReceived(const GoalConstPtr& goal)
    {
        ROS_DEBUG_NAMED(ROS_NAME_SMC, "CB_serverGoalReceived, thread id: %s", boost::lexical_cast<std::string>(boost::this_thread::get_id()).c_str());
        lock_for_result_.lock();

        has_active_goal_ = true;
        last_goal_ = goal;

        // Call custom work callback
        if(server_work_cb_ != NULL)
            server_work_cb_(goal, this);

        // Wait for the result feedback to unlock the mutex
        lock_for_result_.lock();
        lock_for_result_.unlock();
    }

    template <class ClientActionType>
    void sendServerResult( bool succes, const typename ComplexClient<ClientActionType>::Result& result, const ros::Duration& timeout = ros::Duration(0,0))
    {   
        if(lock_for_result_.try_lock())         
        {
            //! @todo This Should be most likly indeed be commented out ROS_ERROR_NAMED(ROS_NAME_SMC, "Error sending result, 'sendServerResult()' cannot send a result before setting a goal. You have a bug in your code.");
            lock_for_result_.unlock();
            return;
        }

        // If a timeout is specified, give the clients some time to finish
        if(timeout.toSec() != 0.0)
            waitForResult(timeout);

        // All clients will be canceled, if they are still active, if result is end  
        cancelAllGoals();
        
        if (succes)
            server_->setSucceeded(result);
        else
            server_->setAborted(result);

        has_active_goal_ = false;

        lock_for_result_.unlock();
    }
    
    template <class ClientActionType>
    void sendServerFeedback( const typename ComplexClient<ClientActionType>::Feedback& feedback )
    {
        ROS_DEBUG_NAMED(ROS_NAME_SMC, "sendServerFeedback");
        //! @todo OH: Mutex? Because what if already canceled?
        server_->publishFeedback(feedback);
    }

    void CB_serverPreempt()
    {
        ROS_DEBUG_NAMED(ROS_NAME_SMC, "CB_serverPreempt, thread id: %s", boost::lexical_cast<std::string>(boost::this_thread::get_id()).c_str());
        
        if(lock_for_result_.try_lock())
        {
            ROS_ERROR_NAMED(ROS_NAME_SMC, "Error preempting, 'CB_serverPreempt()' cannot cancel a goal before setting a goal. You have a bug in your code.");
            lock_for_result_.unlock();
            return;
        }

        ROS_INFO_NAMED(ROS_NAME_SMC, "The server received a cancel request, canceling clients... (thread id: %s)", boost::lexical_cast<std::string>(boost::this_thread::get_id()).c_str());

        // All clients will be canceled if deleted  
        cancelAllGoals();

        // Call custom preempt callback
        if(server_preempt_cb_ != NULL)
            server_preempt_cb_(this);


        // When all clients are canceled set server as preempted
        lock_for_result_.unlock();
        ROS_INFO_NAMED(ROS_NAME_SMC, "Canceling server....");
        server_->setPreempted();

        has_active_goal_ = false;
        ROS_INFO_NAMED(ROS_NAME_SMC, "Done.");

        lock_for_result_.unlock();
    }

    // Returns true if client is succesfully canceled, return false if timed-oput or if client does not exist.
    bool cancelClient(string client_name, const ros::Duration& timeout = ros::Duration(SMC_DEFAULT_CANCEL_TIMEOUT))
    {
        // Check if this client name is actualy registred with this SMC.
        if(!hasClient(client_name))
        {
            ROS_WARN_NAMED(ROS_NAME_SMC, "Trying to cancel non-registred client '%s'.", client_name.c_str());
            return false;
        }

        //! @todo OH: Return boolean?
        if(clients_[client_name]->isBusy())
        {
            if(clients_[client_name]->cancelGoal(timeout))
            {
                ROS_INFO_NAMED(ROS_NAME_SMC, "Canceled client: %s", client_name.c_str());
                return true;
            }
        }

        return false;
    }

    // True if all ACTIVE clients have been canceled, false otherwise
    bool cancelAllGoals(const ros::Duration& timeout = ros::Duration(SMC_DEFAULT_CANCEL_TIMEOUT))
    {
        ROS_INFO_NAMED(ROS_NAME_SMC, "Canceling %d clients.", (int)clients_.size());
        
        unsigned int canceledCount  = 0;
        unsigned int activeCount    = 0;
        // pair<string, ComplexClientBase*> a_pair; 
        for(const auto& a_pair : clients_) 
        {
            if(a_pair.second->isBusy())
            {
                activeCount++;
                if(a_pair.second->cancelGoal(timeout))
                {
                    canceledCount++;
                    ROS_INFO_NAMED(ROS_NAME_SMC, "Canceled client: %s", a_pair.first.c_str());
                }
            }
        }
        
        if(canceledCount > 0)
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

    bool hasClient(string client_name)
    {
        if(clients_.find(client_name) == clients_.end())
            return false;

        return true;
    }

    bool hasActiveGoal()
    {
        return has_active_goal_;        //! @todo OH: Needs mutex?
    }

    bool isClientBusy(string client_name)
    {
        // Check if this client name is actualy registred with this SMC.
        if(!hasClient(client_name))
        {
            ROS_WARN_NAMED(ROS_NAME_SMC, "Trying to check if non-registred client '%s' is busy.", client_name.c_str());
            return false;
        }

        return clients_[client_name]->isBusy();
    }

    bool hasBusyClients()
    {
        for(const auto& a_pair : clients_) 
        {
            if(a_pair.second->isBusy())
                return true;
        }

        return false;
    }

    int getNumberOfBusyClients()
    {
        int count = 0;
        for(const auto& a_pair : clients_) 
        {
            if(a_pair.second->isBusy())
                count++;
        }

        return count;
    }

    vector<ComplexClientBase*> getBusyClients()
    {
        vector<ComplexClientBase*> busy_clients;
        for(const auto& a_pair : clients_) 
        {
            if(a_pair.second->isBusy())
                busy_clients.push_back(a_pair.second);
        }

        return busy_clients;
    }

    GoalConstPtr getLastGoal()
    {
        return last_goal_;
    }

    string getName()
    {
        return server_name_;
    }



  private:
    boost::mutex                                    lock_for_result_;
    SimpleActionServer<ServerActionType>*           server_;
    thread_safe::map<string, ComplexClientBase*>    clients_;   
    GoalConstPtr                                    last_goal_;
    ros::NodeHandle                                 n_;
    string                                          server_name_;
    bool                                            server_started_;
    bool                                            has_active_goal_;

    string                                          last_active_client_;
 
    ServerWorkCallback                              server_work_cb_;
    ServerPreemptCallback                           server_preempt_cb_;

};

#endif // SERVER_MULTIPLE_CLIENT_HPP
