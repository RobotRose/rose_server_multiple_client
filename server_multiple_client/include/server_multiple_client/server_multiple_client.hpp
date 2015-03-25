/***********************************************************************************
* Copyright: Rose B.V. (2013-2015)
*
* Revision History:
*  Author: Okke Hendriks
*  Date  : 2014/01/30
*     - File created.
*
* Description:
*  ServerMultipleClient (SMC) class, header only due to the templated nature.
*  The SMC can contain zero or one SimpleActionServer to which standard ROS SimpleActionClients can connect.
*  The SMC can also contain zero or more ComplexClients, a ComplexClient is a wrapper around the standard ROS SimpleActionClient.
*  When the server side is to be used a ServerWorkCallback must be registerd, otherwise the SMC cannot be started.
*  Clients can only be added to the SMC when not active (not started).
*  
*  There is currently a bug/fluke in the ROS logger which causes the ComplexClient debug information to appear with an incorrect name (see https://github.com/ros/ros_comm/issues/561).
* 
***********************************************************************************/

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

#define ROS_NAME_SMC                        (ROS_NAME + "|SMC")
#define SMC_DEFAULT_CANCELING_TIMEOUT       0.01                  // canceling_timeout specifies the time to wait, before canceling a goal which was send in the past. A canceling_timeout of 0 specifies an infinite timeout.

/**
 * @brief The Server Multiple Client (SMC) class, can contains a SimpleActionServer and zero or more ComplexClients.
 * @details The SMC can contain zero or one SimpleActionServer to which standard ROS SimpleActionClients can connect. 
 * The SMC can also contain zero or more ComplexClients, a ComplexClient is a wrapper around the standard ROS SimpleActionClient.
 * 
 * @tparam ServerActionType = server_multiple_client_msgs::smc_dummy_serverAction The server side action type
 * @return An instance of an SMC, not started. Call startServer() after adding all clients to start the SMC.
 */
template <class ServerActionType = server_multiple_client_msgs::smc_dummy_serverAction> class ServerMultipleClient
{
  public:
    ACTION_DEFINITION(ServerActionType)
    typedef boost::function<void (const GoalConstPtr& goal, ServerMultipleClient<ServerActionType>* smc) > ServerWorkCallback;
    typedef boost::function<void (ServerMultipleClient<ServerActionType>* smc) > ServerPreemptCallback;

    /**
     * @brief Construct an SMC object.
     * @details Pass in the node handle and server name. The server work callback function and the preemt callback function
     * are optional. The server work callback function is however required if the server side of the SMC will be used.
     * 
     * @param n ROS nodehandle
     * @param server_name Name of this SMC, will also define the name which is used to communitcate with the server. The name will be passed to the internal
     * SimpleActionServer.
     * @param server_work_cb Callback function which is called whenever a goal is received by the SimpleActionServer.
     * @param server_preempt_cb Callback funtion which is called whenever a preempt is handled by the SimpleActionServer.
     */
    ServerMultipleClient(ros::NodeHandle n, const std::string& server_name, ServerWorkCallback      server_work_cb = NULL, 
                                                                            ServerPreemptCallback   server_preempt_cb = NULL)
        : server_name_(server_name)
        , n_(n)
        , server_work_cb_(server_work_cb)
        , server_preempt_cb_(server_preempt_cb)
        , server_started_(false)
    {
        // Create server
        server_ = new actionlib::SimpleActionServer<ServerActionType>(n_, server_name_, boost::bind(&ServerMultipleClient<ServerActionType>::CB_serverGoalReceived, this, _1) , false);    
        
        // Register preempt received callback
        server_->registerPreemptCallback(boost::bind(&ServerMultipleClient<ServerActionType>::CB_serverPreempt, this));  

        ROS_INFO_NAMED(ROS_NAME_SMC, "Created server: %s", server_name_.c_str());
    }

    /**
     * @brief Deconstruct this SMC object.
     * @details If there is an active goal it will be aborted. All clients will also be canceled. 
     */
    ~ServerMultipleClient()
    {
        ROS_INFO_NAMED(ROS_NAME_SMC, "Shutting down SMC: `%s`", server_name_.c_str());

        cancelAllClients();
        server_started_ = false;
        
        std::pair<std::string, ComplexClientBase*> a_pair; 
        BOOST_FOREACH(a_pair, clients_) 
        {
            delete a_pair.second;
            ROS_INFO_NAMED(ROS_NAME_SMC, "Removed client: %s", a_pair.first.c_str());
        }

        delete server_;

    }

    /**
     * @brief Start the SMC.
     * @details The SMC will be started if a server work callback function has been registered. If so, the internal SimpleActionServer will be started. 
     * @return TRUE if successfully started, FALSE if there was not server work callback function.
     */
    bool startServer()
    {
        // Check if a server work callback has been registered.
        if(server_work_cb_ == NULL)
        {
            ROS_ERROR_NAMED(ROS_NAME_SMC, "Unable to start server '%s' because no server work callback has been registered.", server_name_.c_str());
            return false;
        }            

        // Start the server
        server_->start();
        server_started_ = true; 

        ROS_INFO_NAMED(ROS_NAME_SMC, "Started server: %s", server_name_.c_str());

        return true;
    }

    /**
     * @brief Stop the SMC.
     * @details Also stops the internal SimpleActionServer.
     */
    void stopServer()
    {
        // Stop the SimpleServer
        server_->stop();
        server_started_ = false; 

        ROS_INFO_NAMED(ROS_NAME_SMC, "Stopped server: %s", server_name_.c_str());
    }

    /**
     * @return Pointer to the internal SimpleActionServer
     */
    actionlib::SimpleActionServer<ServerActionType>* getSimpleServer()
    {
        return server_;
    }

    /**
     * @brief Add a client to the SMC
     * @details By providing a client name and optionally the callback functions a ComplexClient with that name will be created and stored in the list of internal clients.
     * 
     * @param client_name The name of the client to be created, this has to be the same name of the ActionLib server it has to communicate with.
     * @param CB_custom_done Optional callback function which will be called when the client has finished processing a goal succesfully.
     * @param CB_custom_fail Optional callback function which will be called when the client has finished processing a goal unsucesfully.
     * @param CB_custom_active Optional callback function which will be called when the client starts processing a goal.
     * @param CB_custom_feedback Optional callback function which will be called when the client has received feedback of its server.
     * @return True if the client has succesfully been added, false if the server has already been started or if a client with the same name had already been added.
     */
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
        
        clients_.insert( std::pair<std::string, ComplexClientBase*>(client_name, complex_client));

        ROS_INFO_NAMED(ROS_NAME_SMC, " Added client: %s", client_name.c_str());

        return true;
    }

    //! @todo OH [IMPR]: Add send and wait functionality.
    //! @todo OH [IMPR]: Add send and fail send fail result on not completing.

    /**
     * @brief Send a goal to a single server using the only client added.
     * @details A goal will be send to the only client added to this SMC. If there are zero or more than one clients this call will fail.
     * 
     * @tparam ClientActionType Provide the action type of the client when calling this funcion.
     * @param goal Goal which will be send to the server of the client.
     * @param canceling_timeout Specifies the time to wait before canceling a goal which was send in the past. A canceling_timeout of 0 specifies an infinite timeout.
     * @return false if the number of client is not equqal to 1. Otherwise it will return if the ComplexClient was able to send the goal or not.
     */
    template <class ClientActionType>
    bool sendGoal(const typename ComplexClient<ClientActionType>::Goal& goal, const float& canceling_timeout = SMC_DEFAULT_CANCELING_TIMEOUT)
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
            return sendGoal<ClientActionType>(goal, clients_.begin()->first, canceling_timeout);
        }
    }

    /**
     * @brief Send a goal to a server using one of the, previously added, clients.
     * @details A goal will be send to the server of the client with the provided name.
     * 
     * @tparam ClientActionType Provide the action type of the client when calling this funcion.
     * @param goal Goal which will be send to the server of the client.
     * @param client_name Name of the client to use to send the goal. This is the same name as the name of the server which the goal will be send to.
     * @param canceling_timeout Specifies the time to wait before canceling a goal which was send in the past. A canceling_timeout of 0 specifies an infinite timeout.
     * @return false if there are no mathcing client_name has been found. Otherwise it will return if the ComplexClient was able to send the Goal or not.
     * @throw If the cast from the ComplexClientBase class to the ComplexClient<ClientActionType>* class fails it will throw an std::bad_cast error. 
     */
    template <class ClientActionType>
    bool sendGoal(const typename ComplexClient<ClientActionType>::Goal& goal, const std::string& client_name, const float& canceling_timeout = SMC_DEFAULT_CANCELING_TIMEOUT)
    {
        ROS_DEBUG_NAMED(ROS_NAME_SMC, "sendGoal -> %s", client_name.c_str());

        if(clients_.count(client_name) == 0)
        {
            ROS_ERROR_NAMED(ROS_NAME_SMC, "Error while trying to send goal via client '%s', 'client does not exist.", client_name.c_str());
            return false;
        }

        if(!castComplexClientBaseToComplexClientPtr<ClientActionType>(clients_[client_name])->sendComplexGoal(goal, canceling_timeout))
            return false;

        latest_client_ = client_name;

        return true;
    }

    /**
     * @brief Send an identical goal to multiple clients.
     * @details Will call the sendGoal function with all the names provided in the client_names list. All clients called need to be of the same ClientActionType.
     * If one of the SendGoal calls fails, all clients will be canceled and the function will return false.
     * 
     * @tparam ClientActionType Provide the action type of the called client(s) when calling this funcion.
     * @param goal Goal which will be send to the server(s) of the client(s).
     * @param client_names List of client names which are to be called. The clients need to be of the same ClientActionType.
     * @param canceling_timeout Specifies the time to wait before canceling a goal which was send in the past. A canceling_timeout of 0 specifies an infinite timeout.
     * @return false if there are goals outstanding. It will return true only if all calls to sendGoal finish sucesfully.
     */
    template <class ClientActionType>
    bool sendGoals(const typename ComplexClient<ClientActionType>::Goal& goal, const std::vector<std::string>& client_names, const float& canceling_timeout = SMC_DEFAULT_CANCELING_TIMEOUT)
    {
        if( hasBusyClients() )
        {
            ROS_ERROR_NAMED(ROS_NAME_SMC, "Error while trying to send goal, 'sendGoals()' already has outstanding goal(s).");
            return false;
        }

        for(const auto& client_name : client_names)
        {
            if( not sendGoal<ClientActionType>(goal, client_name, canceling_timeout) )
            {
                cancelAllClients();
                return false;
            }
        }

        return true;
    }

    /**
     * @brief Get the last received result of a certain client.
     * @details The result of the client with the specified name will be returned.
     * 
     * @tparam ClientActionType Provide the action type of the client when calling this funcion.
     * @param client_name Name of the client to get the result from.
     * @return A const pointer to the last received result of the specified client.
     * @throw If the cast from the ComplexClientBase class to the ComplexClient<ClientActionType>* class fails it will throw an std::bad_cast error. 
     */
    template <class ClientActionType>
    const typename ComplexClient<ClientActionType>::ResultConstPtr getResult(const std::string& client_name)
    {
        return castComplexClientBaseToComplexClientPtr<ClientActionType>(getClient(client_name))->getLastResult();
    }

    /**
     * @brief Get the last received result of the client which was active last. 
     * @details The result of the client wich was active the shortest amount of time ago will be returned.
     * 
     * @tparam ClientActionType Provide the action type of the client when calling this funcion.
     * @return A const pointer to the last received result of the client which was active last.
     * @throw If the cast from the ComplexClientBase class to the ComplexClient<ClientActionType>* class fails it will throw an std::bad_cast error. 
     */
    template <class ClientActionType>
    typename ComplexClient<ClientActionType>::ResultConstPtr getResultLastestClient()
    {
        ROS_DEBUG_NAMED(ROS_NAME_SMC, "getResultLastestClient()");
        return castComplexClientBaseToComplexClientPtr<ClientActionType>(clients_[latest_client_])->getLastResult();
    }

    /**
     * @brief Wait for, the only client added, to have a result available.
     * 
     * @param timeout The maximal time the function will block waiting on a result. A timeout of 0.0 specifies an infinite timeout.
     * @return false if there is not exactly one client added to this SMC or the return value of the ComplexClient::waitForResult function.
     */
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

    /**
     * @brief Wait for, the specified client, to have a result available.
     * 
     * @param timeout The maximal time the function will block waiting on a result. A timeout of 0.0 specifies an infinite timeout.
     * @return false if there is not exactly one client added to this SMC or the return value of the ComplexClient::waitForResult function.
     */
    bool waitForResult(const std::string& client_name, const ros::Duration& timeout = ros::Duration(0.0))
    {
        return getClient(client_name)->waitForResult(timeout);
    }

    /**
     * @brief Wait, with a optional timeout, until all client have provided an result.
     * @details There will be a waitForResult call for each client sequentially with optionally an provided timeout.
     * 
     * @param timeout The maximal time the function will block waiting on a result. A timeout of 0.0 specifies an infinite timeout.
     * @return true only if all calls to waitForResult for all clients returned true.
     */
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

    /**
     * @brief Wait until the specified client returns a result, then check if this was a successfull result.
     * 
     * @param client_name Specify the client.
     * @param timeout The maximal time the function will block waiting on a result. A timeout of 0.0 specifies an infinite timeout.
     * 
     * @return true only if the client returned a successfull result within the specified timeout.
     */
    bool waitForSuccess(const std::string& client_name, const ros::Duration& timeout = ros::Duration(0.0))
    {
        return getClient(client_name)->waitForSuccess(timeout);
    }

    /**
     * @brief Wait until the all clients returns a result, then check if these were all successfull results.
     * @details The timeout will be applied serially to every client.
     *  
     * @param timeout The maximal time the function will block waiting on a result per client. A timeout of 0.0 specifies an infinite timeout.
     * 
     * @return true only if all the clients returned a successfull result within the specified timeout.
     */
    bool waitForAllSuccess(const ros::Duration& timeout = ros::Duration(0.0))
    {
        ROS_DEBUG_NAMED(ROS_NAME_SMC, "waitForAllSuccess()");
      
        // Wait for all, busy, clients. NOTE this is not in parallel!           //! @todo OH [IMPR]: Do this in parallel.
        for(auto& client : clients_) 
            if( not waitForSuccess(client.second->getName(), timeout) )         //! @todo OH [IMPR]: Race conditions mutex the waitForSuccess functions with send goal functions?
                return false;

        return true; 
    }

    /**
     * @brief Wait until the specified client returns a result, then check if this was a failed result.
     * 
     * @param client_name Specify the client.
     * @param timeout The maximal time the function will block waiting on a result. A timeout of 0.0 specifies an infinite timeout.
     * 
     * @return true only if the client returned a failed result within the specified timeout.
     */
    bool waitForFailed(const std::string& client_name, const ros::Duration& timeout = ros::Duration(0.0))
    {
        return not waitForSuccess(client_name, timeout);
    }

    /**
     * @brief Wait until the all clients returns a result, then check if these were all failed results.
     * @details The timeout will be applied serially to every client.
     * 
     * @param timeout The maximal time the function will block waiting on a result per client. A timeout of 0.0 specifies an infinite timeout.
     * 
     * @return true only if all the clients returned a failed result within the specified timeout.
     */
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

    /**
     * @brief Send a failed server result.
     * @details sendServerResult(false) will be called. Causing all client to be canceled and the server goal to be aborted.
     */
    void abort()
    {
        sendServerResult(false);
    }

    /**
     * @brief Set the server result to either success or false.
     * @details The success parameter specifies if the internal SimpleActionServer state will transition to success or false. Provided.
     * Before this happens however all client will be canceled.
     * 
     * @param success Specifies if the server result will be set to success (true) or aborted (false).
     * @param result Optional result message of the ActionServerType result type.
     * @param timeout The maximal time the function will block waiting on a result per client. A timeout of 0.0 specifies an infinite timeout.
     */
    void sendServerResult( bool success, const Result result = Result(), const ros::Duration& timeout = ros::Duration(0,0))
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
            if( server_->isActive() and not server_->isPreemptRequested() )
            {
                ROS_DEBUG_NAMED(ROS_NAME_SMC, "sendServerResult, server active");

                if( success )
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
    
    /**
     * @brief Send server feedback of the client.
     * 
     * @param feedback Feedback to send.
     */
    void sendServerFeedback( const Feedback& feedback )
    {
        ROS_DEBUG_NAMED(ROS_NAME_SMC, "sendServerFeedback");

        //! @todo OH [IMPR]: Mutex? Because what if already canceled? Not here anyways
        server_->publishFeedback(feedback);
    }

    // 
    /**
     * @brief Cancels the specified client.
     * @details An optional timeout, which defaults to SMC_DEFAULT_CANCEL_TIMEOUT can be provided. 
     * If the client did was not active, it did not have an outstanding goal the client will be consdered to be canceled.
     * 
     * @param client_name Specifies the client to be canceled.
     * @param timeout The maximal time the function will block waiting for the server of the client to acknowledge a cancel. A timeout of 0.0 specifies an infinite timeout.
     * 
     * @return true if the specified client is succesfully canceled or was not busy to begin with, returns false if timed-out or if client does not exist.
     */
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

    /**
     * @brief Cancels all clients contained in the SMC.
     * @details Calls the cancelClient() function on all clients in the SMC.
     * 
     * @param timeout The maximal time the function will block waiting for the server of each client to acknowledge a cancel. A timeout of 0.0 specifies an infinite timeout.
     * @return True if all canelGoal() calls have been canceled sucesfully, false otherwise.
     */
    bool cancelAllClients(const ros::Duration& timeout = ros::Duration(SMC_DEFAULT_CANCEL_TIMEOUT))
    {
        ROS_DEBUG_NAMED(ROS_NAME_SMC, "Canceling %d clients.", (int)clients_.size());
        
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
                    ROS_DEBUG_NAMED(ROS_NAME_SMC, "Canceled client: %s", a_pair.first.c_str());
                }
            }
        }
        
        ROS_DEBUG_NAMED(ROS_NAME_SMC, "%d/%d active clients canceled, %d clients where inactive.", canceledCount, activeCount, (unsigned int)clients_.size() - activeCount);

        // Return true only if all active clients have been canceled
        if(activeCount == canceledCount)
            return true;

        return false;
    }

    /**
     * @brief Check if the specified client was added to this SMC.
     * 
     * @param client_name The client to check for.
     * @return Boolean returning if the client with the specified name was added to this SMC.
     */
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

    /**
     * @brief Check if a specific client is active
     * 
     * @param client_name Which client to check.
     * @return Boolean returning if the specified client was active or not.
     */
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

    /**
     * @brief Return if there are 1 or more active clients.
     * @return Boolean returning if there are more than 1 active clients.
     */
    bool hasBusyClients()
    {
        for(const auto& a_pair : clients_) 
        {
            if(a_pair.second->hasGoalOutstanding())
                return true;
        }

        return false;
    }

    /**
     * @brief Return the number of active clients.
     * @return For how many clients hasGoalOutstanding() returns true.
     */
    unsigned int getNumberOfBusyClients()
    {
        unsigned int count = 0;
        for(const auto& a_pair : clients_) 
        {
            if(a_pair.second->hasGoalOutstanding())
                count++;
        }

        return count;
    }

    /**
     * @brief Returns a list of active clients.
     * @details The number of client for which hasGoalOutstanding() returns true.
     * @return List of clients for which hasGoalOutstanding() was true.
     */
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

    /**
     * @brief Get the current goal of the server side of the SMC.
     * @return True of there is a current goal, false otherwise
     */
    bool getCurrentGoal(GoalConstPtr goal)
    {
        std::lock_guard<std::mutex> lock(goal_mutex);
        if ( current_goal_ != NULL )
        {
            goal = current_goal_;
            return true;
        }

        return false;
    }

    /**
     * @brief Get the last goal received by the server side of the SMC.
     * @return last_goal_
     */
    GoalConstPtr __attribute__((deprecated)) getLastGoal()
    {
        std::lock_guard<std::mutex> lock(goal_mutex);
        return last_goal_;
    }

    /**
     * @brief Get the last goal received by the server side of the SMC.
     * @return  True of there is a current goal, false otherwise
     */
    bool getLastGoal(GoalConstPtr goal)
    {
        std::lock_guard<std::mutex> lock(goal_mutex);
        if ( last_goal_ != NULL )
        {
            goal = last_goal_;
            return true;
        }

        return false;
    }    

    /**
     * @brief Get the name of the server.
     * @return server_name_
     */
    std::string getName()
    {
        return server_name_;
    }

  private:

    /**
     * @brief Cast a ComplexClientBase class to an ComplexClient<ClientActionType>* class.
     * @details Cast the base class which the provided pointer points to to a ComplexClient of the specified templated ClientActionType type using a dynamic_cast
     * 
     * @param ComplexClientBase The base class to be cast.
     * @return ComplexClient* of the ClientActionType type.
     * @throw Throws an std::bad_cast in case the cast fails.
     * 
     */
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

    /**
     * @brief Get a point to the ComplexClientBase with the specified name.
     * 
     * @param client_name The name of the client.
     * @return Pointer to the ComplexClientBase class in the clients_ list.
     */
    ComplexClientBase* getClient(const std::string& client_name)
    {
        const auto& found = clients_.find(client_name);
        
        if(found == clients_.end())
            throw std::out_of_range("Client does not exist.");

        return found->second;
    }

    /**
     * @brief Internal SMC server work callback function.
     * @details This is the function registered as server work function at the internal SimpleActionServer.
     * 
     * @param goal The goal which has been received by the SimpleActionServer.
     */
    void CB_serverGoalReceived(const GoalConstPtr& goal)
    {

        ROS_DEBUG_NAMED(ROS_NAME_SMC, "CB_serverGoalReceived: start");
        
        // Check if the goal got canceled in the mean time
        if( server_->isPreemptRequested() )
        {
            ROS_DEBUG_NAMED(ROS_NAME_SMC, "CB_serverGoalReceived, Preempted! NOT calling user callback.");
            server_->setPreempted(Result(), "Preempted by SMC");
        }
        else
        {
            // Call custom work callback
            ROS_DEBUG_NAMED(ROS_NAME_SMC, "CB_serverGoalReceived, no preempt calling user callback.");

            // Store current goal
            goal_mutex.lock();
            current_goal_   = goal;
            goal_mutex.unlock();

            server_work_cb_(goal, this);

            // Store previous goal
            goal_mutex.lock();
            current_goal_   = GoalConstPtr(); // Reset pointer
            last_goal_      = goal;
            goal_mutex.unlock();

            // Wait until server result has been set or the node is shutdown
            while( n_.ok() and server_->isActive() )
            {
                ROS_DEBUG_THROTTLE_NAMED(0.1, ROS_NAME_SMC, "CB_serverGoalReceived, waiting for server result to be set.");
            }
        }

        ROS_DEBUG_NAMED(ROS_NAME_SMC, "CB_serverGoalReceived, done.");
    }

        // 
    /**
     * @brief Internal SMC server prempt callback function.
     * @details This is the function registered as preempt function at the internal SimpleActionServer. It will only get called if the preempt was for the current for the current goal
     */
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
                ROS_DEBUG_NAMED(ROS_NAME_SMC, "The server received a preempt request, canceling clients.");
                cancelAllClients();

                // Call custom preempt callback
                if(server_preempt_cb_ != NULL)
                    server_preempt_cb_(this);

                // When all clients are canceled set server as preempted
                ROS_DEBUG_NAMED(ROS_NAME_SMC, "Setting server as being preempted.");
                server_->setPreempted(Result(), "Preempted by SMC");

                ROS_DEBUG_NAMED(ROS_NAME_SMC, "Server '%s' preempted.", server_name_.c_str());
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
    ros::NodeHandle                                     n_;
    
    actionlib::SimpleActionServer<ServerActionType>*    server_;
    std::string                                         server_name_;
    bool                                                server_started_;
    ServerWorkCallback                                  server_work_cb_;
    ServerPreemptCallback                               server_preempt_cb_;

    thread_safe::map<std::string, ComplexClientBase*>   clients_;   
    std::string                                         latest_client_;

    std::mutex                                          set_state_mutex_;

    GoalConstPtr                                        current_goal_;
    GoalConstPtr                                        last_goal_;
    std::mutex                                          goal_mutex;
};

#endif // SERVER_MULTIPLE_CLIENT_HPP
