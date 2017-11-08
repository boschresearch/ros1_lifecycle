/* * ROS1 Lifecycle - A library implementing the ROS2 lifecycle for ROS1
 *
 * Copyright 2016,2017 Robert Bosch GmbH
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef LIFECYCLE_Client_H
#define LIFECYCLE_Client_H

#include <list>

#include <boost/shared_ptr.hpp>

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>

#include <lifecycle/lifecycle_model.h>
#include <lifecycle_msgs/LifecycleAction.h>
#include <lifecycle_msgs/Lifecycle.h>

namespace ros { namespace lifecycle {

        class LifecycleAPIException : public std::exception {
        public:
            LifecycleAPIException(const char* msg);
            virtual ~LifecycleAPIException() throw();
            virtual const char* what() const throw();
            
        private:
            std::string message;
        };
        
        class LifecycleTransitionSequence {
        public:
            /****************************************************************
            * Description: Constructor. Initialises the transitions to undergo
            * to reach target_state
            ****************************************************************/
            LifecycleTransitionSequence(boost::shared_ptr<LifecycleActionClient> client, TransitionList transitions, completionCb completion_cb);
            
            /****************************************************************
            * Description: Starts the transitions 
            ****************************************************************/
            void go(void);
            
            /****************************************************************
            * Description: Cancels the transitions
            ****************************************************************/
            void cancel(void);
            
            /****************************************************************
            * Description: Check if the all transitions are complete. Returns
            * true if all the transactions are complete or the transactions 
            * are cancelled else false is returned.
            ****************************************************************/
            bool is_done(void);
            
            /****************************************************************
            * Description: Returns the latest result from the server
            ****************************************************************/
            actionlib::SimpleClientGoalState::StateEnum get_result(void);
            
        protected:
            bool cancelled_;
            bool sequencer_busy_;
            completionCb completion_cb_;
            TransitionList transitions_;
            boost::shared_ptr<LifecycleActionClient> client_;
            actionlib::SimpleClientGoalState::StateEnum result_;
            
            /****************************************************************
            * Description: Non blocking call. Returns true if the latest 
            * goal was completed sucessfully else if the goal resulted in any
            * other condition or the goal was cancelled returns false.
            ****************************************************************/
            bool has_succeded(void);
            
        private:
            void step_(void);
            void transitionCb_(const actionlib::SimpleClientGoalState& state, 
                const lifecycle_msgs::LifecycleResultConstPtr& result);
        };
        
        class LifecycleClient {
        public:
            /****************************************************************
            * Description: Ctor. Creates a action client and 
            * subscribes to lifecycle_state topic
            ****************************************************************/
            LifecycleClient(ros::NodeHandle& nh, std::string node_name);
            
            /****************************************************************
            * Description: Sends the necessary events to go the given target state,
            * based on the current state. Invokes "completion_cb" when done. 
            * completion_cb should take a single boolean to indicate success.
            * Throws LifecycleAPIException if the server is not found.
            ****************************************************************/
            void goToState(State target_state, completionCb completion_cb);
            
            void cancel();
            
            /****************************************************************
            * Description: Stores the last server state internally
            * param type: msg: Lifecycle
            ****************************************************************/
            void stateCb(const lifecycle_msgs::Lifecycle& msg);
            
        protected:
            State server_state_;
            completionCb completion_cb_;
            LifecycleSubscriber state_sub_;
            boost::shared_ptr<LifecycleTransitionSequence> handle_;
            boost::shared_ptr<LifecycleActionClient> action_client_;
            
        private:
            Events events_;
            ros::NodeHandle nh_;
            std::string node_name_;
            void transition_completion_cb_(bool result);
        };
        
}}

#endif //LIFECYCLE_Client_H

