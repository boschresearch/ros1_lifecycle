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

#ifndef LIFECYCLE_LifecycleManager_H
#define LIFECYCLE_LifecycleManager_H

#include <map>

#include <boost/function.hpp>
#include <boost/bind.hpp>

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>

#include "lifecycle/lifecycle_model.h"
#include <lifecycle_msgs/LifecycleAction.h>
#include <lifecycle_msgs/Lifecycle.h>
#include <lifecycle/broadcaster.h>

#define PARAM_LIFECYCLE_MANAGEMENT "/lifecycle_enabled"

namespace ros { namespace lifecycle {

        class IllegalTransitionException : public std::exception {
        public:
            IllegalTransitionException(const PrimaryInput& input);
            IllegalTransitionException(const SecondaryInput& input);
            virtual ~IllegalTransitionException() throw();
            virtual const char* what() const throw();
        private:
            std::string message;
        };

        class LifecycleManager {
        public:
            /******************************************************************
            *Description: Constructor for the LifecycleManager class. 
            *Initialises the values of primary_steps_ and secondary_steps_
            *with valid transitions. Defines a action server and a publisher.
            ******************************************************************/
            LifecycleManager(const ros::NodeHandle& nh);
            
            /******************************************************************
            *Description: Constructor for the LifecycleManager class. 
            *Initialises the values of primary_steps_ and secondary_steps_
            *with valid transitions. Defines a action server and a publisher.
            ******************************************************************/
            virtual ~LifecycleManager();

            /*************************************************************
            * Description: Starts the action server
            *************************************************************/
            void start();

            /*************************************************************
            * Description: sets the callback functions of the respective 
            * transition
            *************************************************************/
            void setTransitionCallback(Transition, transitionCb);
            
            /*************************************************************
            * Description: A error callback function for the Lifecycle 
            * manager node which handles the errors 
            *************************************************************/
            void setErrorCb(errorCb);
            
            /*************************************************************
            * Description: An api to raise error so that lifecycle managed
            * node transitions to the errorProcessing state
            *************************************************************/
            bool raiseError(const std::exception&);
            
            /*************************************************************
            * Description: "Configure" Transition is initiated
            *************************************************************/
            bool configure();
            
            /*************************************************************
            * Description: "Activate" Transition is initiated
            *************************************************************/
            bool activate();
            
            /*************************************************************
            * Description: "Deactivate" Transition is initiated
            *************************************************************/
            bool deactivate();
            
            /*************************************************************
            * Description: "Shutdown" Transition is initiated
            *************************************************************/
            bool shutdown();
            
            /*************************************************************
            * Description: "Cleanup" Transition is initiated
            *************************************************************/
            bool cleanup();

            /*************************************************************
            * Description: Gets the current state of the LM node
            *************************************************************/
            State getCurrentState() { return current_; };
            
        protected:

            /*************************************************************
            * Description: A goal callback function for the action server
            * updates the result of the action
            *************************************************************/
            void goalCb();
            
            /*************************************************************
            * Description: Handles the transition of node from one primary
            * state to another primary state
            *************************************************************/
            bool handleTransition(const Transition& transition);
            
            /*************************************************************
            * Description: Publishes the Lifecycle events. Whenever the 
            * state of the node changes, this method is called to publish 
            * the transition that triggered the change, current state and 
            * the result code of the transition
            *************************************************************/
            void publishTransition(const Transition& transition, const ResultCode& result_code);

            errorCb onError_;

            ros::NodeHandle nh_;
            typedef actionlib::SimpleActionServer<lifecycle_msgs::LifecycleAction> LifecycleActionServer;
            LifecycleActionServer as_;

            typedef ros::Publisher LifecyclePublisher;
            LifecyclePublisher state_pub_;
            LmEventBroadcaster lm_broadcaster_;

        private:
            PrimaryStepMap primary_steps_;
            SecondaryStepMap secondary_steps_;
            CallbackMap callbacks_;
            State current_;
            std::exception activeEx_;
            
            /*************************************************************
            * Description: handles the first step of the transition i.e.
            * from a primary state to a transitional state
            *************************************************************/
            bool handlePrimaryStep(const PrimaryInput& input);
            
            /*************************************************************
            * Description: Handles the second step of the transition i.e.
            * from a transitional state to a primary state.
            *************************************************************/   
            bool handleSecondaryStep(const SecondaryInput& input);
            
            /****************************************************************
            * Description: Handles the ErrorProcessing step in a transition 
            ****************************************************************/
            bool handleErrorProcessing(const std::exception& ex);
            
            
            bool activeEx_cb(void) { throw activeEx_; return true; };
        };
}}

#endif //LIFECYCLE_LifecycleManager_H
