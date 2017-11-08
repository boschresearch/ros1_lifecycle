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

#ifndef LIFECYCLE_MODEL_H
#define LIFECYCLE_MODEL_H

#include <map>

#include <boost/function.hpp>
#include <boost/bind.hpp>

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>

#include <lifecycle_msgs/LifecycleAction.h>
#include <lifecycle_msgs/Lifecycle.h>

#define LIFECYCLE_ACTION_NAME "lifecycle"
#define LIFECYCLE_STATE_TOPIC "lifecycle_state"

namespace ros { namespace lifecycle {    
        
        enum State {
            // Primary States
            UNCONFIGURED    = lifecycle_msgs::LifecycleGoal::PSTATE_UNCONFIGURED,
            INACTIVE        = lifecycle_msgs::LifecycleGoal::PSTATE_INACTIVE,
            ACTIVE          = lifecycle_msgs::LifecycleGoal::PSTATE_ACTIVE,
            FINALIZED       = lifecycle_msgs::LifecycleGoal::PSTATE_FINALIZED,
            //Secondary States
            ErrorProcessing = lifecycle_msgs::LifecycleGoal::TSTATE_ERROR_PROCESSING,
            CleaningUp      = lifecycle_msgs::LifecycleGoal::TSTATE_CLEANING_UP,
            Configuring     = lifecycle_msgs::LifecycleGoal::TSTATE_CONFIGURING,
            Activating      = lifecycle_msgs::LifecycleGoal::TSTATE_ACTIVATING,
            Deactivating    = lifecycle_msgs::LifecycleGoal::TSTATE_DEACTIVATING,
            ShuttingDown    = lifecycle_msgs::LifecycleGoal::TSTATE_SHUTTING_DOWN
        };
        enum Transition {
            CONFIGURE   = lifecycle_msgs::LifecycleGoal::EV_CONFIGURE,
            CLEANUP     = lifecycle_msgs::LifecycleGoal::EV_CLEANUP,
            ACTIVATE    = lifecycle_msgs::LifecycleGoal::EV_ACTIVATE,
            DEACTIVATE  = lifecycle_msgs::LifecycleGoal::EV_DEACTIVATE,
            SHUTDOWN    = lifecycle_msgs::LifecycleGoal::EV_SHUTDOWN,
            ERROR       = lifecycle_msgs::LifecycleGoal::EV_ERROR
        };
        enum ResultCode {
            SUCCESS = lifecycle_msgs::LifecycleGoal::EV_SUCCESS,
            FAILURE = lifecycle_msgs::LifecycleGoal::EV_FAILURE
        };
        
        typedef boost::function<bool (void)> transitionCb;
        typedef boost::function<bool (const std::exception&)> errorCb;
        typedef boost::function<void (bool)> completionCb;
        typedef std::pair<State, Transition> PrimaryInput;
        typedef std::pair<State, ResultCode> SecondaryInput;
        typedef std::pair<State, State> Eventsinput;
        typedef std::list<Transition> TransitionList;
        typedef std::map<PrimaryInput, State> PrimaryStepMap;
        typedef std::map<SecondaryInput, State> SecondaryStepMap;
        typedef std::map<PrimaryInput, transitionCb> CallbackMap;
        typedef std::map<Eventsinput, TransitionList> Events;
        typedef ros::Subscriber LifecycleSubscriber;
        typedef actionlib::SimpleActionClient<lifecycle_msgs::LifecycleAction> LifecycleActionClient;
        
        class LifecycleModel {
        public:
            /****************************************************************
            * Description: Returns True if the state is a Primary State, i.e.
            * UNCONFIGURED or INACTIVE or ACTIVE or FINALIZED
            ****************************************************************/
            static bool isPrimaryState(State state);
            
        };
    }}

#endif //LIFECYCLE_MODEL_H
