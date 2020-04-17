//
// ROS1 Lifecycle - A library implementing the ROS2 lifecycle for ROS1
//
// Copyright 2016,2017 Robert Bosch GmbH
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//   http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//

#include "lifecycle/manager.h"
#include "lifecycle/lifecycle_model.h"
#include "lifecycle/broadcaster.h"
#include <boost/format.hpp>
#include <ros/callback_queue_interface.h>

namespace ros { namespace lifecycle {

using std::make_pair;
LifecycleManager::LifecycleManager(const ros::NodeHandle& nh) :
        nh_(nh),
        as_(nh, LIFECYCLE_ACTION_NAME, false),
        current_(UNCONFIGURED),
        lm_broadcaster_(nh){
    primary_steps_[make_pair(UNCONFIGURED, CONFIGURE)]      = Configuring;
    primary_steps_[make_pair(UNCONFIGURED, SHUTDOWN)]       = ShuttingDown;
    
    primary_steps_[make_pair(INACTIVE, CLEANUP)]            = CleaningUp;
    primary_steps_[make_pair(INACTIVE, ACTIVATE)]           = Activating;
    primary_steps_[make_pair(INACTIVE, SHUTDOWN)]           = ShuttingDown;
    primary_steps_[make_pair(INACTIVE, ERROR)]              = ErrorProcessing;
        
    primary_steps_[make_pair(ACTIVE, SHUTDOWN)]             = ShuttingDown;
    primary_steps_[make_pair(ACTIVE, DEACTIVATE)]           = Deactivating;
    primary_steps_[make_pair(ACTIVE, ERROR)]                = ErrorProcessing;
        
    secondary_steps_[make_pair(Configuring, SUCCESS)]       = INACTIVE;
    secondary_steps_[make_pair(Configuring, FAILURE)]       = UNCONFIGURED;
    
    secondary_steps_[make_pair(CleaningUp, SUCCESS)]        = UNCONFIGURED; // must not fail
    
    secondary_steps_[make_pair(Activating, SUCCESS)]        = ACTIVE;
    secondary_steps_[make_pair(Activating, FAILURE)]        = INACTIVE;
    
    secondary_steps_[make_pair(ShuttingDown, SUCCESS)]      = FINALIZED; // must not fail

    secondary_steps_[make_pair(Deactivating, SUCCESS)]      = INACTIVE; // must not fail

    secondary_steps_[make_pair(ErrorProcessing, SUCCESS)]   = UNCONFIGURED;
    secondary_steps_[make_pair(ErrorProcessing, FAILURE)]   = FINALIZED;
    
    setTransitionCallback(ERROR, boost::bind(&LifecycleManager::activeEx_cb, this) );

    state_pub_ = nh_.advertise<lifecycle_msgs::Lifecycle>(LIFECYCLE_STATE_TOPIC, true);
    as_.registerGoalCallback(boost::bind(&LifecycleManager::goalCb, this));
}

LifecycleManager::~LifecycleManager() {
    
}

void LifecycleManager::publishTransition(const Transition& transition, const ResultCode& result_code) {
    lifecycle_msgs::Lifecycle msg;
    msg.transition = transition;
    msg.end_state = current_;
    msg.result_code = result_code;
    
    state_pub_.publish(msg);
    lm_broadcaster_.send_lm_event(msg);
}

namespace {
using namespace ros;

	class ActivationCallback : public CallbackInterface {
	public:
		ActivationCallback(LifecycleManager* lcm) : lcm(lcm) {};

		CallbackInterface::CallResult call() {
			try {
				if(lcm->configure()) {
					lcm->activate();
					return Success;
				} else {
					ROS_WARN_STREAM("Configuring " << ros::this_node::getName()
					<< " failed, won't activate");
				}
			} catch(const std::exception& ex) {
				ROS_ERROR_STREAM("Error when enabling "
						<< ros::this_node::getName() << ": " << ex.what());
			} catch(...) {
				ROS_ERROR_STREAM("Unknown error when enabling "
										<< ros::this_node::getName());
			}
			return Invalid;
		}
	private:
		LifecycleManager* lcm;
	};
}

void LifecycleManager::start() {
    as_.start();
    bool lifecycle_enabled = false;
    nh_.param(PARAM_LIFECYCLE_MANAGEMENT, lifecycle_enabled, false);
    if(!lifecycle_enabled) {
    	// schedule a callback that will activate this node in the absence of
    	// a manager
    	nh_.getCallbackQueue()->addCallback(boost::make_shared<ActivationCallback>(this));
    }
}

void LifecycleManager::setTransitionCallback(Transition tr, transitionCb cb) {
    switch(tr) {
        case CONFIGURE:
            callbacks_[make_pair(UNCONFIGURED, tr)] = cb;
            break;
        case ACTIVATE:
            callbacks_[make_pair(INACTIVE, tr)] = cb;
            break;
        case DEACTIVATE:
            callbacks_[make_pair(ACTIVE, tr)] = cb;
            break;
        case SHUTDOWN:
            callbacks_[make_pair(ACTIVE, tr)] = cb;
            callbacks_[make_pair(INACTIVE, tr)] = cb;
            callbacks_[make_pair(UNCONFIGURED, tr)] = cb;
            break;
        case CLEANUP:
            callbacks_[make_pair(INACTIVE, tr)] = cb;
            break;
        case ERROR:
            callbacks_[make_pair(ACTIVE, tr)] = cb;
            callbacks_[make_pair(INACTIVE, tr)] = cb;
            break;
        default:
            assert(false);
    }
}

void LifecycleManager::goalCb() {
    lifecycle_msgs::LifecycleGoalConstPtr goal = as_.acceptNewGoal();
    lifecycle_msgs::LifecycleResult result;
    try {
        if (handleTransition(static_cast<Transition>(goal->transition))) {
            result.end_state = getCurrentState();
            as_.setSucceeded(result, "goal state reached okay");
        } else {
            result.end_state = getCurrentState();
            as_.setAborted(result, "transition failed");
        }
    } catch(const IllegalTransitionException& ite) {
        result.end_state = getCurrentState();
        as_.setAborted(result, "requested transition is not a valid lifecycle transition");
    }
}

bool LifecycleManager::handleTransition(const Transition& transition) {
    PrimaryInput p_input = make_pair(current_, transition);
    bool result;
    result = handlePrimaryStep(p_input);
    try {
        if(result) {
            handleSecondaryStep(make_pair(current_, SUCCESS));
        } else {
            handleSecondaryStep(make_pair(current_, FAILURE));
        }
    } catch (const std::exception& ex) {
        if(handleErrorProcessing(ex)) {
            handleSecondaryStep(make_pair(current_, SUCCESS));
        } else {
            handleSecondaryStep(make_pair(current_, FAILURE));
        }
    }
    
    if(result) {
        publishTransition(transition, SUCCESS);
    } else {
        publishTransition(transition, FAILURE);
    }
    
    return result;
}

bool LifecycleManager::handlePrimaryStep(const PrimaryInput& input) {
    PrimaryStepMap::const_iterator primary_steps_iter = primary_steps_.find(input);
    CallbackMap::iterator cb_iter = callbacks_.find(input);
    if(primary_steps_iter != primary_steps_.end()) {
        current_ = primary_steps_iter->second;
        if(cb_iter != callbacks_.end()) {
            try {
                return cb_iter->second();
            } catch(const std::exception& ex) {
                return handleErrorProcessing(ex);
            }
        } else {
            return true;
        }
    } else {
        throw IllegalTransitionException(input);
    }
}
  
bool LifecycleManager::handleSecondaryStep(const SecondaryInput& input) {
    SecondaryStepMap::const_iterator secondary_steps_iter = secondary_steps_.find(input);
    if(secondary_steps_iter != secondary_steps_.end()) {
        current_ = secondary_steps_iter->second;
        return true;
    } else {
        throw IllegalTransitionException(input);
    }
}

bool LifecycleManager::handleErrorProcessing(const std::exception& ex) {
    current_ = ErrorProcessing;
    try {
        return onError_(ex);
    } catch(const std::exception& ex) {
        return false;
    }
}

void LifecycleManager::setErrorCb(errorCb cb) {
    onError_ = cb;
}

bool LifecycleManager::raiseError(const std::exception& ex) {
    if(LifecycleModel::isPrimaryState(current_)) {
        activeEx_ = ex;
        return handleTransition(ERROR);
    } else {
        return handleErrorProcessing(ex);
    }
}

bool LifecycleManager::configure() {
    return handleTransition(CONFIGURE);
}

bool LifecycleManager::activate() {
    return handleTransition(ACTIVATE);
}

bool LifecycleManager::deactivate() {
    return handleTransition(DEACTIVATE);
}

bool LifecycleManager::shutdown() {
    return handleTransition(SHUTDOWN);
}

bool LifecycleManager::cleanup() {
    return handleTransition(CLEANUP);
}

IllegalTransitionException::IllegalTransitionException(const PrimaryInput& input) {

    message = str(boost::format("IllegalTransitionException state=%1%, transition=%2%") %
                      input.first % input.second);
};
IllegalTransitionException::IllegalTransitionException(const SecondaryInput& input) {

    message = str(boost::format("IllegalTransitionException state=%1%, transition=%2%") %
                      input.first % input.second);
};
IllegalTransitionException::~IllegalTransitionException() throw() { }
const char* IllegalTransitionException::what() const throw() {
    return message.c_str();
}

}}
