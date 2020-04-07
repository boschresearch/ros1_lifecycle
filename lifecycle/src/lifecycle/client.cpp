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

#include <string>
#include <list>

#include <boost/format.hpp>
#include <boost/function.hpp>
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>

#include <ros/ros.h>

#include "lifecycle/client.h"
#include <lifecycle/lifecycle_model.h>

namespace ros { namespace lifecycle {

using std::make_pair;
LifecycleAPIException::LifecycleAPIException(const char* msg) {

    message = str(boost::format(msg));
};

LifecycleAPIException::~LifecycleAPIException() throw() { }

const char* LifecycleAPIException::what() const throw() {
    return message.c_str();
}

LifecycleTransitionSequence::LifecycleTransitionSequence(boost::shared_ptr<LifecycleActionClient> client, 
        TransitionList transitions, completionCb completion_cb){
    client_         = client;
    transitions_    = transitions;
    sequencer_busy_ = false;
    cancelled_      = false;
    completion_cb_  = completion_cb;
    
}

void LifecycleTransitionSequence::go(void){
    step_();
}

void LifecycleTransitionSequence::cancel(void){
    cancelled_ = true;
    if (sequencer_busy_){
        client_->cancelGoal();
        sequencer_busy_ = false;
    }
}

bool LifecycleTransitionSequence::is_done(void){
    return (cancelled_ || (transitions_.empty()));
}

actionlib::SimpleClientGoalState::StateEnum LifecycleTransitionSequence::get_result(void){
    return (result_);
}

bool LifecycleTransitionSequence::has_succeded(void){
    return (!cancelled_ && (result_ == actionlib::SimpleClientGoalState::SUCCEEDED));
}

void LifecycleTransitionSequence::step_(void){
    if (cancelled_ || (transitions_.size() == 0)){
        completion_cb_(has_succeded());
    }else {
        lifecycle_msgs::LifecycleGoal goal;
        goal.transition = transitions_.front();
        sequencer_busy_ = true;
        client_->sendGoal(goal,boost::bind(&LifecycleTransitionSequence::transitionCb_, this, _1, _2));
    }
}

void LifecycleTransitionSequence::transitionCb_(const actionlib::SimpleClientGoalState& state, 
        const lifecycle_msgs::LifecycleResultConstPtr& result){
    //if we completed the current transition, invoke next
    //TODO Check for errors
    if (state.isDone()){
        result_ = state.state_;
        //on success, we remove the current event only
        if (has_succeded()){
            transitions_.pop_front();
        }// on failure, we remove everything
        else if (result_ == actionlib::SimpleClientGoalState::ABORTED){
            transitions_.clear();
        }else{
            //on other conditions (like RECALLED, REJECTED, PREEMPTED or LOST) we try again with the same transition
        }
        
        step_();
    }
}


LifecycleClient::LifecycleClient(ros::NodeHandle& nh, std::string node_name):nh_(nh), node_name_(node_name){
    events_[make_pair(UNCONFIGURED, INACTIVE)].push_back(CONFIGURE);
    events_[make_pair(UNCONFIGURED, ACTIVE)].push_back(CONFIGURE);
    events_[make_pair(UNCONFIGURED, ACTIVE)].push_back(ACTIVATE);
    events_[make_pair(UNCONFIGURED, FINALIZED)].push_back(SHUTDOWN);
    events_[make_pair(INACTIVE, UNCONFIGURED)].push_back(CLEANUP);
    events_[make_pair(INACTIVE, ACTIVE)].push_back(ACTIVATE);
    events_[make_pair(INACTIVE, FINALIZED)].push_back(SHUTDOWN);
    events_[make_pair(ACTIVE, INACTIVE)].push_back(DEACTIVATE);
    events_[make_pair(ACTIVE, UNCONFIGURED)].push_back(DEACTIVATE);
    events_[make_pair(ACTIVE, UNCONFIGURED)].push_back(CLEANUP);
    events_[make_pair(ACTIVE, FINALIZED)].push_back(SHUTDOWN);
    
    action_client_  = boost::make_shared<LifecycleActionClient>(node_name + "/" + LIFECYCLE_ACTION_NAME);
    server_state_   = UNCONFIGURED;
    handle_.reset();
    
    state_sub_ = nh_.subscribe("/" + node_name + "/" + LIFECYCLE_STATE_TOPIC, 10, &LifecycleClient::stateCb, this);
}

void LifecycleClient::goToState(State target_state, completionCb completion_cb){
    completion_cb_ = completion_cb;
    
    //check if the node's actionlib server is up, else raise an exception after timeout
    if(!action_client_->waitForServer(ros::Duration(6.0))){
        std::string message = node_name_ + " Node Not Found";
        throw LifecycleAPIException(message.c_str());
    }
    
    // if we're already in the target state, do nothing
    if (target_state == server_state_){
        completion_cb(true);
        return;
    }
    
    // cancel current sequence if there is one, i.e the object was not reset()
    if (handle_){
        cancel();
    }
    
    // start the sequence of events/transitions to get to the target state
    TransitionList events = events_[make_pair(server_state_, target_state)];
    if (!events.empty()){
        handle_ = boost::make_shared<LifecycleTransitionSequence>(action_client_, events, 
            boost::bind(&LifecycleClient::transition_completion_cb_, this, _1));
        handle_->go();
    }
}

void LifecycleClient::cancel(){
    if(handle_){ //if the current sequence is present. i.e. the object was not reset()
        handle_->cancel();
        handle_.reset();
    }
}
void LifecycleClient::stateCb(const lifecycle_msgs::Lifecycle& msg){
    server_state_ = static_cast<State>(msg.end_state);
}

void LifecycleClient::transition_completion_cb_(bool result){
    handle_.reset(); 
    completion_cb_(result);
}

}}
