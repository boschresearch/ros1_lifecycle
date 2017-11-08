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

#include <ros/ros.h>

#include "lifecycle/broadcaster.h"
#include <lifecycle_msgs/lm_events.h>

namespace ros { namespace lifecycle {

LmEventBroadcaster::LmEventBroadcaster(const ros::NodeHandle& nh) :
        nh_(nh){
    node_name = ros::this_node::getName();
    //Remove the '/' character from the namespace
    node_name.erase(std::remove(node_name.begin(), node_name.begin()+1, '/'), node_name.begin()+1);
    pub_lm_monitor = nh_.advertise<lifecycle_msgs::lm_events>(LM_MONITOR_EVENT_TOPIC, 1000);
}

void LmEventBroadcaster::send_lm_event(const lifecycle_msgs::Lifecycle msg){
    lifecycle_msgs::lm_events lm_monitor_msg;
    
    lm_monitor_msg.node_name = node_name;
    lm_monitor_msg.lifecycle_event = msg;
    
    pub_lm_monitor.publish(lm_monitor_msg);
}

}}
