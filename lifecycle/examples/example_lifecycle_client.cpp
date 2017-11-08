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

#include <ros/ros.h>

#include <boost/thread.hpp>

#include "lifecycle/client.h"
#include <lifecycle/lifecycle_model.h>

#define NODE_NAME "lm_node_name"

using namespace ros;
using namespace ros::lifecycle;
using namespace lifecycle_msgs;

std::string TRANSITION;

void cb_func(bool result){
    if(result){
        ROS_INFO("Transition %s Sucessful",TRANSITION.c_str());
    }else {
        ROS_INFO("Transition %s UnSucessful",TRANSITION.c_str());
    }
}

void spinThread()
{
  ros::spin();
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "example_client_node");
    ros::NodeHandle nh("~");
    std::string node_name = "";
    nh.param(NODE_NAME, node_name, node_name);
    
    boost::thread spin_thread(&spinThread);
    
    LifecycleClient lm_client(nh, node_name);
    
    try {
        TRANSITION = "UNCONFIGURED";
        lm_client.goToState(UNCONFIGURED, cb_func);
        ros::Duration(2.0).sleep();
        
        TRANSITION = "INACTIVE";
        lm_client.goToState(INACTIVE, cb_func);
        ros::Duration(2.0).sleep();
        
        TRANSITION = "ACTIVE";
        lm_client.goToState(ACTIVE, cb_func);
        ros::Duration(2.0).sleep();
        
        TRANSITION = "UNCONFIGURED";
        lm_client.goToState(UNCONFIGURED, cb_func);
        ros::Duration(2.0).sleep();
        
        TRANSITION = "FINALIZED";
        lm_client.goToState(FINALIZED, cb_func);
        ros::Duration(2.0).sleep();
    } catch(const LifecycleAPIException ex){
        ROS_INFO("The node: %s is not launched!", node_name.c_str());
    }
    // shutdown the node and join the thread back before exiting
    ros::shutdown();
    spin_thread.join();
}