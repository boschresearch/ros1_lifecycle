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

#include "lifecycle/manager.h"

class ExampleNode {
public:
    ExampleNode() : _nh("~"), _lcm(_nh) {
        _lcm.setTransitionCallback(ros::lifecycle::CONFIGURE, boost::bind(&ExampleNode::configure, this));
        _lcm.setTransitionCallback(ros::lifecycle::CONFIGURE, boost::bind(&ExampleNode::cleanup, this));
        _lcm.setTransitionCallback(ros::lifecycle::ACTIVATE, boost::bind(&ExampleNode::activate, this));
        _lcm.setTransitionCallback(ros::lifecycle::DEACTIVATE, boost::bind(&ExampleNode::deactivate, this));
        _lcm.setTransitionCallback(ros::lifecycle::ACTIVATE, boost::bind(&ExampleNode::shutdown, this));
        _lcm.setErrorCb(boost::bind(&ExampleNode::errorprocessing, this, _1));
        
        //start the action server
        _lcm.start();
    }

protected:
    bool configure() { return true; };
    bool cleanup() { return true; };
    bool activate() { return true; };
    bool deactivate() { return true; };
    bool shutdown() { return true; };
    bool errorprocessing(std::exception ex) { return true; };

private:
    ros::NodeHandle _nh;
    ros::lifecycle::LifecycleManager _lcm;
};

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "example_lcm_node");
    
    ExampleNode node;
    
    ros::spin();
}