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

#include "lifecycle/managed_node.h"

class ExampleManagedNode : public ros::lifecycle::ManagedNode {
public:
    //The nodehandle has to be private namespace, for all the lifecycle related toics to be unabigous.
    ExampleManagedNode(ros::NodeHandle& nh) : _nh(nh), ros::lifecycle::ManagedNode(nh) {
    }

protected:
    // override must-have functions from managed node
    bool onActivate() { ROS_INFO("Activating"); return true; };
    
    // define other virtual methods
    bool onConfigure() { ROS_INFO("Configuring"); return true; };
    bool onDeactivate() { ROS_INFO("Deactivating"); return true; };
    bool onShutdown() { ROS_INFO("Shutting down"); return true; };
    bool onCleanup() { ROS_INFO("Cleaning up"); return true; };

private:
    ros::NodeHandle _nh;
};

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "example_lcm_node");
    ros::NodeHandle nh("~");
    nh.setParam(PARAM_LIFECYCLE_MANAGEMENT, true);

    ExampleManagedNode node(nh);
    
    ros::spin();
}