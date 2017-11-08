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

#include "lifecycle/managed_node.h"

namespace ros { namespace lifecycle {
        ManagedNode::ManagedNode(const ros::NodeHandle& nh) : nh_(nh), lm_(nh) {
            lm_.setTransitionCallback(CONFIGURE, boost::bind(&ManagedNode::onConfigure, this));
            lm_.setTransitionCallback(ACTIVATE, boost::bind(&ManagedNode::onActivate, this));
            lm_.setTransitionCallback(DEACTIVATE, boost::bind(&ManagedNode::onDeactivate, this));
            lm_.setTransitionCallback(SHUTDOWN, boost::bind(&ManagedNode::onShutdown, this));
            lm_.setTransitionCallback(CLEANUP, boost::bind(&ManagedNode::onCleanup, this));
            lm_.setErrorCb(boost::bind(&ManagedNode::onError, this, _1));
            //start the action server
            lm_.start();
        }
    }}