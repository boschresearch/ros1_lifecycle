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

#ifndef LIFECYCLE_MANAGED_NODE_H
#define LIFECYCLE_MANAGED_NODE_H

#include <ros/ros.h>

#include "manager.h"

namespace ros { namespace lifecycle {
        /**
         * Class which implements all the callbacks that are used by the Lifecycle as empty, and registers
         * them with a manager.
         */
        class ManagedNode {
        public:
            ManagedNode(const ros::NodeHandle& nh);
            virtual ~ManagedNode(){};
        protected:
            /** Empty transition callbacks, default to return true (-> SUCCESS) */
            virtual bool onConfigure() { return true; };
            virtual bool onCleanup() { return false; };
            //A node must not start directly after process creation when the life-cycle is in use
            //Hence making onActivate pure virtual so that a user implementation is enforced.
            virtual bool onActivate() = 0;
            virtual bool onDeactivate() { return false; };
            virtual bool onShutdown() { return true; };
            /** Error recovery callback. Defaults to return false. */
            virtual bool onError(const std::exception&) { return false; };

            State getCurrentState() { return lm_.getCurrentState(); }
            bool raiseError(const std::exception& ex) {
            	return lm_.raiseError(ex);
            }

            ros::NodeHandle nh_;
        private:
            LifecycleManager lm_;
        };
    }}

#endif //LIFECYCLE_MANAGED_NODE_H
