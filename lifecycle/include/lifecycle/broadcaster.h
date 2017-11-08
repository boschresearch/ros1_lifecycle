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

#ifndef BROADCASTER_H
#define BROADCASTER_H

#include <ros/ros.h>

#include <lifecycle_msgs/lm_events.h>

#define LM_MONITOR_EVENT_TOPIC "/lm_events"

namespace ros { namespace lifecycle {

        class LmEventBroadcaster {
        public:
            /******************************************************************
            *`LmEventBroadcaster` is a convenient way to send Lifecycle 
            *event updates on the ``"/lm_events"`` message topic.
            ******************************************************************/
            LmEventBroadcaster(const ros::NodeHandle& nh);
            
            void send_lm_event(const lifecycle_msgs::Lifecycle msg);
            
        private:
            ros::NodeHandle nh_;
            std::string node_name;
            ros::Publisher pub_lm_monitor;
        };
}}

#endif //BROADCASTER_H
