#! /usr/bin/env python

# ROS1 Lifecycle - A library implementing the ROS2 lifecycle for ROS1
#
# Copyright 2016,2017 Robert Bosch GmbH
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#   http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


from __future__ import print_function

import rospy
import actionlib
from actionlib.action_client import CommState
from lifecycle_msgs.msg import LifecycleGoal, LifecycleAction
import sys

if __name__ == '__main__':
    if len(sys.argv) < 3:
        print("Syntax: change_lifecycle.py topic transition")
        sys.exit(-1)

    topic = sys.argv[1]
    transition = int(sys.argv[2])

    rospy.init_node("change_lifecycle", anonymous=True)

    client = actionlib.action_client.ActionClient(sys.argv[1], LifecycleAction)
    client.wait_for_server(timeout=rospy.Duration(1.0))

    goal = LifecycleGoal()
    goal.transition = transition

    handle = client.send_goal(goal, lambda msg: print("Transition: ", msg.get_goal_status(),
                                                      msg.get_goal_status_text(),
                                                      CommState.to_string(msg.get_comm_state()), msg.get_result()))
