#! /usr/bin/env python

from __future__ import print_function

import sys
import rospy
import actionlib
from actionlib.action_client import CommState
from lifecycle_msgs.msg import LifecycleGoal, LifecycleAction

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
