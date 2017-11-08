#!/usr/bin/env python

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

PKG = 'lm_monitor'
NODE_NAME = 'TestLmMonitor'
LM_MONITOR_EVENT_TOPIC = "/lm_events"

import sys
import unittest
import threading

import rospy

from lifecycle_msgs.msg import Lifecycle
from lifecycle_msgs.msg import lm_events
from lm_monitor.listener import LmEventListener, ListenerCbException

def ROSINIT ():
    rospy.init_node(NODE_NAME)
    
class SpinThread(threading.Thread):
    import rospy
    def __init__(self, threadID, name, counter):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        self.counter = counter
    def run(self):
        rospy.spin()

## A python unit test class for Listener
class TestListener(unittest.TestCase):
    Count = 0 
    CB = 5
    
    def CB1(self):
        pass
        
    def listener_cb(self, buffer):
        self.assertEquals(buffer[NODE_NAME]["transition"], "CONFIGURE")
        self.assertEquals(buffer[NODE_NAME]["end_state"], "INACTIVE")
        self.assertEquals(buffer[NODE_NAME]["result_code"], "SUCCESS")
        
        self.Count = self.Count + 1
        
    ## test 1
    def test_listener(self):
        ROSINIT()
        pub_lm_monitor = rospy.Publisher("/lm_events", lm_events, queue_size=1000)
        
        listener = LmEventListener(self.listener_cb)
        
        rospy.sleep(1.0)
        
        lm_monitor_msg = lm_events()
        lifecycle_msg = Lifecycle()
        
        lifecycle_msg.transition  = Lifecycle.EV_CONFIGURE
        lifecycle_msg.end_state   = Lifecycle.PSTATE_INACTIVE
        lifecycle_msg.result_code = Lifecycle.EV_SUCCESS
        
        lm_monitor_msg.node_name        = NODE_NAME
        lm_monitor_msg.lifecycle_event  = lifecycle_msg
        pub_lm_monitor.publish(lm_monitor_msg)
        
        #wait for callback to be processed
        rospy.sleep(2.0)
        #Check that the call back function has been called once
        self.assertEquals(1, self.Count);
        rospy.sleep(2.0)
        
    ## Test 2
    def test_simulate_failure_on_listener_callback_args(self):
        ROSINIT()
        
        with self.assertRaises(ListenerCbException):
            listener = LmEventListener(self.CB)
        
        with self.assertRaises(ListenerCbException):
            listener = LmEventListener(self.CB1)
        
if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'Test_Lifecycle_Manager', TestListener)
