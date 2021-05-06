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

PKG = 'lifecycle_python'
NODE_NAME = 'Test_run'

import sys
import unittest
import threading

import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus

from lifecycle.lifecycle_model import State, Transition, Result_Code, LifecycleModel
from lifecycle.manager import LifecycleManager, IllegalTransitionException, LIFECYCLE_ACTION_NAME, LIFECYCLE_STATE_TOPIC, LifecycleCbException
from lifecycle_msgs.msg import LifecycleGoal, LifecycleAction, LifecycleResult, Lifecycle
from lifecycle.managed_node import ManagedNode


def ROSINIT ():
    rospy.init_node(NODE_NAME)

def return_false():
    return False
    
CB = 5

def CB1(arg):
    pass

ERROR_CB = 5
    
def ERROR_CB1():
    pass
    
    
class SpinThread(threading.Thread):
    import rospy
    def __init__(self, threadID, name, counter):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        self.counter = counter
    def run(self):
        rospy.spin()
        
## A python unit test class for LifecycleManager
class TestLifecycleManager(unittest.TestCase):
    ## test 1
    def test_internal(self):
        ROSINIT()
        lm = LifecycleManager(NODE_NAME)
        lm.configure()
        self.assertEquals(State.INACTIVE, lm.get_current_state())
        lm.cleanup() 
        self.assertEquals(State.UNCONFIGURED, lm.get_current_state()) 
        lm.configure() 
        lm.activate() 
        self.assertEquals(State.ACTIVE, lm.get_current_state()) 
        lm.shutdown() 
        self.assertEquals(State.FINALIZED, lm.get_current_state()) 
    
    def check_goal(self, lm, client, transition, target):
        goal = LifecycleGoal()
        
        goal.transition = transition
        client.send_goal(goal)
        self.assertTrue(client.wait_for_result(rospy.Duration.from_sec(1.0)))
        self.assertEquals(GoalStatus.SUCCEEDED, client.get_state())
        self.assertEquals(target, lm.get_current_state())
    
    def test_through_action_client(self):
        ROSINIT()
        lm = LifecycleManager(NODE_NAME)
        lm.start()
        
        #TODO: implement this test case
        
    def test_transitions_for_unconfigured(self):
        ROSINIT()
        lm = LifecycleManager(NODE_NAME)
        self.assertEquals(State.UNCONFIGURED, lm.get_current_state())

        # make sure these are not accepted
        with self.assertRaises(IllegalTransitionException):
            lm.activate()
        self.assertEquals(State.UNCONFIGURED, lm.get_current_state())
        with self.assertRaises(IllegalTransitionException):
            lm.deactivate()
        self.assertEquals(State.UNCONFIGURED, lm.get_current_state())
        with self.assertRaises(IllegalTransitionException):
            lm.cleanup()
        self.assertEquals(State.UNCONFIGURED, lm.get_current_state())
        # now make sure all the valid ones are accepted
        self.assertTrue(lm.configure())
        self.assertEquals(State.INACTIVE, lm.get_current_state())
        lm.cleanup()
        self.assertEquals(State.UNCONFIGURED, lm.get_current_state())
        self.assertTrue(lm.shutdown())
        self.assertEquals(State.FINALIZED, lm.get_current_state())

    def test_transitions_for_inactive(self):
        ROSINIT()
        lm = LifecycleManager(NODE_NAME) 
        
        lm.configure() 
        self.assertEquals(State.INACTIVE, lm.get_current_state()) 
        # make sure these are not accepted
        with self.assertRaises(IllegalTransitionException):
            lm.configure()
        self.assertEquals(State.INACTIVE, lm.get_current_state())
        with self.assertRaises(IllegalTransitionException):
            lm.deactivate()
        self.assertEquals(State.INACTIVE, lm.get_current_state())
        # now make sure all the valid ones are accepted
        self.assertTrue(lm.activate()) 
        self.assertEquals(State.ACTIVE, lm.get_current_state()) 
        
        # go back
        lm.deactivate() 
        self.assertEquals(State.INACTIVE, lm.get_current_state()) 

        self.assertTrue(lm.cleanup()) 
        self.assertEquals(State.UNCONFIGURED, lm.get_current_state()) 

        # go back
        self.assertTrue(lm.configure()) 
        self.assertEquals(State.INACTIVE, lm.get_current_state()) 

        self.assertTrue(lm.shutdown()) 
        self.assertEquals(State.FINALIZED, lm.get_current_state()) 

    def test_transitions_for_active(self):
        ROSINIT()
        lm = LifecycleManager(NODE_NAME) 
        lm.configure() 
        lm.activate() 
        self.assertEquals(State.ACTIVE, lm.get_current_state()) 

        # make sure these are not accepted
        with self.assertRaises(IllegalTransitionException):
            lm.configure()
        self.assertEquals(State.ACTIVE, lm.get_current_state()) 
        with self.assertRaises(IllegalTransitionException):
            lm.activate()
        self.assertEquals(State.ACTIVE, lm.get_current_state()) 
        with self.assertRaises(IllegalTransitionException):
            lm.cleanup()
        self.assertEquals(State.ACTIVE, lm.get_current_state()) 

        # now make sure all the valid ones are accepted
        self.assertTrue(lm.deactivate()) 
        self.assertEquals(State.INACTIVE, lm.get_current_state()) 

        # go back
        lm.activate() 
        self.assertEquals(State.ACTIVE, lm.get_current_state()) 

        self.assertTrue(lm.shutdown()) 
        self.assertEquals(State.FINALIZED, lm.get_current_state()) 

    def test_simulate_failure_during_configuring(self):
        ROSINIT()
        lm = LifecycleManager(NODE_NAME) 
        lm.set_transition_callback(Transition.CONFIGURE, return_false) 

        self.assertFalse(lm.configure()) 
        self.assertEquals(State.UNCONFIGURED, lm.get_current_state()) 

    def test_simulate_failure_during_activating(self):
        ROSINIT()
        lm = LifecycleManager(NODE_NAME) 
        lm.set_transition_callback(Transition.ACTIVATE, return_false) 
        self.assertTrue(lm.configure()) 
        self.assertEquals(State.INACTIVE, lm.get_current_state()) 
        self.assertFalse(lm.activate()) 
        self.assertEquals(State.INACTIVE, lm.get_current_state()) 
        
    def test_simulate_failure_on_set_transition_callback(self):
        ROSINIT()
        lm = LifecycleManager(NODE_NAME) 
        with self.assertRaises(LifecycleCbException):
            lm.set_transition_callback(Transition.ACTIVATE, CB)
        
        with self.assertRaises(LifecycleCbException):
            lm.set_transition_callback(Transition.ACTIVATE, CB1)
        
    def test_simulate_failure_on_set_error_cb(self):
        ROSINIT()
        lm = LifecycleManager(NODE_NAME) 
        with self.assertRaises(LifecycleCbException):
            lm.set_error_cb(ERROR_CB)
            
        with self.assertRaises(LifecycleCbException):
            lm.set_error_cb(ERROR_CB1)

    Expected_End_State  = [State.INACTIVE, State.UNCONFIGURED, State.INACTIVE, State.ACTIVE, State.FINALIZED]
    Expected_Transition    = [Transition.CONFIGURE, Transition.CLEANUP, Transition.CONFIGURE, Transition.ACTIVATE, Transition.SHUTDOWN]
    Expected_Result_Code   = [Result_Code.SUCCESS, Result_Code.SUCCESS, Result_Code.SUCCESS, Result_Code.SUCCESS, Result_Code.SUCCESS]
    Count = 0 

    def subscriber_cb(self, msg):
        if(self.Count < 5):
            self.assertEquals(self.Expected_End_State[self.Count], msg.end_state) 
            self.assertEquals(self.Expected_Transition[self.Count], msg.transition) 
            self.assertEquals(self.Expected_Result_Code[self.Count], msg.result_code) 
            self.Count = self.Count + 1

    def test_publish_transition(self):
        ROSINIT()
        lm = LifecycleManager(NODE_NAME) 
        lm.start() 
        
        state_sub = rospy.Subscriber(NODE_NAME + '/' + LIFECYCLE_STATE_TOPIC, Lifecycle, self.subscriber_cb) 
        spin_thread = SpinThread(2, "SubscriberCB_Thread", 2) 
        spin_thread.setDaemon(True)
        spin_thread.start()
        
        lm.configure() 
        self.assertEquals(State.INACTIVE, lm.get_current_state()) 
        rospy.sleep(1.0)
        lm.cleanup() 
        self.assertEquals(State.UNCONFIGURED, lm.get_current_state()) 
        rospy.sleep(1.0)
        lm.configure() 
        rospy.sleep(1.0)
        lm.activate() 
        self.assertEquals(State.ACTIVE, lm.get_current_state()) 
        rospy.sleep(1.0)
        lm.shutdown() 
        self.assertEquals(State.FINALIZED, lm.get_current_state()) 
        rospy.sleep(3.0)
        self.assertEquals(5, self.Count) 
        
        spin_thread.join(1.0)
        
        lm.__del__()
        
    def test_simulate_error_in_active(self):
        ROSINIT()
        lm = LifecycleManager(NODE_NAME)
        lm.set_error_cb(lambda ex: True)
        
        lm.configure()
        self.assertEquals(State.INACTIVE, lm.get_current_state())
        lm.activate() 
        self.assertEquals(State.ACTIVE, lm.get_current_state())
        lm.raise_error(Exception('Test'))
        self.assertEquals(State.UNCONFIGURED, lm.get_current_state())
        
        lm.set_error_cb(lambda ex: False)
        
        lm.configure()
        self.assertEquals(State.INACTIVE, lm.get_current_state())
        lm.activate() 
        self.assertEquals(State.ACTIVE, lm.get_current_state())
        lm.raise_error(Exception('Test'))
        self.assertEquals(State.FINALIZED, lm.get_current_state())
        
    def test_simulate_error_in_inactive(self):
        ROSINIT()
        lm = LifecycleManager(NODE_NAME)
        lm.set_error_cb(lambda ex: True)
        
        lm.configure()
        self.assertEquals(State.INACTIVE, lm.get_current_state())
        lm.raise_error(Exception('Test'))
        self.assertEquals(State.UNCONFIGURED, lm.get_current_state())
        
        lm.set_error_cb(lambda ex: False)
        
        lm.configure()
        self.assertEquals(State.INACTIVE, lm.get_current_state())
        lm.raise_error(Exception('Test'))
        self.assertEquals(State.FINALIZED, lm.get_current_state())
        
    def test_simulate_error_in_secondary_state(self):
        ROSINIT()
        lm = LifecycleManager(NODE_NAME)
        lm.set_error_cb(lambda ex: True)
        lm.set_transition_callback(Transition.CONFIGURE, lambda : lm.raise_error(Exception('Test'))) 
        
        lm.configure()
        self.assertEquals(State.UNCONFIGURED, lm.get_current_state())
        
        lm.set_error_cb(lambda ex: False)
        
        lm.configure()
        self.assertEquals(State.FINALIZED, lm.get_current_state())
        
class TestManagedNode(unittest.TestCase):

    def test_managed_node(self):
        class MyNode(ManagedNode):
            def __init__(self, component_fqn):
                super(MyNode,self).__init__(component_fqn)
                
            def _on_configure(self):
                print '_on_configure'
                return True
            
            def _on_cleanup(self):
                print '_on_cleanup '
                return True
            
            def _on_activate(self):
                print '_on_activate ' 
                return True

            def _on_deactivate(self):
                print '_on_deactivate '
                return True
            
            def _on_shutdown(self):
                print '_on_shutdown ' 
                return True
            
            def _on_error(self, ex):
                print '_on_error ' 
                return True
            
        ROSINIT()
        test_node = MyNode(NODE_NAME)
        
        test_node._lm.configure()
        self.assertEquals(State.INACTIVE, test_node._lm.get_current_state())
        test_node._lm.cleanup()
        self.assertEquals(State.UNCONFIGURED, test_node._lm.get_current_state())
        test_node._lm.configure()
        self.assertEquals(State.INACTIVE, test_node._lm.get_current_state())
        test_node._lm.activate()
        self.assertEquals(State.ACTIVE, test_node._lm.get_current_state())
        test_node._lm.deactivate()
        self.assertEquals(State.INACTIVE, test_node._lm.get_current_state())
        test_node._lm.activate()
        self.assertEquals(State.ACTIVE, test_node._lm.get_current_state())
        test_node._lm.shutdown()
        self.assertEquals(State.FINALIZED, test_node._lm.get_current_state())
        
    def test_default_implementation_managed_node(self):
        class ActiveNode(ManagedNode):
            def __init__(self, component_fqn):
                super(ActiveNode,self).__init__(component_fqn)
            
            def __del__(self):
                super(ActiveNode,self).__del__()
                
            def _on_activate(self):
                print '_on_activate ' 
                return True
            
        class SimpleNode(ManagedNode):
            def __init__(self, component_fqn):
                super(SimpleNode,self).__init__(component_fqn)
                
        ROSINIT()
        test_node = ActiveNode(NODE_NAME) 
        
        test_node._lm.configure()
        self.assertEquals(State.INACTIVE, test_node._lm.get_current_state())
        test_node._lm.cleanup()
        self.assertEquals(State.FINALIZED, test_node._lm.get_current_state())
        test_node.__del__()
        
        test_node = ActiveNode(NODE_NAME) 
        test_node._lm.configure()
        self.assertEquals(State.INACTIVE, test_node._lm.get_current_state())
        test_node._lm.activate()
        self.assertEquals(State.ACTIVE, test_node._lm.get_current_state())
        test_node._lm.deactivate()
        self.assertEquals(State.FINALIZED, test_node._lm.get_current_state())
        test_node.__del__()
        
        test_node = ActiveNode(NODE_NAME) 
        test_node._lm.configure()
        self.assertEquals(State.INACTIVE, test_node._lm.get_current_state())
        test_node._lm.activate()
        self.assertEquals(State.ACTIVE, test_node._lm.get_current_state())
        test_node._lm.shutdown()
        self.assertEquals(State.FINALIZED, test_node._lm.get_current_state())
        test_node.__del__()
        
        with self.assertRaises(TypeError):
            test_node = SimpleNode(NODE_NAME) 
        test_node.__del__()
        
if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'Test_Lifecycle_Manager', TestLifecycleManager)
    rostest.rosrun(PKG, 'Test_ManagedNode', TestManagedNode)

