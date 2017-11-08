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


PKG = "lifecycle"
import roslib; roslib.load_manifest(PKG)

import unittest
from mock import MagicMock

import actionlib
from actionlib_msgs.msg import GoalStatus

from lifecycle_msgs.msg import LifecycleGoal, Lifecycle
from lifecycle.client import LifecycleTransitionSequence, LifecycleAPIException, LifecycleClient

class LifecycleTransitionSequenceTest(unittest.TestCase):
    def setUp(self):
        self._called = False
        self._result = None
        self._goals = []
        self._transition_cb = None

    def _completion_cb(self, result):
        self._called = True
        self._result = result

    def test_empty_event(self):
        seq = LifecycleTransitionSequence(None, (), self._completion_cb)
        seq.go()
        self.assertTrue(self._called)

    def test_bad_event(self):
        try:
            LifecycleTransitionSequence(None, ("activate", ), self._completion_cb)
            self.fail("Should have raised an exception")
        except LifecycleAPIException:
            pass

    def test_bad_cb(self):
        try:
            LifecycleTransitionSequence(None, (), None)
            self.fail("Should have raised an exception")
        except LifecycleAPIException:
            pass

    def _send_goal_side_effect(self, goal, transition_cb):
        self._goals.append(goal)
        goal = MagicMock()
        self._transition_cb = transition_cb

    def test_single_event(self):
        client = MagicMock(spec=actionlib.action_client)
        client.send_goal = MagicMock(side_effect=self._send_goal_side_effect)  # (goal, self._transition_cb)

        seq = LifecycleTransitionSequence(client, (LifecycleGoal.EV_CONFIGURE, ), self._completion_cb)
        seq.go()
        # check that side effect has captured
        self.assertFalse(self._called)
        self.assertIsNotNone(self._transition_cb)
        self.assertEquals(1, len(self._goals))

        # configure next step
        goal = _create_goal_mock(actionlib.CommState.ACTIVE, GoalStatus.PENDING)

        # this shouldn't call, yet, because CommState was "active"
        self._transition_cb(goal)
        # still not called
        self.assertFalse(self._called)

        # now this should trigger it
        goal = _create_goal_mock(actionlib.CommState.DONE, GoalStatus.SUCCEEDED)
        self._transition_cb(goal)
        self.assertTrue(self._called)
        self.assertTrue(self._result)

        self.assertTrue(seq.is_done())

    def test_event_sequence(self):
        client = MagicMock(spec=actionlib.action_client)
        client.send_goal = MagicMock(side_effect=self._send_goal_side_effect)

        seq = LifecycleTransitionSequence(client, (LifecycleGoal.EV_CONFIGURE, LifecycleGoal.EV_ACTIVATE),
                                          self._completion_cb)
        seq.go()

        goal = _create_goal_mock(actionlib.CommState.DONE, GoalStatus.SUCCEEDED)
        self._transition_cb(goal)
        self.assertFalse(seq.is_done())
        self._transition_cb(goal)
        self.assertTrue(seq.is_done())

        self.assertEquals([LifecycleGoal.EV_CONFIGURE, LifecycleGoal.EV_ACTIVATE], [g.transition for g in self._goals])
        self.assertTrue(self._called)
        self.assertTrue(self._result)

    def test_server_failure(self):
        client = MagicMock(spec=actionlib.action_client)
        client.send_goal = MagicMock(side_effect=self._send_goal_side_effect)

        seq = LifecycleTransitionSequence(client, (LifecycleGoal.EV_CONFIGURE, ),
                                          self._completion_cb)
        seq.go()
        goal = _create_goal_mock(actionlib.CommState.DONE, GoalStatus.ABORTED)
        self._transition_cb(goal)
        self.assertTrue(seq.is_done())
        self.assertFalse(seq.has_succeeded())


def _create_goal_mock(commstate, status):
    goal = MagicMock()
    goal.get_comm_state = MagicMock(side_effect=lambda: commstate)
    goal.get_goal_status = MagicMock(side_effect=lambda: status)
    return goal


class LifecycleClientTest(unittest.TestCase):
    def setUp(self):
        self._result = None
        self._goals = []

    def _completion_cb(self, result):
        self._result = result

    def _send_goal_se(self, goal, cb):
        self._goals.append(goal)
        goal = _create_goal_mock(actionlib.CommState.DONE, GoalStatus.SUCCEEDED)
        cb(goal)

    def test_client(self):
        action_client = MagicMock(spec=actionlib.action_client.ActionClient)
        action_client.send_goal = MagicMock(side_effect=self._send_goal_se)
        client = LifecycleClient(action_client)
        client.go_to_state(Lifecycle.PSTATE_ACTIVE, self._completion_cb)
        self.assertEquals([20, 22], [g.transition for g in self._goals])

if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, "test_lifecycle_client", LifecycleClientTest)
    rostest.rosrun(PKG, "test_lifecycle_transition_sequence", LifecycleTransitionSequenceTest)

