
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

from inspect import isfunction, ismethod, getcallargs

import rospy
import actionlib
from actionlib.action_client import CommState
from actionlib_msgs.msg import GoalStatus

from lifecycle_msgs.msg import LifecycleGoal, LifecycleAction, Lifecycle
from lifecycle.lifecycle_model import LifecycleModel

LIFECYCLE_ACTION_NAME = "lifecycle"
LIFECYCLE_STATE_TOPIC = "lifecycle_state"


class LifecycleAPIException(Exception):
    """
    Thrown on bad use of the lifecycle API
    """
    def __init__(self, msg):
        Exception.__init__(self, msg)


def check_args(fn, *args):
    if not isfunction(fn) and not ismethod(fn):
        return False
    try:
        getcallargs(fn, *args)
        return True
    except TypeError as e:
        return False


class LifecycleTransitionSequence(object):
    def __init__(self, client, events, completion_cb):
        for event in events:
            if event not in LifecycleModel.KNOWN_EVENTS:
                raise LifecycleAPIException("Event %s is not a life-cycle event" % event)
        # make sure the completion_callback works, or we'll get an exception much later
        if completion_cb is None or not check_args(completion_cb, True):
            raise LifecycleAPIException("Invalid completion_cb, got '%s', but function with 1 unbound "
                                        "argument is required" % completion_cb)

        self._client = client
        self._events = list(events)
        self._current_handle = None
        self._cancelled = False
        self._completion_cb = completion_cb
        self._result = None

    def go(self):
        self._step()

    def cancel(self):
        self._cancelled = True
        if self._current_handle is not None:
            self._current_handle.cancel()
            self._current_handle = None

    def is_done(self):
        """
        Returns true when the sequence has completed processing, either because
        it was consumed, or because it was cancelled. Use has_succeeded to determine the status
        """
        return self._cancelled or len(self._events) == 0

    def get_result(self):
        """
        Gets the last result from the server.
        :return:
        """
        return self._result

    def has_succeeded(self):
        """
        Returns true when the sequence was successfully executed
        :return:
        """
        return not self._cancelled and self._result == GoalStatus.SUCCEEDED

    def _step(self):
        if self._cancelled or len(self._events) == 0:
            self._completion_cb(self.has_succeeded())
        else:
            goal = LifecycleGoal()
            goal.transition = self._events[0]
            self._current_handle = self._client.send_goal(goal, self._transition_cb)

    def _transition_cb(self, client_goal_handle):
        # if we completed the current transition, invoke the next
        # TODO check for errors
        if client_goal_handle.get_comm_state() == CommState.DONE:
            self._result = client_goal_handle.get_goal_status()
            # on success, we remove the current event only
            if self.has_succeeded():
                self._events.pop(0)
            # on failure, we remove everything
            elif self._result == GoalStatus.ABORTED:
                self._events = []
            else:
                print(self._result)

            self._step()


class LifecycleClient(object):
    def __init__(self, client):
        """
        ONLY USE DIRECTLY DURING TESTING -- Use "create_client" for normal creation.

        Creates a life-cycle client that uses the given action-client to communicate. Also exposes a state_cb
        for registration with state-listeners (create_client does that correctly)
        :param client:
        :return:
        """
        self._client = client
        self._server_state = LifecycleGoal.PSTATE_UNCONFIGURED
        self._handle = None

    def go_to_state(self, target_state, completion_cb):
        """
        Sends the necessary events to go the given target state, based on the current state. Invokes "completion_cb"
        when done. completion_cb should take a single boolean to indicate success.

        :param target_state:
        :return:
        """
        self.completion_cb_ = completion_cb
        
        # if we're already in the target state, do nothing
        if target_state == self._server_state:
            completion_cb(True)
            return

        # cancel current sequence if there is one
        if self._handle is not None:
            self.cancel()

        # otherwise start the sequence of events to get to the target state
        events = LifecycleModel.EVENTS.get((self._server_state, target_state), None)
        if events is not None:
            self._handle = LifecycleTransitionSequence(self._client, events, self._transition_completion_cb)
            self._handle.go()

    def cancel(self):
        if self._handle is not None:
            self._handle.cancel()
            self._handle = None

    def state_cb(self, msg):
        """
        Stores the last server state internally
        :param msg:
        :type msg: Lifecycle
        :return:
        """
        self._server_state = msg.end_state
    
    def _transition_completion_cb(self, result):
        self._handle = None
        self.completion_cb_(result)


def create_client(component_fqn):
    action_client = actionlib.action_client.ActionClient(component_fqn + "/" + LIFECYCLE_ACTION_NAME,
                                                  LifecycleAction)
    action_client.wait_for_server(timeout=rospy.Duration(1.0))
    lc_client = LifecycleClient(action_client)
    rospy.Subscriber(component_fqn + "/" + LIFECYCLE_STATE_TOPIC, Lifecycle, lc_client.state_cb)
    return lc_client
