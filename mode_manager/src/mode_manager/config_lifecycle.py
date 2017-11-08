
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
from . import BaseConfig
from lifecycle.client import LifecycleClient, create_client
from lifecycle_msgs.msg import LifecycleGoal
from mode_manager.config_action import ConfigAction


class LifecycleEventConfigAction(ConfigAction):
    """
    :type _client: lifecycle.client.LifecycleClient
    """
    _client = None

    def __init__(self, client, target_state):
        self._client = client
        self._target_state = target_state

    def _activate(self):
        self._client.go_to_state(self._target_state)

    def _deactivate(self):
        pass


def create_ros2_config_actions(client):
    return BaseConfig({"inactive": LifecycleEventConfigAction(client, LifecycleGoal.PSTATE_INACTIVE),
            "active": LifecycleEventConfigAction(client, LifecycleGoal.PSTATE_ACTIVE),
            "unconfigured": LifecycleEventConfigAction(client, LifecycleGoal.PSTATE_UNCONFIGURED),
            "finalized": LifecycleEventConfigAction(client, LifecycleGoal.PSTATE_FINALIZED)}, "unconfigured")


def create_ros2_lifecycle(component_fqn):
    client = create_client(component_fqn)
    return create_ros2_config_actions(client)


