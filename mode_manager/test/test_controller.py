
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

import unittest
from mock import MagicMock
import yaml
import os
import subprocess

import mode_manager.component_control as cc
from mode_manager.config_lifecycle import create_ros2_config_actions

from roslaunch.substitution_args import resolve_args

import mode_manager.config_action


class DummyConfigurationController(mode_manager.config_action.ConfigAction):
    def __init__(self):
        mode_manager.config_action.ConfigAction.__init__(self)
        self.activate_called = 0
        self.inactive_called = 0

    def _activate(self):
        self.activate_called+=1

    def _deactivate(self):
        self.inactive_called+=1

    def get_state(self):
        return self.activate_called, self.inactive_called


class ComponentControllerTestCase(unittest.TestCase):
    def test_dummycontroller(self):
        dc = DummyConfigurationController()

        dc._activate()
        dc._deactivate()
        self.assertEquals((1, 1), dc.get_state())

        dc.activate()
        self.assertEquals((2, 1), dc.get_state())
        dc.deactivate()
        self.assertEquals((2, 2), dc.get_state())

    def test_activation(self):
        dc = DummyConfigurationController()

        coordinator = cc.ComponentCoordinator({'test': dc})

        self.assertEqual((0, 0), dc.get_state())

        coordinator.activate(["test"])
        self.assertEqual((1, 0), dc.get_state())

        # second activation should have no effect
        coordinator.activate(["test"])
        self.assertEqual((1, 0), dc.get_state())

        # now stop
        coordinator.deactivate(["test"])
        self.assertEqual((1, 1), dc.get_state())

        # no effect on second again
        coordinator.deactivate(["test"])
        self.assertEqual((1, 1), dc.get_state())

        # 2nd round
        coordinator.activate(["test"])
        self.assertEqual((2, 1), dc.get_state())

        coordinator.deactivate(["test"])
        self.assertEqual((2, 2), dc.get_state())


class ControllerBuilderTestCase(unittest.TestCase):
    def setUp(self):
        self.recorder_launch_file_path = resolve_args("$(find mode_manager)/examples/launch/recorder.launch")
        self.assertTrue(os.path.exists(self.recorder_launch_file_path))
        self.param_launch_file_path = resolve_args("$(find mode_manager)/examples/launch/set_param_to_arg.launch")
        self.assertTrue(os.path.exists(self.param_launch_file_path))

    def test_build(self):
        builder = cc.ControllerBuilder()
        with open("../examples/config/example_modes.yaml") as yf:
            config = yaml.safe_load(yf)
            controllers = builder.build(config["components"], {
                'ros2_lifecycle': lambda c: create_ros2_config_actions(MagicMock())})

        components = {"param", "jai", "kinect"}

        self.assertEquals(components, set(controllers.keys()))
        for component in ["param", "kinect"]:
            self.assertEquals({"inactive", "active"}, set(controllers[component].get_configurations()))
            self.assertEquals({"inactive", "active"}, set(controllers[component].get_configurations()))

        for component in ["param"]:
            for config in ("inactive", "active"):
                self.assertTrue(isinstance(controllers[component]._configs[config],
                                           mode_manager.config_action.EntryExitProcessConfigAction))

        for component in ["jai"]:
            self.assertEquals({"unconfigured", "inactive", "active", "finalized"},
                              set(controllers[component]._configs.keys()))

        # further test of functioning in "test_ros_support"

if __name__ == '__main__':
    unittest.main()
