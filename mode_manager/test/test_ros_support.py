
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

import subprocess
import unittest

import rospy
from roslaunch.substitution_args import resolve_args

from mode_manager.config_action import EntryExitProcessConfigAction


class ROSProcessTestCase(unittest.TestCase):
    def setUp(self):
        self.process = subprocess.Popen(["roscore"])
        rospy.init_node("test", anonymous=True)

    def tearDown(self):
        try:
            self.process.terminate()
        except OSError as e:
            pass

    def test_roslaunch_controller(self):
        pc = EntryExitProcessConfigAction(["roslaunch", resolve_args("$(find mode_manager)/examples/launch/set_param_to_arg.launch"),
                                                          "value:=foo"])
        try:
            rospy.get_param("/test/param")
            self.fail("should have thrown key error")
        except KeyError as e:
            pass

        pc.activate()
        value = rospy.get_param("/test/param")
        self.assertEquals("foo", value)


if __name__ == '__main__':
    unittest.main()
