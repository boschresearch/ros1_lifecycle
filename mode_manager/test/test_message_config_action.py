
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
from mode_manager.config_message import SendMessageConfigAction
from mode_manager import ConfigActionException
from std_msgs.msg import String
import rospy


class MessageConfigActionTestCase(unittest.TestCase):
    """
    :type mock: MagicMock
    :type mock.publish: MagicMock
    :type string: std_msgs.msg.String
    """

    def setUp(self):
        self.string = String()
        self.string.data = "foo"
        self.mock = MagicMock(spec=rospy.Publisher)
        self.mock.resolved_name = MagicMock(side_effect=lambda: "foo")
        self.mock.publish = MagicMock()
        self.action = SendMessageConfigAction(self.mock, self.string)

    def test_check_subscribers(self):
        self.mock.get_num_connections = MagicMock(side_effect=lambda: 0)

        try:
            self.action.activate()
            self.fail("Should have thrown an exception")
        except ConfigActionException as ex:
            pass

        # mock in Python 2 doesn't have assert_not_called, yet
        self.assertEquals(0, self.mock.publish.call_count)

    def test_publish(self):
        self.mock.get_num_connections = MagicMock(side_effect=lambda: 1)
        self.action.activate()
        self.mock.publish.assert_called_once_with(self.string)


if __name__ == '__main__':
    unittest.main()
