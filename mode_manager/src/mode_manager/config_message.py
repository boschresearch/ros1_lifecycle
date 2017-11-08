
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

from . import ControllerException, ConfigActionException
import rospy
from rospy.topics import Publisher
from mode_manager.config_action import ConfigAction


class SendMessageConfigAction(ConfigAction):
    """
    A config action which sends a message on a given topic. It is an error if there is no receiver for the message.


    :type _client: rospy.topics.Publisher
    """
    _client = None

    def __init__(self, publisher, message):
        """

        :param publisher: The publisher to send the message on
        :type publisher: rospy.topics.Publisher
        :param message:
        :return:
        """
        ConfigAction.__init__(self)
        self._publisher = publisher
        self._message = message

    def _activate(self):
        if self._publisher.get_num_connections() < 1:
            raise ConfigActionException("There are no subscribers for publisher %s" % self._publisher.resolved_name)
        self._publisher.publish(self._message)

    def _deactivate(self):
        pass


class ServiceRequestConfigAction(ConfigAction):
    """
    A config action which invokes a given service and waits for the response.

    :type _client: rospy.ServiceProxy
    """
    _client = None

    def __init__(self, proxy, request):
        """
        :param proxy: The publisher to send the message on
        :type publisher: rospy.topics.Publisher
        :param request:
        :return:
        """
        ConfigAction.__init__(self)
        self._proxy = proxy
        self._request = request

    def _activate(self):
        try:
            self._proxy.call(self._request)
        except TypeError as ex:
            raise ConfigActionException("%s %s for service %s" % (ex, self._request, self._proxy))
        except rospy.ServiceException as ex:
            raise ConfigActionException("%s %s for service %s" % (ex, self._request, self._proxy))

    def _deactivate(self):
        pass
