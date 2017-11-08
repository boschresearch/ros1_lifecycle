#! /usr/bin/env python

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



class ModeManagerException(Exception):
    def __init__(self, message, exception=None):
        Exception.__init__(self, message, exception)


class ControllerException(ModeManagerException):
    """
    Something went wrong when building or invoking controller action, e.g., the target configuration did not exist,
    or similar things.
    """
    def __init__(self, message, exception=None):
        ModeManagerException.__init__(self, message, exception)


class ConfigActionException(ModeManagerException):
    """
    Something went wrong when executing a config action.
    """
    def __init__(self, message, exception=None):
        ModeManagerException.__init__(self, message, exception)


class BaseConfig(object):
    def __init__(self, configs, initial_config):
        self.configs = configs
        self.initial_config = initial_config
