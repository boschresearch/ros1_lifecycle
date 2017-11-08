
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

import actionlib
import rospy
from mode_msgs.msg import *
from mode_msgs.srv import *
from rospy import logerr
import component_control as cc

class ModeManager(object):
    STATUS_TOPIC = "mode"
    CHANGE_ACTION = STATUS_TOPIC  # because it's an action, actual topics will be suffixed
    LIST_SERVICE = "list_modes"
    # the major dictionaries of the configuration
    COMPONENTS_PARAM = "~components"
    MODES_PARAM = "~modes"
    # keys into the dictionaries
    INITIAL_MODE_PARAM = "~initial_mode"

    CONFIGURATION_KEY = "component_configuration"
    INHERITS_KEY = "inherits"
    EXTENDS_KEY = cc.ControllerBuilder.EXTENDS_KEY
    FQN_KEY = "name"
    MODE_KEYS = (CONFIGURATION_KEY, INHERITS_KEY, EXTENDS_KEY, FQN_KEY)

    @staticmethod
    def _component_existence_check(component, components, mode, context):
        """
        :return: (has_error, msgs) True iff errors were found, in which cases msgs will contain explanations
        """
        has_error = False
        messages = []
        if component not in components.keys():
            messages.append("Component %s specified to %s in mode %s is not configured" % (component, context, mode))
            has_error = True
        return has_error, messages

    @staticmethod
    def check_configuration(components, modes, initial_mode):
        """
        :return: (has_error, msgs) True iff errors were found, in which cases msgs will contain explanations
        """
        has_error = False
        messages = []
        # do some sanity checking
        for mode_name in modes:
            mode = modes[mode_name]
            for key in mode.keys():
                # check for unknown keys
                if key not in ModeManager.MODE_KEYS:
                    messages.append("Item %s for mode %s is not a known configuration name" % (key, mode))
                # check for missing keys
                for mode_key in ModeManager.MODE_KEYS:
                    if mode_key not in mode.keys():
                        messages.append("Required configuration parameter %s not present for mode %s" % (mode_key, mode))
                        has_error = True
                if not has_error:
                    # check that all components listed are known
                    for component in mode[ModeManager.CONFIGURATION_KEY]:
                        err, msg = ModeManager._component_existence_check(component, components, mode, "configuration")
                        has_error |= err
                        messages.extend(msg)

        if initial_mode not in modes.keys():
            messages.append("Initial mode %s not in configured modes %s" % (initial_mode, modes.keys()))
            has_error = True

        return has_error, messages

    def __init__(self, status_topic=STATUS_TOPIC, change_action=CHANGE_ACTION,
                 list_service=LIST_SERVICE):
        self._status_pub    = rospy.Publisher(status_topic, Mode, queue_size=1)
        self._list_service  = rospy.Service(list_service, ListModes, self.list_handler)
        self._change_action = actionlib.action_server.ActionServer(change_action,
                                                                   ModeChangeAction,
                                                                   self.change_cb,
                                                                   auto_start=False)
        self._components = rospy.get_param(self.COMPONENTS_PARAM)
        self._modes = rospy.get_param(self.MODES_PARAM)
        self._initial_mode = rospy.get_param(self.INITIAL_MODE_PARAM)
        err, msg = self.check_configuration(self._components, self._modes, self._initial_mode)
        if err:
            logerr("Fatal errors in configuration found")
            for m in msg:
                logerr(m)
            raise Exception("Fatal errors in configuration found: %s." % "\n".join(msg))

        # build up a dictionary of component configurations for each mode
        self._component_configs = {}
        for name, mode in self._modes.items():
            configs = {}
            inherited_mode = mode.get(self.INHERITS_KEY, None)
            if inherited_mode is not None:
                configs = dict(self._modes[inherited_mode][self.CONFIGURATION_KEY])
            configs.update(mode[self.CONFIGURATION_KEY])

            # check that the configurations are complete
            diff = set(set(self._components.keys()).difference(configs.keys()))
            if len(diff) > 0:
                msg = "Missing component configurations %s in mode %s" % (diff, name)
                logerr(msg)
                raise Exception(msg)
            self._component_configs[name] = configs

        self._list_response = ListModesResponse()
        self._list_response.modes = self._modes.keys()

        builder = cc.ControllerBuilder()
        self._controllers = builder.build(self._components)

        # use initial mode as starting point
        self._start_mode(self._initial_mode)

    def start(self):
        self._change_action.start()

    def list_handler(self, _):
        return self._list_response

    def _start_mode(self, target_mode):
        # now apply changes
        for component_name, component_config in self._component_configs[target_mode].items():
            self._controllers[component_name].set_configuration (component_config)

    def change_cb(self, goal):
        """

        :param goal:
        :type goal: ModeChangeActionGoal
        :return:
        """

        try:
            target_mode = goal.goal.goal.target_mode

            if target_mode not in self._modes.keys():
                goal.set_rejected(text="Mode %s not known. Use list_modes to retrieve known modes" % goal.goal.target_mode)
                return
            else:
                goal.set_accepted()

            self._start_mode(target_mode)

            goal.set_succeeded()
        except Exception as e:
            goal.set_canceled(text=str(e))


def check_config_file(filename):
    import yaml
    with open(filename) as config:
        data = yaml.safe_load(config)
        components = data[ModeManager.COMPONENTS_PARAM.replace("~", "")]
        modes = data[ModeManager.MODES_PARAM.replace("~", "")]
        err, msg = ModeManager.check_configuration(components, modes, data[ModeManager.INITIAL_MODE_PARAM.replace("~", "")])
        if err:
            print("Found fatal errors in configuration")
            for m in msg:
                print(m)