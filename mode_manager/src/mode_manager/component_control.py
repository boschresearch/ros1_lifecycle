
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

import os.path
from roslaunch.substitution_args import resolve_args

from . import BaseConfig, ConfigActionException, ControllerException
from mode_manager.config_action import ConfigAction, CompoundAction, PersistentProcessConfigAction, \
    EntryExitProcessConfigAction


class ComponentController(object):
    """
    Switches configurations.
    """
    def __init__(self, configs, initial_config):
        """

        :param configs:
        :type configs: dict[str, mode_manager.config_action.ConfigAction]
        :param initial_config:
        :return:
        """
        if initial_config not in configs:
            raise ControllerException("Initial configuration %s not in config dictionary %s" %
                                      (initial_config, configs))
        self._configs = dict(configs)
        self._current_config = None

    def set_configuration(self, config_name):
        if config_name not in self._configs:
            raise ControllerException("Configuration %s not known" % config_name)
        # noop?
        if self._configs[config_name] is self._current_config:
            return
        # else disable current and enable new
        if self._current_config is not None:
            self._current_config.deactivate()
        self._current_config = self._configs[config_name]
        self._current_config.activate()

    def get_configurations(self):
        return self._configs.keys()


class ControllerBuilder(object):
    """
    Load the controller configurations.
    """
    ARG_TYPE = "type"
    ARG_ARGS = "args"
    ARG_ENTRY = "entry"
    ARG_EXIT = "exit"
    ARG_CONFIGS = "configs"
    ARG_INITIAL_CONFIG = "initial_config"
    ARG_LAUNCH_FILE = "launch_file"
    TYPE_ROSLAUNCH = "roslaunch"
    TYPE_NONE = "none"
    COMMAND_ROSLAUNCH = "roslaunch"
    EXTENDS_KEY = "extends"
    FQN_KEY = "name"

    def __init__(self):
        pass

    def build(self, components, bases={}):#
        """

        :param components:
        :param bases: A dictionary of callables (taking a component argument) that return a BaseConfig
        :return:
        """
        controllers = {}
        for component_name, component in components.items():
            configs = {}
            # check if we're extending something
            extends = component.get(ControllerBuilder.EXTENDS_KEY, None)
            if extends is not None:
                if extends in bases.keys():
                    base_config = bases[extends](component)
                    configs.update(base_config.configs)
                    if not self.ARG_INITIAL_CONFIG in component:
                        component[self.ARG_INITIAL_CONFIG] = base_config.initial_config
                elif extends in components.keys():
                    configs.update(components[extends])
                else:
                    raise ControllerException("Unknown component %s specified in extends of component %s" %
                                              (extends, component_name))

            for config_name, config in component.get(self.ARG_CONFIGS, {}).items():
                actions = configs.get(config_name, [])
                # go through all creation methods
                self._create_entry_exit(actions, config_name, config)
                self._create_persistent(actions, config_name, config)

                if len(actions) > 1:
                    configs[config_name] = CompoundAction(actions)
                elif len(actions) > 0:
                    configs[config_name] = actions[0]

            controllers[component_name] = ComponentController(configs, component[self.ARG_INITIAL_CONFIG])

        return controllers

    def _load_roslaunch(self, config):
        args = config.get(self.ARG_ARGS, None)
        launch_file = resolve_args(config[self.ARG_LAUNCH_FILE])
        if not os.path.exists(launch_file):
            raise ControllerException("Lanch file %s (resolved from %s) for roslaunch config action does not exist" %
                                      (launch_file, config[self.ARG_LAUNCH_FILE]))
        command = [self.COMMAND_ROSLAUNCH, launch_file]
        if args is not None:
            if isinstance(args, list):
                command.extend(args)
            else:
                command.append(args)
        return command

    def _handle_command(self, config):
        config_type = config[self.ARG_TYPE]
        if config_type == self.TYPE_ROSLAUNCH:
            return self._load_roslaunch(config)
        else:
            return ControllerException("Unhandled command type " + config)

    def _create_entry_exit(self, actions, name, component):
        if self.ARG_ENTRY in component:
            entry_command = self._handle_command(component[self.ARG_ENTRY])
            exit_command = None
            if self.ARG_EXIT in component:
                exit_command = self._handle_command(component[self.ARG_EXIT])
            actions.append(EntryExitProcessConfigAction(entry_command, exit_command))

    def _create_persistent(self, actions, name, component):
        if self.ARG_TYPE in component:
            config_type = component[self.ARG_TYPE]
            if config_type == self.TYPE_ROSLAUNCH:
                command = self._load_roslaunch(component)
                actions.append(PersistentProcessConfigAction(command))
            elif config_type == self.TYPE_NONE:
                actions.append(ConfigAction())  # base class is no-op


class ComponentCoordinator(object):
    def __init__(self, controllers):
        self._controllers = controllers

    def _for_each(self, components, fn):
        for comp in components:
            if comp in self._controllers:
                try:
                    fn(self._controllers[comp])
                except ControllerException as ce:
                    print("Error activate component %s: %s" % (comp, ce))
                except ConfigActionException as ce:
                    print("Error activate component %s: %s" % (comp, ce))
                except Exception as e:
                    print("WHOA: Unexpected exception %s" % e)
            else:
                print("Could not activate component %s: No controller configured")

    def activate(self, components):
        self._for_each(components, lambda x: x.activate())

    def deactivate(self, components):
        self._for_each(components, lambda x: x.deactivate())