
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

from . import ControllerException


class WrappedProcess:
    def __init__(self, command, args=None):
        self._command = command
        if args is not None:
            if isinstance(args, (list, tuple)):
                self._command.extend(args)
            else:
                self._command.append(args)
        self._process = None

    def run(self):
        if self._process is not None:
            return
        try:
            print("Running ", self._command)
            self._process = subprocess.Popen(self._command)
        except OSError as e:
            raise ControllerException("Could not execute %s" % " ".join(self._command), e)

    def terminate(self):
        try:
            print("Stopping " % self._command)
            self._process.terminate()
            self.wait()
            print("done")
        except OSError:
            # ignore when the process has already terminated
            return 0

    def wait(self):
        if self._process is None:
            return
        try:
            self._process.wait()
        except OSError as e:
            pass
        finally:
            self._process = None


class NoopProcess(object):
    def run(self):
        pass

    def terminate(self):
        pass

    def wait(self):
        pass


class ConfigAction(object):
    """
    Base class for actions that configure components.
    """
    def __init__(self):
        self.active = False

    def activate(self):
        """
        Activates the configuration.
        :throws: ControllerException when there is a problem
        :return: None
        """
        if self.active:
            return
        self._activate()
        self.active = True

    def _activate(self):
        pass

    def deactivate(self):
        """
        Deactivates the configuration.
        :throws: ControllerException when there is a problem
        :return: None
        """
        if not self.active:
            return
        self._deactivate()
        self.active = False

    def _deactivate(self):
        pass


class CompoundAction(ConfigAction):
    """
    Configuration action that invokes a sequence of actions
    """
    def __init__(self, actions):
        ConfigAction.__init__(self)
        self._actions = list(actions)

    def _activate(self):
        for action in self._actions:
            action.activate()

    def _deactivate(self):
        for action in self._actions:
            action.deactivate()


class PersistentProcessConfigAction(ConfigAction):
    """
    Action that executes a process on activation, and kills it on process on deactivation.
    Arguments taken are the process and its arguments.
    """

    def __init__(self, command, args=None):
        ConfigAction.__init__(self)
        self._process = WrappedProcess(command, args)

    def _activate(self):
        self._process.run()

    def _deactivate(self):
        self._process.terminate()


class EntryExitProcessConfigAction(ConfigAction):
    _NOOP = NoopProcess()
    """
    Action that executes one process on activation, and another one on deactivation.
    Arguments taken are the process and its arguments.
    """

    def __init__(self, entry_command=None, exit_command=None, entry_args=None, exit_args=None):
        ConfigAction.__init__(self)
        if entry_command is not None:
            self._entry_process = WrappedProcess(entry_command, entry_args)
        else:
            self._entry_process = self._NOOP
        if exit_command is not None:
            self._exit_process = WrappedProcess(exit_command, exit_args)
        else:
            self._exit_process = self._NOOP

    def _activate(self):
        self._entry_process.run()
        self._entry_process.wait()

    def _deactivate(self):
        self._exit_process.run()
        self._exit_process.wait()