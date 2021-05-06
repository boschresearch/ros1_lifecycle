#!/usr/bin/env python

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


from abc import ABCMeta
from abc import abstractmethod

from lifecycle.manager import LifecycleManager, Transition

class ManagedNode(object):
    __metaclass__ = ABCMeta
    def __init__(self, component_fqn):
        super(ManagedNode,self).__init__()
        self._lm = LifecycleManager(component_fqn)
        self._lm.set_transition_callback(Transition.CONFIGURE, self._on_configure)
        self._lm.set_transition_callback(Transition.CLEANUP, self._on_cleanup)
        self._lm.set_transition_callback(Transition.ACTIVATE, self._on_activate)
        self._lm.set_transition_callback(Transition.DEACTIVATE, self._on_deactivate)
        self._lm.set_transition_callback(Transition.SHUTDOWN, self._on_shutdown)
        self._lm.set_error_cb(self._on_error)
        #start the action server
        self._lm.start()
        
    def __del__(self):
        self._lm.__del__()
        
    def _on_configure(self):
        return True
    
    def _on_cleanup(self):
        return False
    
    '''A node must not start directly after process creation when the life-cycle is in use,
    so the user needs to provide an on_activate callback and this is enforced by using abstractmethod.'''
    @abstractmethod
    def _on_activate(self):
        return True

    def _on_deactivate(self):
        return False
        
    def _on_shutdown(self):
        return True
    
    def _on_error(self, ex):
        return False
