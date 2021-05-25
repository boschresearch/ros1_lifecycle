#!/usr/bin/env python

from inspect import isfunction, ismethod, getcallargs

import rospy
import actionlib

from lifecycle_msgs.msg import LifecycleGoal, LifecycleAction, LifecycleResult, Lifecycle
from lifecycle.lifecycle_model import State, Transition, Result_Code, LifecycleModel
from lifecycle.broadcaster import LmEventBroadcaster

LIFECYCLE_ACTION_NAME = "lifecycle"
LIFECYCLE_STATE_TOPIC = "lifecycle_state"
    
class IllegalTransitionException(Exception):
    def __init__(self, arg):
        self.msg = 'IllegalTransitionException current state =' + str(arg[0]) + ', transition requested =' +str(arg[1])
        
class LifecycleCbException(Exception):
    """
    Thrown on invalid callback function
    """
    def __init__(self, msg):
        Exception.__init__(self, msg)

def check_args(fn, *args):
    '''Checks if the argument passed is a function or a method; and also checks for number of arguments'''
    if not isfunction(fn) and not ismethod(fn):
        return False
    try:
        getcallargs(fn, *args)
        return True
    except TypeError as e:
        return False

class LifecycleManager(object):
    def __init__(self, component_fqn):
        self._callbacks = {}
        self._current = State.UNCONFIGURED
        '''Variable to store the exception occurred during ACTIVE state'''
        self._active_ex = Exception('None')
        '''A lambda expression which raises an exception for the ERROR transition call back'''
        self.set_transition_callback(Transition.ERROR, lambda : (_ for _ in ()).throw(self._active_ex))
        self._as = actionlib.SimpleActionServer(component_fqn + "/" + LIFECYCLE_ACTION_NAME, LifecycleAction, self._goal_cb, False)
        self.state_pub_ = rospy.Publisher(component_fqn + "/" + LIFECYCLE_STATE_TOPIC, Lifecycle, queue_size = 10, latch=True)
        self.lm_broadcaster = LmEventBroadcaster(component_fqn);
        
    def __del__(self):
        self._as.__del__()

    def _publish_transition(self, transition, result_code):
        """
        Publishes the Lifecycle events. Whenever the state of the node changes,
        this method is called to publish the transition that triggered the change,
        current state and the result code of the transition
        :param transition: transition requested
        :type transition: Transition
        :param result_code: the result of the transition
        :type result_code: Result_Code
        :return: 
        """
        msg = Lifecycle()
        msg.transition = transition
        msg.end_state = self._current
        msg.result_code = result_code
        
        self.state_pub_.publish(msg)
        self.lm_broadcaster.sendLmEvent(msg)

    def start(self):
        """
        Starts the action server
        :param :
        :return:
        """
        self._as.start()

    def set_transition_callback(self, tr, cb):
        """
        Sets the callback functions of the respective transition
        :param tr : transition 
        :type tr: Transition
        :param cb : callback function
        :type cb : function
        :return:
        """
        if cb is None or not check_args(cb):
            raise LifecycleCbException("The callback %s must be callable with no arguments" %cb)
        for item in LifecycleModel.STATE_TRANSITION_VALIDPAIRS[tr]:
            self._callbacks[item] = cb

    def _goal_cb(self, goal):
        """
        A goal callback function for the action server updates the result of the action
        :param :
        :return:
        """
        result = LifecycleResult()
        try:
            if(self._handle_transition(goal.transition) != 0):
                result.end_state = self.get_current_state()
                self._as.set_succeeded(result, 'goal state reached okay')
            else:
                result.end_state = self.get_current_state()
                self._as.set_aborted(result, 'transition failed')
        except IllegalTransitionException:
            result.end_state = self.get_current_state()
            self._as.set_aborted(result, 'requested transition is not valid lifecycle transition')

    def _handle_transition(self, transition):
        """
        Handles the transition of node from one primary state to another primary state
        :param transition: requested Transition
        :type transition: Transition
        :return result_var: bool value to indicate success or failure
        """
        input_var = (self._current, transition)
        result_var = self._handle_primary_step(input_var)
        try :
            if(result_var != False):
                self._handle_secondary_step((self._current, Result_Code.SUCCESS))
            else:
                self._handle_secondary_step((self._current, Result_Code.FAILURE))
        except Exception as ex:
            if(self._handle_error_processing(ex)):
                self._handle_secondary_step((self._current, Result_Code.SUCCESS))
            else:
                self._handle_secondary_step((self._current, Result_Code.FAILURE))
                
        if(result_var != False):
            self._publish_transition(transition, Result_Code.SUCCESS)
        else :
            self._publish_transition(transition, Result_Code.FAILURE)
            
        return result_var

    def _handle_primary_step(self, input_var):
        """
        handles the first step of the transition i.e. from a primary state to a transitional state
        :param input_var: has the current state and the transition requested
        :type input_var: (State, Transition)
        :return result_var: bool value to indicate success or failure
        """
        try:
            #this statement may throw an exception if key is not found
            self._current = LifecycleModel.PRIMARY_STEPS[input_var]
            try:
                #this statement may throw an exception if key is not found
                cb_func = self._callbacks[input_var]
                try:
                    return cb_func()
                except Exception as ex:
                    return self._handle_error_processing(ex)
            except KeyError:
                return True
        except KeyError:
            raise IllegalTransitionException(input_var)
            
    def _handle_secondary_step(self, input_var):
        """
        Handles the second step of the transition i.e. from a transitional state to a primary state.
        :param input_var: has the current state and the result code
        :type input_var: (State, Result_Code)
        :return result_var: bool value to indicate success or failure
        """
        try:
            self._current = LifecycleModel.SECONDARY_STEPS[input_var]
            return True
        except KeyError:
            raise IllegalTransitionException(input_var)

    def _handle_error_processing(self,ex):
        """
        Handles the ErrorProcessing step of a transition
        :param ex: exception
        :type ex: Exception
        :return : bool value to indicate success or failure
        """
        self._current = State.ErrorProcessing
        try :
            return self.onError_(ex)
        except Exception as ex:
            return False

    def set_error_cb(self, cb):
        if cb is None or not check_args(cb, Exception):
            raise LifecycleCbException("The error callback %s must be callable with one argument" %cb)
        self.onError_ = cb
    
    def _isPrimaryState(self, state):
        """
        checks if the given state is Primary
        :param state: state of the node
        :param type: STATE
        :return : bool value to indicate if the given state is primary state
        """
        return((state == State.UNCONFIGURED) or (state == State.INACTIVE) or (state == State.ACTIVE) or (state == State.FINALIZED))
        
    def raise_error(self, ex):
        """
        Handles the error raised during any state of the node
        :param ex: exception
        :type ex: Exception
        :return : bool value to indicate success or failure
        """
        if (self._isPrimaryState(self._current)):
            self._active_ex = ex;
            return self._handle_transition(Transition.ERROR)
        else:
            return self._handle_error_processing(ex)
    
    def configure(self):
        return self._handle_transition(Transition.CONFIGURE)

    def activate(self):
        return self._handle_transition(Transition.ACTIVATE)

    def deactivate(self):
        return self._handle_transition(Transition.DEACTIVATE)

    def shutdown(self):
        return self._handle_transition(Transition.SHUTDOWN)

    def cleanup(self):
        return self._handle_transition(Transition.CLEANUP)

    def get_current_state(self):
        return self._current
