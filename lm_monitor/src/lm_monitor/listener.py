#!/usr/bin/env python

from __future__ import print_function
from inspect import isfunction, ismethod, getcallargs

import rospy

import threading

from lifecycle_msgs.msg import lm_events
from lm_monitor.monitor import LmMonitor

class ListenerCbException(Exception):
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

## Thread that subscribes to the /lm_events topic and
## prints the messages with string contents.

class LmEventListenerThread(threading.Thread):
    def __init__(self, tl, app_cb):
        threading.Thread.__init__(self)
        self.tl = tl
        self.app_cb = app_cb
    
    def run(self):
        rospy.Subscriber("/lm_events", lm_events, self.lmEventlistener_callback)
        rospy.spin()

    def lmEventlistener_callback(self, msg):
        self.last_update_ros_time = rospy.Time.now()
        self.tl.setLmEvent(msg, self.last_update_ros_time)
        #call the application's call back function with the LmEventsbuffer value
        self.app_cb(self.tl.LmEventsBuffer)
        
class LmEventListener(LmMonitor):
    """
    :class: LmEventListener is a subclass of :class:`LmMonitor` that
    subscribes to the ``"/lm_events"`` message topic, and calls :meth:`LmMonitor.setLmEvent`
    with each incoming event message.

    In this way a LmEventListener object automatically
    stays up to to date with all current events.
    """
    
    def __init__(self, app_cb, *args):
        super(LmEventListener,self).__init__()
        if app_cb is None or not check_args(app_cb, 1):
            raise ListenerCbException("The error callback %s must be callable with one argument" %app_cb)
        thr = LmEventListenerThread(self, app_cb)
        thr.setDaemon(True)
        thr.start()
        
    def __delete__(self):
        self.thr.join(3)
        
if __name__ == '__main__':
          
    def Listen_Cb(lm_events_buffer):
        print (lm_events_buffer)
        print ()
            
            
    rospy.init_node('LifecycleEventListner')
    listener = LmEventListener(Listen_Cb)
    
    try:
        while not rospy.is_shutdown():
            pass
    except rospy.ROSInterruptException:
        pass
