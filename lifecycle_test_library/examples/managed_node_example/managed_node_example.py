#! /usr/bin/env python

from __future__ import print_function

#ROS Imports
import rospy
from std_msgs.msg import String

#Python Imports
import sys
import threading

#Package Imports
from lifecycle.managed_node import ManagedNode
from lifecycle_test_library.srv import *

if __name__ == '__main__':

    class MyActiveThread(threading.Thread):
        def __init__(self, threadID, name, counter, pub):
            threading.Thread.__init__(self)
            self.name = name
            self.counter = counter
            self._pub = pub
            self._terminate = False
            
        def run(self):
            self._rate = rospy.Rate(10) # 10hz
            while(not self._terminate and not rospy.is_shutdown()):
                self.hello_str = "Hello World %s" % rospy.get_time()
                self._pub.publish(self.hello_str)
                self._rate.sleep()
            
        def terminate_thread(self):
            self._terminate = True
    
    class MyNode(ManagedNode):
        def __init__(self, component_fqn):
            super(MyNode,self).__init__(component_fqn)
            self.node_name = component_fqn
            self._my_active_thread = None
            self._handle_add_two_ints = None
        
        def _on_configure(self):
            print ('_on_configure ')
            self._pub = rospy.Publisher(self.node_name + '/chatter', String, queue_size=10)
            try:
                self._sub = rospy.Subscriber("noise", String, self._callback)
            except Exception as ex:
                print (ex)
            #Assign the inactive callback function to the handle
            self._handle_add_two_ints = self._service_callback_inactive
            self._srv = rospy.Service(self.node_name + '/add_two_ints', AddTwoInts, self._handle_add_two_ints)
            return True
        
        def _on_cleanup(self):
            print ('_on_cleanup ')
            self._pub.unregister()
            self._pub = None
            #TODO: check if just deleting the subscriber variable would disable the callback function too!
            self._sub.unregister()
            self._sub = None
            try:
                self._srv.shutdown()
                self._srv = None
            except Exception as ex:
                print (ex)
            return True
        
        def _on_activate(self):
            print ('_on_activate ')
            self._my_active_thread = MyActiveThread(1, 'NAME', 1, self._pub)
            self._my_active_thread.start()
            #Assign the active callback function to the handle
            self._handle_add_two_ints = self._service_callback_active
            return True

        def _on_deactivate(self):
            print ('_on_deactivate ')
            self._stop_my_active_thread()
            #Assign the inactive callback function to the handle
            self._handle_add_two_ints = self._service_callback_inactive
            return True
        
        def _on_shutdown(self):
            print ('_on_shutdown ')
            if (self._my_active_thread != None):
                self._stop_my_active_thread()
            if (self._pub != None):
                self._pub.unregister()
                self._pub = None
            if (self._sub != None):
                self._sub.unregister()
                self._sub = None
            if (self._srv != None):
                self._srv.shutdown()
                self._srv = None
            return True
        
        def _on_error(self, ex):
            print ('_on_error ' )
            if (self._my_active_thread != None):
                self._stop_my_active_thread()
            if (self._pub != None):
                self._pub.unregister()
                self._pub = None
            if (self._sub != None):
                self._sub.unregister()
                self._sub = None
            if (self._srv != None):
                self._srv.shutdown()
                self._srv = None
            return True
            
        def _callback(self, msg):
            rospy.loginfo(rospy.get_caller_id() + "I heard %s", msg.data)
            
        def _stop_my_active_thread(self):
            self._my_active_thread.terminate_thread()
            self._my_active_thread.join()
            self._my_active_thread = None
        
        def _service_callback_active(self, req):
            return AddTwoIntsResponse(req.a + req.b)
            
        def _service_callback_inactive(self, req):
            #do nothing
            pass
            
    rospy.init_node("Example_Node")
    print("Welcome to the example node")
    
    eg_node = MyNode("Example_Node")
    
    while not rospy.is_shutdown():
        pass
