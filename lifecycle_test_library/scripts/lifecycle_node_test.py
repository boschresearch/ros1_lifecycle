#! /usr/bin/env python

from __future__ import print_function

#ROS Imports
import rospy
from rosservice import get_service_list

#Python Imports
import sys

#Package Imports
from lifecycle.lifecycle_model import State
from lifecycle_test_library.lifecycle_test_library import LifecycleTestLibrary

if __name__ == '__main__':

    rospy.init_node("Testing_Node")
    print("Hi Welcome to the lifecycle node testing tool!")
    
    #Load all the parmeters from the yaml file 
    #or load node_name from the parameter and set rest to default values
    
    #check for node_name set in yaml file
    if rospy.has_param('~node_name'):
        topic = rospy.get_param("~node_name")
        #check if node_name is sent from cmd line
    elif len(sys.argv) < 2:
        print("Need to configure node_name in a yaml file or as parameter")
        print("Syntax: lifecycle_node_test.py topic")
        sys.exit(-1)
    else:
        topic = sys.argv[1]
    
    states = ['INACTIVE', 'UNCONFIGURED', 
              'INACTIVE', 'ACTIVE', 'INACTIVE', 'UNCONFIGURED', 
              'ACTIVE', 'UNCONFIGURED']
    states = rospy.get_param("~states", states)
    
    timeout = rospy.get_param("~timeout", 3)
    
    delay = rospy.get_param("~delay", 1)
    
    publications = rospy.get_param("~publications", [])
    
    publishing_hz = rospy.get_param('~publishing_hz', [])
    
    subscriptions = rospy.get_param("~subscriptions", [])
    
    services = rospy.get_param("~services", [])
    
    print("Start Tests")
    #Start the test
    test_obj = LifecycleTestLibrary(topic, publications, publishing_hz, subscriptions, services)
    
    assert(test_obj.test_transitions(states, timeout, delay))
    
    #Check the behaviour of all the states of the node
    assert(test_obj.test_all_states())
    
    print("Node tested successfully")
