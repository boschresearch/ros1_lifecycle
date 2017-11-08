
from __future__ import print_function

#ROS Imports
import rospy
import actionlib
from rosgraph import Master, MasterError
from rospy.msproxy import MasterProxy
from rosservice import get_service_list

#Python Imports
from numpy import isclose

#Package Imports
from lifecycle.client import LifecycleClient, create_client
from lifecycle.lifecycle_model import State, LifecycleModel

SLEEP_TIME = 0.001
        
class LifecycleTestException(Exception):
    '''
    throw on bad use of the Lifecycle test library
    '''
    def __init__(self, msg):
        Exception.__init__(self, msg)
        
class LmClient(object):
    '''
    A class to create a client for the managed node to handle the state transitions
    '''
    def __init__(self, component_fqn):
        self._client = create_client(component_fqn)
        self._status = None
        self._waiting = 0
        
    def go_to_state(self, state, timeout = 3):
        '''
        Transitions the node through various states to reach the specified state
        :param state: the end state the node has to transition to
        :param type: STATE 
        :param timeout: timeout for the transition to complete in seconds
        :return : bool to indicate sucess or failure
        '''
        #call the client to make the state change
        self._client.go_to_state(state, self._transition_cb)
        
        while(self._status == None):
            rospy.sleep(SLEEP_TIME)
            self._waiting += SLEEP_TIME
            if (self._waiting >= timeout):
                print("Time-out occurred")
                return False
        
        self._waiting = 0
        if(self._status == True):
            self._status = None #reset the variable for the next transition
            return True
        else:
            print("Couldn't transition")
            return False
    
    def _transition_cb(self, result):
        '''
        callback for the client
        '''
        self._status = result;

class NodeStateSequencer(object):
    '''
    Class to transition the node from one state to another sequentially with a delay between two transitions
    '''
    def __init__(self, lm_client, states, timeout, delay):
        self._lm_client = lm_client
        self._states = states
        self._timeout = timeout
        self._delay = delay
        
    def start_sequencer(self):
        '''
        Starts making the transitions from one state to another according to the list "_states"
        :return result: success or failure of the state transitions
        :type   result: bool
        '''
        for state in self._states:
            #call the Lm client to make the state change
            result = self._lm_client.go_to_state(state, self._timeout)
            if (result == False):
                print("Transition %s failed" %LifecycleModel.STATE_TO_STR[state])
                return False
            rospy.sleep(self._delay) #wait until specified time before going to next state
        
        return True #all the states in the given list are completed so return success
    

class NodeInfo(object):
    def __init__(self, node_name):
        self._node_name = node_name
        self._master    = Master("NodeInfo")
        try:
            self._uri   = self._master.lookupNode(self._node_name)
        except MasterError:
            raise LifecycleTestException("Node %s not found" %self._node_name)
        import threading 
        self.lock = threading.Lock() 
        self._mproxy    = MasterProxy(self._uri)
        
    def get_publisher_list(self):
        '''
        Returns the list of publisher names that are registered by the node
        :return _pub: list of publishers
        :return type: [string]
        '''
        self._result = self._mproxy.getPublications()
        if (self._result != []):
            _,_,self._pub = self._result
        else:
            self._pub = []
        return self._pub
        
    def get_publishers_hz(self, publishers):
        '''
        Finds the publishing rate of the given publishers
        :param publishers: List of publisher names
        :param type: [string]
        :return _pub_hz: list of rate of each publisher
        :return type: [float]
        '''
        self._node_pub = self.get_publisher_list()
        self._pub_hz = []
        for pub in publishers:
            self._temp = [x for x in self._node_pub if (x[0]==pub)]
            if self._temp == []:
                hz = -1 #report error as the required topic is not published by the node
            else:
                hz = self._get_publisher_hz(self._temp[0]) #we expect to find only one topic with a particular name
            self._pub_hz.append(hz)
        return self._pub_hz
        
    def get_subscriber_list(self):
        '''
        Returns the list of subscriber names that are registered by the node
        :return _sub: list of subscribers
        :return type: [string]
        '''
        self._result = self._mproxy.getSubscriptions()
        if (self._result != []):
            _,_,self._sub = self._result
        else:
            self._sub = []
        return self._sub
    
    def get_services_list(self):
        '''
        Returns the list of service names that are registered by the node
        :return _srv: list of services
        :return type: [string]
        '''
        self._srv = get_service_list('/' + self._node_name)
        return self._srv
        
    def _get_publisher_hz(self, publisher):
        '''
        Finds the publishing rate of the given publisher by subscribing to it for 3 seconds
        :param publishers: publisher name
        :param type: string
        :return _pub_hz: rate of the publisher
        :return type: float
        '''
        #TODO:Need to implement a different logic to compute the rate, maybe use logic similar to class ROSTopicHz from rostopic
        self._msg_count = 0
        self._wait_time = 3
        self._temp_sub = rospy.Subscriber(publisher[0], rospy.AnyMsg, self._hz_calc_cb)
        rospy.sleep(self._wait_time)
        self._rate = self._msg_count / self._wait_time
        self._msg_count = 0
        return self._rate
        
    def _hz_calc_cb(self, msg):
        '''
        Callback function for the subscribed topic
        '''
        with self.lock:
            self._msg_count = self._msg_count + 1
        return True
    
class LifecycleTestLibrary(object):
    def __init__(self, component_fqn, pub, hz, sub, srv, **kwargs):
        '''
        Accepts the node name from the user for which the tests have to be performed
        :param component_fqn: node name
        :param type: string
        :param pub: list of [publishers, msg_type]
        :param  hz: list of publishing rates [float]
        :param sub: list of [subscribers, msg_type]
        :param srv: list of services
        '''
        self._node_name = component_fqn
        self.lm_client = LmClient(component_fqn)
        #Configure node values
        self.set_expected_node_info(pub, hz, sub, srv)
        
    def set_expected_node_info(self, pub, hz, sub, srv):
        '''
        This method is called to set the node information such as the list of publishers, 
        list of subscribers and the list of services offered by the node.
        :param pub: list of [publishers, msg_type]
        :param  hz: list of publishing rates [float]
        :param sub: list of [subscribers, msg_type]
        :param srv: list of services
        '''
        self.npub = pub
        self.nhz = hz
        self.nsub = sub
        self.nsrv = srv
        
    def test_transitions(self, states, timeout, transition_delay):
        '''
        Tests the node for multiple transitions between different states. Except FINALIZED!
        :param states: list of state names the node has to transition to in a sequence
        :param timeout: maximum time each transition has to take in seconds
        :param transition_delay: time duration between two transitions in seconds
        :return result: success or failure of the transitions
        '''
        #convert the state names in str to state codes
        self._state_codes = [LifecycleModel.STR_TO_STATE[state] for state in states]
        
        self._nss = NodeStateSequencer(self.lm_client, self._state_codes, timeout, transition_delay)
        return self._nss.start_sequencer()
        
    def _collect_node_info(self):
        '''
        Checks the node for the registered Publications and its rate, Subscriptions and Services
        '''
        self._ni = NodeInfo(self._node_name)
        self._pub = self._ni.get_publisher_list()
        self._hz = self._ni.get_publishers_hz(self.npub)
        self._sub = self._ni.get_subscriber_list()
        self._srv = self._ni.get_services_list()
        
    def test_unconfigured_state(self):
        '''
        Changes the node to UNCONFIGURED state and check for the pub, sub & srv to be non-existent
        '''
        if(self.lm_client.go_to_state(State.UNCONFIGURED)):
            self._collect_node_info()
            #Check that none the expected node's publishers are not present in the UNCONFIGURED state
            for pub in self.npub:
                result = [x[0] for x in self._pub if (x[0]==pub)]    #Compare the publisher name ignore the msg type
                if (result != []):
                    print ("The publisher topic is registered in UNCONFIGURED state")
                    print (result)
                    return False
            #Check that none of the expected node's subscribers are not subscribed to in the UNCONFIGURED state
            for sub in self.nsub:
                result = [x[0] for x in self._sub if (x[0]==sub)]    #Compare the subscriber name ignore the msg type
                if (result != []):
                    print ("The subscriber topic is registered in UNCONFIGURED state")
                    print (result)
                    return False
            #Check that none of the expected node's services are not subscribed to in the UNCONFIGURED state
            for srv in self.nsrv:
                result = [x for x in self._srv if (x==srv)]
                if (result != []):
                    print ("The service is registered in UNCONFIGURED state")
                    print (result)
                    return False
        else:
            print ("Couldn't transition to UNCONFIGURED State")
            return False
        return True
    
    def test_inactive_state(self):
        '''
        Changes the node to INACTIVE state and check for the pub, sub & srv to be inactive
        '''
        if(self.lm_client.go_to_state(State.INACTIVE)):
            self._collect_node_info()
            #Check that all the expected node's publishers are present in the INACTIVE state
            for pub in self.npub:
                result = [x[0] for x in self._pub if (x[0]==pub)]    #Compare the publisher name ignore the msg type
                if (result == []):
                    print ("The publisher topic is not-registered in INACTIVE state")
                    print (pub)
                    return False
            #if all the publisher are existing for the node verify the rate to be 0
            for hz in self._hz:
                if(hz != 0):
                    print ("The publisher topic is being published in INACTIVE state")
                    return False
            #Check that all the expected node's subscribers are subscribed to in the INACTIVE state
            for sub in self.nsub:
                result = [x[0] for x in self._sub if (x[0]==sub)]    #Compare the subscriber name ignore the msg type
                if (result == []):
                    print ("The subscriber topic is not-registered in INACTIVE state")
                    print (sub)
                    return False
            #Check that all the expected node's services are subscribed to in the INACTIVE state
            for srv in self.nsrv:
                result = [x for x in self._srv if (x==srv)]
                if (result == []):
                    print ("The service is not-registered in INACTIVE state")
                    print (srv)
                    return False
        else:
            print ("Couldn't transition to INACTIVE State")
            return False
        return True
        
    def test_active_state(self):
        '''
        Changes the node to ACTIVE state and check for the pub, sub & srv to be active
        '''
        if(self.lm_client.go_to_state(State.ACTIVE)):
            self._collect_node_info()
            #Check that all the expected node's publishers are present in the ACTIVE state
            for pub in self.npub:
                result = [x[0] for x in self._pub if (x[0]==pub)]    #Compare the publisher name ignore the msg type
                if (result == []):
                    print ("The publisher topic is not-registered in ACTIVE state")
                    print (pub)
                    return False
            #if all the publisher are existing for the node verify the rate to be as expected
            for i, hz in enumerate(self._hz):
                if((isclose(hz,self.nhz[i], 0.1)) == False): #Check isclose within 10% tolerance
                    print ("The rate of publishing doesn't match")
                    return False
            #Check that all the expected node's subscribers are subscribed to in the ACTIVE state
            for sub in self.nsub:
                result = [x[0] for x in self._sub if (x[0]==sub)]    #Compare the subscriber name ignore the msg type
                if (result == []):
                    print ("The subscriber topic is not-registered in ACTIVE state")
                    print (sub)
                    return False
            #Check that all the expected node's services are subscribed to in the ACTIVE state
            for srv in self.nsrv:
                result = [x for x in self._srv if (x==srv)]
                if (result == []):
                    print ("The service is not-registered in ACTIVE state")
                    print (srv)
                    return False
        else:
            print ("Couldn't transition to ACTIVE State")
            return False
        return True
        
    def test_finalized_state(self):
        '''
        Changes the node to FINALIZED state and check for the pub, sub & srv to be non-existent
        '''
        if(self.lm_client.go_to_state(State.FINALIZED)):
            self._collect_node_info()
            #Check that none of the expected node's publishers are not present in the FINALIZED state
            for pub in self.npub:
                result = [x[0] for x in self._pub if (x[0]==pub)]    #Compare the publisher name ignore the msg type
                if (result != []):
                    print ("The publisher topic is registered in FINALIZED state")
                    print (result)
                    return False
            #Check that none of the expected node's subscribers are not subscribed to in the FINALIZED state
            for sub in self.nsub:
                result = [x[0] for x in self._sub if (x[0]==sub)]    #Compare the subscriber name ignore the msg type
                if (result != []):
                    print ("The subscriber topic is registered in FINALIZED state")
                    print (result)
                    return False
            #Check that none the expected node's services are not subscribed to in the FINALIZED state
            for srv in self.nsrv:
                result = [x for x in self._srv if (x==srv)]
                if (result != []):
                    print ("The service is registered in FINALIZED state")
                    print (result)
                    return False
        else:
            print ("Couldn't transition to FINALIZED State")
            return False
        
        
    def test_all_states(self):
        '''
        tests the behaviour of the node in each and every state
        '''
        print ('Test UNCONFIGURED')
        result = self.test_unconfigured_state()
        if (result == False):
            return False
            
        print ('Test INACTIVE')
        result = self.test_inactive_state()
        if (result == False):
            return False
            
        print ('Test ACTIVE')
        result = self.test_active_state()
        if (result == False):
            return False
            
        print ('Test FINALIZED')
        result = self.test_finalized_state()
        if (result == False):
            return False
            
        #All the above tests have passed so return true
        return True