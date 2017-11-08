#!/usr/bin/env python

import rospy

from lifecycle_msgs.msg import lm_events
from lifecycle_msgs.msg import Lifecycle
from lifecycle.lifecycle_model import LifecycleModel

class LmMonitor(object):
    """
    :class:`LmMonitor` collects the "/lm_monitor" and stores them in a dictionary for further use
    """
    
    def __init__(self):
        super(LmMonitor,self).__init__()
        self.LmEventsBuffer = {}
       
    def setLmEvent(self, lm_monitor_msg, time_stamp):
        """
        :param lm_monitor_msg: the lm_monitor message with node_name and lifecycle_event
        """
        lmevent_dict = self.lifecycleMsg2Dict(time_stamp, lm_monitor_msg.lifecycle_event)
        try:
            self.LmEventsBuffer[lm_monitor_msg.node_name] = lmevent_dict 
        except KeyError:
            self.LmEventsBuffer[lm_monitor_msg.node_name] = lmevent_dict #first message from this node
        
    def lifecycleMsg2Dict(self, time_stamp, lifecycle_msg):
        """
        :brief: Packs the lifecycle message into dictionary in the format:
        {time_stamp : ros_time, transition : "Configure", end_state : "INACTIVE", result_Code : "Success"}
        :param: time_stamp of type ros time
        :param: lifecycle_msg of type lifecycle_msgs/Lifecycle
        """
        dict = {}
        dict["time_stamp"]      = time_stamp
        dict["transition"]      = LifecycleModel.STATE_TO_STR[lifecycle_msg.transition]
        dict["end_state"]       = LifecycleModel.STATE_TO_STR[lifecycle_msg.end_state]
        dict["result_code"]     = LifecycleModel.STATE_TO_STR[lifecycle_msg.result_code]

        return dict

        
        
if __name__ == '__main__':
    rospy.init_node('Lifecycle Monitor')
    
