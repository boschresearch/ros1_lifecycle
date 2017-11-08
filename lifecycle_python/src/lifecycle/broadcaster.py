#!/usr/bin/env python

import rospy

from lifecycle_msgs.msg import lm_events
from lifecycle_msgs.msg import Lifecycle

class LmEventBroadcaster(object):
    """
    :class:`LmEventBroadcaster` is a convenient way to send Lifecycle event updates on the ``"/lm_events"`` message topic.
    """
    
    def __init__(self, component_fqn, queue_size=100):
        self.node_name = component_fqn
        self.pub_lm_monitor = rospy.Publisher("/lm_events", lm_events, queue_size=queue_size)
       
    def sendLmEvent(self, lifecycle_msg):
        """
        :param lifecycle_msg: the lifecycle message with transition, end_state and result code
        """
        lm_monitor_msg = lm_events()
        lm_monitor_msg.node_name = self.node_name
        lm_monitor_msg.lifecycle_event = Lifecycle()
        lm_monitor_msg.lifecycle_event = lifecycle_msg
        self.pub_lm_monitor.publish(lm_monitor_msg)
        

if __name__ == '__main__':
    rospy.init_node('LifecycleEventBroadcaster')
