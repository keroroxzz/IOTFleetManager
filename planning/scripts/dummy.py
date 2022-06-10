#! /usr/bin/env python3

import sys
import rospy
import numpy as np
from std_msgs.msg import String

class DummyTarget():
    def __init__(self, target_name):
        self.target = target_name
        rospy.Subscriber(f'{self.target}/action', String, callback=self.action_cb, queue_size=10)
        self.status_pub = rospy.Publisher('manager/update', String, queue_size=10)
        self.action = 'STOP'

    def action_cb(self, msg: String):
        self.action = msg.data
        rospy.loginfo(f'Receive an action {self.action}.')
        rospy.sleep(1)
        rospy.loginfo(f'Action done.')
        self.status_pub.publish(String(data=f'{self.target}/DONE'))

if __name__ == "__main__":

    name = sys.argv[1]
    rospy.init_node(f'Dummy_{name}')
    dummy = DummyTarget(target_name=name)
    rospy.spin()
