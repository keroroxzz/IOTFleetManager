#! /usr/bin/env python3
'''
IOT AGV Fleet project - Fleet Manager

Author: RTU
Data: 2022/06/04
Ver: 1.0

Description:
    This script is resposibile for fleet management, e.g., fleet tracking, receiving and control task for each of the members.
'''

import numpy as np
from fleet_msgs.msg import Path, Status, Point, FleetStatus
from std_msgs.msg import String
import rospy


class FleetManager():
    def __init__(self, node_name):
        rospy.init_node(node_name, anonymous=True)

        rospy.Subscriber('/manager/plan', Path, callback=self.__plan_cb, queue_size=10)
        rospy.Subscriber('/manager/update', String, callback=self.__status_cb, queue_size=10)

        self.fleet_pub = rospy.Publisher('/console/fleet_status', FleetStatus, queue_size=10)
        self.action_pubs = {}

        self.status = {}
        self.plans = {}

    def __p2np(self, p: Point):
        return np.asarray((p.x, p.y, p.z), dtype=int)

    def __moveStatus(self, status: Status):

        if status.next.id+1>=len(self.plans[status.target].path):
            return 'STOP'

        new_next = self.plans[status.target].path[status.next.id+1]
        
        next = self.__p2np(new_next)
        now = self.__p2np(status.next)
        prev = self.__p2np(status.current)

        action = np.cross((now-prev), (next-now))[2]
        action = {-1:'LEFT', 0:'STRAIGHT', 1:'RIGHT'}[action]

        status.current = status.next
        status.next = new_next
        return action

    def __isTargetExist(self, target):
        return target in self.action_pubs.keys()

    def __add_target(self, target: String):
        self.action_pubs[target] = rospy.Publisher(f'{target}/action', String, queue_size=10)
        rospy.loginfo(f'Add a target {target} to the action publisher list.')

    def __plan_cb(self, msg: Path):
        target = msg.target

        if not self.__isTargetExist(target):
            self.__add_target(target)
        
        status = Status()
        status.current = msg.path[0]
        status.next = msg.path[1]
        status.target = target

        self.plans[target] = msg
        self.status[target] = status
        rospy.loginfo(f'Update the plan of {target}.')

        self.action_pubs[target].publish(String(data='STRAIGHT'))

    def __pub_fleet(self):
        fleet = FleetStatus(fleet=[v for v in self.status.values()])
        self.fleet_pub.publish(fleet)

    def __status_cb(self, msg: String):
        target, data = msg.data.split('/')
        if data == 'DONE':

            status = self.status[target]
            action = self.__moveStatus(status)
            self.action_pubs[target].publish(String(data=action))
            rospy.loginfo(f'Target {target} just finish the action. Assign the next action {action} to the car.')
        
        self.__pub_fleet()
            
            
if __name__ == '__main__':
    manager = FleetManager('FleetManager')
    rospy.spin()



