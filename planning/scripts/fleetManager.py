#! /usr/bin/env python3
'''
IOT AGV Fleet project - Fleet Manager

Author: RTU
Data: 2022/06/04
Ver: 1.0

Description:
    This script is resposibile for fleet management, e.g., fleet tracking, receiving and control task for each of the members.
'''

from os import stat
import numpy as np
from fleet_msgs.msg import Path, Status, Point, FleetStatus, Heading
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
        # self.waitlist = []

    def __p2np(self, p: Point):
        return np.asarray((p.x, p.y, p.z), dtype=int)

    def __getAction(self, status: Status):

        if status.next.id == status.current.id:
            if status.current.id == 0:
                return 'STRAIGHT'
            else:
                return ''

        if status.next.id+1>=len(self.plans[status.target].path) and status.next.id != status.current.id:
            return 'STOP'

        new_next = self.plans[status.target].path[status.next.id+1]
        
        next = self.__p2np(new_next)
        now = self.__p2np(status.next)
        prev = self.__p2np(status.current)

        action = np.cross((now-prev), (next-now))[2]
        action = {-1:'LEFT', 0:'STRAIGHT', 1:'RIGHT'}[action]

        return action

    # def __getNextStatus(self, status: Status):
    #     new_status = Status() 
    #     new_status = status
    #     if status.next.id+1>=len(self.plans[status.target].path):
    #         new_status.arg='STOP'
    #         return new_status
    #     new_status.current = status.next
    #     new_status.next = self.plans[status.target].path[status.next.id+1]
    #     return new_status

    def __moveStatus(self, status: Status):

        new_id = min(status.next.id+1, len(self.plans[status.target].path)-1)

        status.current = status.next
        status.next = self.plans[status.target].path[new_id]

        new_heading = Heading()
        new_heading.used = True
        new_heading.x = status.next.x - status.current.x
        new_heading.y = status.next.y - status.current.y

        # If the next node is not the same to the current one, update the heading
        if not (new_heading.x==0 and new_heading.y)==0:
            status.heading=new_heading

    def __isTargetExist(self, target):
        return target in self.action_pubs.keys()

    def __add_target(self, target: String):
        self.action_pubs[target] = rospy.Publisher(f'{target}/action', String, queue_size=10)
        rospy.loginfo(f'Add a target {target} to the action publisher list.')

        status = Status()
        status.target = target
        status.heading = Heading()
        status.heading.used = False
        self.status[target] = status

    def __plan_cb(self, msg: Path):
        target = msg.target

        if not self.__isTargetExist(target):
            self.__add_target(target)
        
        self.plans[target] = msg
        self.status[target].current = msg.path[0]
        self.status[target].next = msg.path[0]
        rospy.loginfo(f'Update the plan of {target}.')

        action=self.__getAction(self.status[target])
        self.action_pubs[target].publish(String(data=action))
        rospy.loginfo(f'Assign the first action {action} to {target}.')

        self.__moveStatus(self.status[target])
        action=self.__getAction(self.status[target])
        self.action_pubs[target].publish(String(data=action))
        rospy.loginfo(f'Assign the first action {action} to {target}.')

    def __pub_fleet(self):
        fleet = FleetStatus(fleet=[v for v in self.status.values()])
        self.fleet_pub.publish(fleet)

    # def __check_wait(self):
    #     while len(self.waitlist)>0:
    #         waiter = self.waitlist.pop()
    #         target = waiter.target
    #         if self.solveConfliction(waiter):
    #             action = self.__getAction(waiter)
    #             self.action_pubs[target].publish(String(data=action))
    #             rospy.loginfo(f'Conflict solved! Assign the next action {action} to target {target}.')

    def __is_all_ready(self):
        id=None
        running_targets = [status for status in self.status.values() if status.current.id!=status.next.id]
        running_ids = [status.current.id for status in running_targets]
        for i in range(1,len(running_ids)):
            if running_ids[i] != running_ids[i-1]:
                return False
        return True

    def __move_all(self):
        for s in self.status.values():
            target = s.target
            action = self.__getAction(s)
            if action!='':
                self.action_pubs[target].publish(String(data=action))
                rospy.loginfo(f'Assign the next action {action} to {target}.')

    def __status_cb(self, msg: String):
        target, data = msg.data.split('/')
        if data == 'DONE':

            status = self.status[target]
            self.__moveStatus(status)

            if self.__is_all_ready():
                self.__move_all()
            # if self.solveConfliction(status):
            #     action = self.__getAction(status)
            #     self.action_pubs[target].publish(String(data=action))
            #     rospy.loginfo(f'Assign the next action {action} to {target}.')

            # self.__check_wait()

        self.__pub_fleet()

    # def at(self, a: Point, b: Point):
    #     return a.x==b.x and a.y==b.y

    # def isConflict(self, a: Status, b: Status):
    #     if a.arg == 'STOP' and b.arg == 'STOP':
    #         return False
    #     elif (not a.arg == 'STOP') and (not b.arg == 'STOP'):
    #         if self.at(a.next, b.next):
    #             return True
    #     elif not a.arg == 'STOP':
    #         b_ = self.__getNextStatus(b)
    #         if b_.arg == 'STOP':
    #             return False
    #         if (self.at(a.current, b_.current) and self.at(a.next, b_.next)) or\
    #             (self.at(a.current, b_.next) and self.at(a.next, b_.current)):
    #             return True
    #     elif not b.arg == 'STOP':
    #         a_ = self.__getNextStatus(a)
    #         if a_.arg == 'STOP':
    #             return False
    #         if (self.at(a_.current, b.current) and self.at(a_.next, b.next)) or\
    #             (self.at(a_.current, b.next) and self.at(a_.next, b.current)):
    #             return True
    #     return False

    # def solveConfliction(self, a: Status):
    #     a.arg=''
    #     is_conflict=False
    #     for b in self.status.values():
    #         if b is not a:
    #             is_conflict = is_conflict or self.isConflict(a,b)

    #     if is_conflict:
    #         is_conflict=False
    #         a.arg='STOP'
    #         self.waitlist.append(a)
    #         for b in self.status.values():
    #             if b is not a:
    #                 is_conflict = is_conflict or self.isConflict(a,b)

    #         if is_conflict:
    #             rospy.logfatal('Cannot solve the conflict!')
    #         else:
    #             rospy.logerr(f'Confliction founded! Stop {a.target}')
    #         return False
    #     else:
    #         return True

            
if __name__ == '__main__':
    manager = FleetManager('FleetManager')
    rospy.spin()



