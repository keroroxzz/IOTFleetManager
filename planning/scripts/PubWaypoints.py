#! /usr/bin/env python3
'''
IOT AGV Fleet project - Fake task publisher

Author: RTU
Data: 2022/06/04
Ver: 1.0
'''

import cv2
import numpy as np
from nav_msgs.msg import Path, MapMetaData
from geometry_msgs.msg import PoseStamped, Pose, Point
import rospy      

class PubWayNode():
  def __init__(self, node_name):
    rospy.init_node(node_name, anonymous=True)
    
    self.planner = None
    rospy.Publisher('map', MapMetaData, queue_size=10)
    rospy.Publisher('waypoints', Path, queue_size=10)

  def map_cb(self, msg):
      rospy.loginfo(f'Planner: receive a map with size:{(msg.width, msg.height)}')

      map = np.zeros((msg.width, msg.height), dtype=int)
      self.planner = AStar(map)

  def waypoint_cb(self, msg: Path):
      rospy.loginfo(f'Planner: receive a task with {len(msg.poses)} waypoints')

      path = self.planner.PathFromWaypoints(msg)
      self.path_pub.publish(path)
            
if __name__ == '__main__':
    PubWayNode('WaypointPublisher')
    rospy.spin()



