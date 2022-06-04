#! /usr/bin/env python

import cv2
import time
import rospy

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
cvb = CvBridge()

class CarVision():
  def __init__(self, node_name, topic_img):
    rospy.init_node(node_name, anonymous=True)
    
    self.sub_img = rospy.Subscriber(topic_img, Image, callback=self.img_cb, queue_size=1)

  def img_cb(msg):
    img = cvb.imgmsg_to_cv2(msg)
    # image processing

if __name__ == '__main__':
  
