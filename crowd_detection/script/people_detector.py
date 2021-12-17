#!/usr/bin/env python

# ROS modules
import rospy
import os
#from darknet_ros_msgs.msg import BoundingBoxes, BoundingBox, ObjectCount
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
from opencv_apps.msg import RectArrayStamped

# Standard library
import copy
import cv2
import time
import numpy as np
#from playsound import playsound
from geometry_msgs.msg import Point


class CrowdDetector(object):
    def __init__(self):
        self.pub = rospy.Publisher("crowd_information", String, queue_size=5)
        #self.human_detector_sub = rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, self.human_detector_callback)
        self.people_detect_sub = rospy.Subscriber("/people_detect/found", RectArrayStamped, self.people_detector_callback)
        self.point_pub = rospy.Publisher("/mitsu_point", Point, queue_size=10)

    def people_detector_callback(self, datas):
        #print(datas.rects[0].x)
        point = Point()
        point.x = datas.rects[0].x
        point.y = datas.rects[0].y
        point.z = 0
        self.point_pub.publish(point)

if __name__ == "__main__":
    rospy.init_node("SecurityCenter")
    c_detector = CrowdDetector()
    rospy.spin()
