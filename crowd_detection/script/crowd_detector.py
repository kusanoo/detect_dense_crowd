#!/usr/bin/env python

# ROS modules
import rospy
import os
from darknet_ros_msgs.msg import BoundingBoxes, BoundingBox, ObjectCount
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

# Standard library
import copy
import cv2
import time
import numpy as np
#from playsound import playsound
from pygame import mixer


class CrowdDetector(object):
    def __init__(self):
        self.pub = rospy.Publisher("crowd_information", String, queue_size=5)
        self.human_detector_sub = rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, self.human_detector_callback)
        self.raw_img_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.img_receive_callback, queue_size = 1)
        self.bridge = CvBridge()
        self.sailen = "/home/kusa/catkin_ws/src/detect_dense_crowd/crowd_detection/script/yuriko_test/20210922_024553.mp3"
        self.mitsu = "/home/kusa/catkin_ws/src/detect_dense_crowd/crowd_detection/script/yuriko_test/20210922_023949.mp3"
        mixer.init()
        mixer.music.load(self.sailen)

    def img_receive_callback(self, img):
        self.img = self.bridge.imgmsg_to_cv2(img, desired_encoding="bgr8")

    def human_detector_callback(self, detection_datas):
        human_data = []

        for obj in detection_datas.bounding_boxes:
            if obj.Class == "person" and obj.probability >= 0.7:
                person_bounding_x = (obj.xmax - obj.xmin) # image pixel col position
                person_bounding_y = (obj.ymax - obj.ymin) # image pixel row position

                detected_human = {}
                #detected_human["position"] = (person_bounding_x/2, person_bounding_y/2)
                #detected_human["size"] = (person_bounding_x, person_bounding_y)
                detected_human["box"] = (obj.xmin, obj.xmax, obj.ymin, obj.ymax)

                human_data.append(copy.deepcopy(detected_human))

        self.crowd_data = []
        self.dense_crowd_detector(human_data)
        self.visualize_crowd()

    def dense_crowd_detector(self, human_data):
        overlap_thresh = 10

        while human_data:
            check_human = human_data.pop(0)
            for human in human_data:
                overlap_x = self.get_overlap(check_human["box"][:2], human["box"][:2])
                overlap_y = self.get_overlap(check_human["box"][2:], human["box"][2:])
                
                if overlap_x >= overlap_thresh and overlap_y >= overlap_thresh:
                    group = [copy.deepcopy(check_human), copy.deepcopy(human)]
                    if group not in self.crowd_data:
                        self.crowd_data.append(group)
            print("crowd:", len(self.crowd_data))
            #print self.crowd_data
            #print


    def get_overlap(self, a, b):
        return max(0, min(a[1], b[1]) - max(a[0], b[0]))

    def visualize_crowd(self):
        window_name = "Camera"
        color = (0,0,255)
        thickness = 2

        draw_img = copy.deepcopy(self.img)

        for person1, person2 in self.crowd_data:
            cv2.rectangle(draw_img, (person1["box"][0], person1["box"][2]), (person1["box"][1], person1["box"][3]), color, thickness)
            cv2.rectangle(draw_img, (person2["box"][0], person2["box"][2]), (person2["box"][1], person2["box"][3]), color, thickness)

        if self.crowd_data and not mixer.music.get_busy():
            mixer.music.play()
        cv2.imshow(window_name, draw_img)
        cv2.waitKey(1)


    
if __name__ == "__main__":
    rospy.init_node("SecurityCenter")
    c_detector = CrowdDetector()
    rospy.spin()
