#!/usr/bin/env python3
# coding: utf-8


import rospy
import cv2
import math
import numpy as np
import copy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger
from sensor_msgs.msg import LaserScan
from yolov5_pytorch_ros.msg import BoundingBox, BoundingBoxes
from std_msgs.msg import Bool
from std_msgs.msg import String



class ObjectTracker():
    def __init__(self):
        rospy.init_node('object_tracking', anonymous=True)
        self._cv_bridge = CvBridge()
        self._point_of_centroid = None
        self.object_place = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
        self.final_object_place = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
        self.object_name = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
        self.board_place = [(x_min,x_max,y_min,y_max),(),()......] 
	self.yolo_pub = rospy.Publisher("yolo_result", String, queue_size=10)


    def _calculate_centroid_point(self, msg):
        point = False
        # initialization
        box_xmin = 0
        box_xmax = 0
        box_ymin = 0
        box_ymax = 0
        probability = 0
        (x_center, y_center) = (0, 0)

        if(msg.bounding_boxes[0].Class != "None"):  # true (recognized objects in boundingboxes)
            for box in msg.bounding_boxes:
                point = True
                print(box)
                box_xmin = float(box.xmin)
                box_xmax = float(box.xmax)
                box_ymin = float(box.ymin)
                box_ymax = float(box.ymax)
                probability = box.probability
                (x_center, y_center) = ((box_xmin + box_xmax)//2, (box_ymin + box_ymax)//2)
                point = (x_center, y_center)
                print(point)
                if box.Class == "b_l_s_0": # black_long_square_no hole
                    self.object_place[0] = point
                    self.object_name[0] = box.class
                if box.Class == "b_l_s_1":  
                    self.object_place[1] = point
                    self.object_name[1] = box.class
                if box.Class == "b_l_c_0":  
                    self.object_place[2] = point
                    self.object_name[2] = box.class
                if box.Class == "b_l_c_1":  
                    self.object_place[3] = point
                    self.object_name[3] = box.class
                if box.Class == "b_s_s_0":  
                    self.object_place[4] = point
                    self.object_name[4] = box.class
                if box.Class == "b_s_s_1":
                    self.object_place[5] = point
                    self.object_name[5] = box.class
                if box.Class == "b_s_s_0":  
                    self.object_place[6] = point
                    self.object_name[6] = box.class
                if box.Class == "b_s_c_1":  
                    self.object_place[7] = point
                    self.object_name[7] = box.class
                if box.Class == "w_l_s_0":  
                    self.object_place[8] = point
                    self.object_name[8] = box.class
                if box.Class == "w_l_s_1":  
                    self.object_place[9] = point
                    self.object_name[9] = box.class
                if box.Class == "w_l_c_0":  
                    self.object_place[10] = point
                    self.object_name[10] = box.class
                if box.Class == "w_l_c_1":  
                    self.object_place[11] = point
                    self.object_name[11] = box.class
                if box.Class == "w_s_s_0":  
                    self.object_place[12] = point
                    self.object_name[12] = box.class
                if box.Class == "w_s_s_1":  
                    self.object_place[13] = point
                    self.object_name[13] = box.class
                if box.Class == "w_s_s_0":  
                    self.object_place[14] = point
                    self.object_name[14] = box.class
                if box.Class == "w_s_c_1": # white_short_circle_hole 
                    self.object_place[15] = point
                    self.object_name[15] = box.class
                else:  # false (recognized white_line in boundingboxes)
                    point = False
        else:  # false (recognized objects inboundingboxes)
            point = False
        return self.object_place

    def quarto_recognition(self):
        for i in range(16): #object
            for j in range(17):
                if self.board_place[j][0] < self.object_place[i][0] < self.board_place[j][1] and self.board_place[j][2] < self.object_place[i][1] < self.board_place[j][3]:
                    self.final_object_place[j] = self.object_name[i]

        self.yolo_pub.publish(self.final_object_place)


if __name__ == '__main__':
    ot = ObjectTracker()
    DURATION = 0.2
    r = rospy.Rate(1/DURATION)
    rospy.Subscriber("/detected_objects_in_image", BoundingBoxes, ot.main_callback, queue_size=1)
    rospy.spin()
