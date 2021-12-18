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
        self.object = np.zeros(16, dtype=int)
        self.final_object_place = np.zeros(17,dtype=int)
        self.object_place = np.zeros(17,dtype=int)
        self.board_place = [(115,136,33,63),(150,162,39,69),(178,195,46,76),(212,222,40,84),(108,125,51,81),(140,157,59,92),(173,187,57,95),(211,225,64,102),(93,115,66,107),(128,147,83,116),(165,182,81,116),(202,217,91,132),(81,103,103,137),(116,136,111,144),(156,177,109,155),(195,209,113,162,(110,137,174,216))] #ボードの閾値 
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
                    self.object[0] = point
                    self.object_place[0] = box.class
                if box.Class == "b_l_s_1":  
                    self.object[1] = point
                    self.object_place[1] = box.class
                if box.Class == "b_l_c_0":  
                    self.object[2] = point
                    self.object_place[2] = box.class
                if box.Class == "b_l_c_1":  
                    self.object[3] = point
                    self.object_place[3] = box.class
                if box.Class == "b_s_s_0":  
                    self.object[4] = point
                    self.object_place[4] = box.class
                if box.Class == "b_s_s_1":
                    self.object[5] = point
                    self.object_place[5] = box.class
                if box.Class == "b_s_s_0":  
                    self.object[6] = point
                    self.object_place[6] = box.class
                if box.Class == "b_s_c_1":  
                    self.object[7] = point
                    self.object_place[7] = box.class
                if box.Class == "w_l_s_0":  
                    self.object[8] = point
                    self.object_place[8] = box.class
                if box.Class == "w_l_s_1":  
                    self.object[9] = point
                    self.object_place[9] = box.class
                if box.Class == "w_l_c_0":  
                    self.object[10] = point
                    self.object_place[10] = box.class
                if box.Class == "w_l_c_1":  
                    self.object[11] = point
                    self.object_place[11] = box.class
                if box.Class == "w_s_s_0":  
                    self.object[12] = point
                    self.object_place[12] = box.class
                if box.Class == "w_s_s_1":  
                    self.object[13] = point
                    self.object_place[13] = box.class
                if box.Class == "w_s_s_0":  
                    self.object[14] = point
                    self.object_place[14] = box.class
                if box.Class == "w_s_c_1": # white_short_circle_hole 
                    self.object[15] = point
                    self.object_place[15] = box.class
                else:  # false (recognized white_line in boundingboxes)
                    point = False
        else:  # false (recognized objects inboundingboxes)
            point = False
        return self.object

    def quarto_recognition(self):
        for i in range(16): 
            for j in range(17):
                if self.board_place[j][0] < self.object[i][0] < self.board_place[j][1] and self.board_place[j][2] < self.object[i][1] < self.board_place[j][3]:
                    self.final_object_place[j] = self.object_place[i]

        self.yolo_pub.publish(self.final_object_place)


if __name__ == '__main__':
    ot = ObjectTracker()
    DURATION = 0.2
    r = rospy.Rate(1/DURATION)
    rospy.Subscriber("/detected_objects_in_image", BoundingBoxes, ot.main_callback, queue_size=1)
    rospy.spin()
