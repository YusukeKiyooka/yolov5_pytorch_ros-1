#!/usr/bin/env python3
# coding: utf-8


import rospy
import cv2
import math
import numpy as np
import copy
import sys
from cv_bridge import CvBridge, CvBridgeError
from std_srvs.srv import Trigger
from yolov5_pytorch_ros.msg import BoundingBox, BoundingBoxes
from std_msgs.msg import Bool
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
from rocon_std_msgs.msg import StringArray


class ObjectTracker():
    def __init__(self):
        rospy.init_node('get_coordinates', anonymous=True)
        self._cv_bridge = CvBridge()
        self._point_of_centroid = None
        self.object = [(0, 0), (0, 0),(0, 0),(0, 0),(0, 0),(0, 0),(0, 0),(0, 0),(0, 0),(0, 0),(0, 0),(0, 0),(0, 0),(0, 0),(0, 0),(0, 0)]
        self.board_place = [(0,10000,0,100000),(150,162,39,69),(178,195,46,76),(212,222,40,84),(108,125,51,81),(140,157,59,92),(173,187,57,95),(211,225,64,102),(93,115,66,107),(128,147,83,116),(165,182,81,116),(202,217,91,132),(81,103,103,137),(116,136,111,144),(156,177,109,155),(195,209,113,162),(110,137,174,216)] #ボードの閾値
        self.object_place = []
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
        count = 0
        object_place = np.full(16,'None   ')
        final_object_place = np.full(17,'None   ')

        if(msg.bounding_boxes[0].Class != "None"):  # true (recognized objects in boundingboxes)
            for box in msg.bounding_boxes:
                point = True
                #print(box)
                box_xmin = float(box.xmin)
                box_xmax = float(box.xmax)
                box_ymin = float(box.ymin)
                box_ymax = float(box.ymax)
                probability = box.probability
                (x_center, y_center) = ((box_xmin + box_xmax)//2, (box_ymin + box_ymax)//2)
                point = (x_center, y_center)
                #print(point)
                if probability > 0.7:
                    if box.Class == "b_l_s_0": # black_long_square_no hole
                        self.object[0] = point
                        object_place[0] = box.Class 
                    if box.Class == "b_l_s_1":  
                        self.object[1] = point
                        object_place[1] = box.Class 
                    if box.Class == "b_l_c_0":  
                        self.object[2] = point
                        object_place[2] = box.Class 
                    if box.Class == "b_l_c_1":  
                        self.object[3] = point
                        object_place[3] = box.Class 
                    if box.Class == "b_s_s_0":  
                        self.object[4] = point
                        object_place[4] = box.Class 
                    if box.Class == "b_s_s_1":
                        self.object[5] = point
                        object_place[5] = box.Class 
                    if box.Class == "b_s_c_0":  
                        self.object[6] = point
                        object_place[6] = box.Class 
                    if box.Class == "b_s_c_1":  
                        self.object[7] = point
                        object_place[7] = box.Class 
                    if box.Class == "w_l_s_0":  
                        self.object[8] = point
                        object_place[8] = box.Class 
                    if box.Class == "w_l_s_1":  
                        self.object[9] = point
                        object_place[9] = box.Class 
                    if box.Class == "w_l_c_0":  
                        self.object[10] = point
                        object_place[10] = box.Class 
                    if box.Class == "w_l_c_1":  
                        self.object[11] = point
                        object_place[11] = box.Class 
                    if box.Class == "w_s_s_0":  
                        self.object[12] = point
                        object_place[12] = box.Class 
                    if box.Class == "w_s_s_1":  
                        self.object[13] = point
                        object_place[13] = box.Class 
                    if box.Class == "w_s_c_0":  
                        self.object[14] = point
                        object_place[14] = box.Class 
                    if box.Class == "w_s_c_1": # white_short_circle_hole 
                        self.object[15] = point
                        object_place[15] = box.Class 
                    else:  # false (recognized white_line in boundingboxes)
                        point = False
        else:  # false (recognized objects inboundingboxes)
            point = False
        #print(object_place)

        for i in range(16): 
            for j in range(17):
                #if type(self.object[i]) is int:
                    if self.board_place[j][0] < self.object[i][0] < self.board_place[j][1]:
                        if self.board_place[j][2] < self.object[i][1] < self.board_place[j][3]:
                            final_object_place[j] = object_place[i]
                        else:
                            pass
                    
                    else: 
                        pass
        print(final_object_place)
        test = String(data = final_object_place)
        self.yolo_pub.publish(test)
        #self.yolo_pub.publish(box.Class)


if __name__ == '__main__':
    ot = ObjectTracker()
    DURATION = 0.2
    r = rospy.Rate(1/DURATION)
    rospy.Subscriber("/detected_objects_in_image", BoundingBoxes, ot._calculate_centroid_point, queue_size=1)
    rospy.spin()
