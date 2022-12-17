#!/usr/bin/env python
import cv2
import time
import numpy as np

import rospy
import tf2_ros
import ros_numpy
import message_filters
import image_geometry
# from geometry_msgs import Point
from cv_bridge import CvBridge, CvBridgeError
# import YOLOxyzROS as yolo
import csv 

FIRST_TIME = True
TF_BUFFER = None
TF_LISTENER = None

def transformation(base, target):
    """
    Integrated transform method which save transformation output data as a CSV which is read in by the detection code(depth_csv.py)
    """
    #Set upp buffer and listners
    TF_BUFFER = tf2_ros.Buffer()
    TF_LISTENER = tf2_ros.TransformListener(TF_BUFFER)
    AR_TAG = ['ar_marker_0','ar_marker_5']
    goals = []
    #Go through given targets and produce transformation brom base to target
    for i in range(len(target)):
      try:
        #Set up a long listen avrege transformation which gets transfromations over a period of time and avreges the output
        #This accounts for minor movements (like table/camera shaking) which can skew one look transfromations
        translation = [0.0,0.0,0.0]
        q = [0.0,0.0,0.0,0.0]
        #generate a lot of transfromas
        rvalu = 400000
        for tftr in range(rvalu):
          trans = TF_BUFFER.lookup_transform(base, target[i], rospy.Time(),rospy.Duration(0.5))
          trans = trans.transform
          translation = [translation[0] + trans.translation.x, translation[1] + trans.translation.y, translation[2] + trans.translation.z]
          q = [q[0] + trans.rotation.x, q[1] + trans.rotation.y, q[2] + trans.rotation.z, q[3] + trans.rotation.w]
        #get aavrage
        q = [q[0]/rvalu, q[1]/rvalu, q[2]/rvalu, q[3]/rvalu]
        translation = [translation[0]/rvalu, translation[1]/rvalu, translation[2]/rvalu]
        rot_mat = np.array([[2*(q[0]**2 + q[1]**2) - 1, 2*(q[1]*q[2] - q[0]*q[3]), 2*(q[1]*q[3] + q[0]*q[2])],
                                            [2*(q[1]*q[2] + q[0]*q[3]), 2*(q[0]**2 + q[2]**2)-1, 2*(q[2]*q[3] + q[0]*q[1])],
                                            [2*(q[1]*q[3] + q[0]*q[2]), 2*(q[2]*q[3] + q[0]*q[1]), 2*(q[0]**2 + q[3]**2)-1]])
        #Transform test data
        p = [0.5087037682533264, 0.11962322890758514, 1.290000081062317]
        print(q)
        print(rot_mat,"\n\n\n")
        print(trans,"\n\n\n")
        print((rot_mat.tolist()[0]+rot_mat.tolist()[1]+rot_mat.tolist()[2]))
        print(translation)
        #Write transfrom vals to csv
        with open('AR_output.csv', 'w') as csvfile:
          csvwriter = csv.writer(csvfile)
          csvwriter.writerows([q])
          csvwriter.writerows([translation])
          #csvwriter.writerows([rot_mat.tolist()[0]+rot_mat.tolist()[1]+rot_mat.tolist()[2]])

        #---------------------Test---------------------------
        rotation_comp = np.matmul(rot_mat, np.array(p).T)
        translation_comp = np.array(translation)
        points = rotation_comp + translation_comp
        points = points.T 
        pointX = points[0]
        pointY = points[1]
        pointZ = points[2]
        points = [pointX, pointY, pointZ]
        goals.append(points)
        print('goals\n', goals)

        
      except (tf2_ros.LookupException, tf2_ros.ConnectivityException) as e:
        print(e)
        pass
    return goals


if __name__ == '__main__':

  #Init a node
  rospy.init_node('transformation', anonymous=False)
  #Set up and try a transformation
  try:
    AR_TAG = ['ar_marker_8']
    base = 'camera_link'
    # base = 'ar_marker_5'
    transformation(base,AR_TAG)
  except rospy.ROSInterruptException:
    pass