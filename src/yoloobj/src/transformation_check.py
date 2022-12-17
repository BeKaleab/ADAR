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

FIRST_TIME = True
TF_BUFFER = None
TF_LISTENER = None

def transformation(base, target):
    """
    This is just a backup file to check transformations directly if issues with the direct method in the pupeline pop up(not integrated in the pipline)
    """
    TF_BUFFER = tf2_ros.Buffer()
    TF_LISTENER = tf2_ros.TransformListener(TF_BUFFER)
    AR_TAG = ['ar_marker_0','ar_marker_5']
    goals = []
    for i in range(len(target)):
      try:
        trans = TF_BUFFER.lookup_transform(base, target[i], rospy.Time(),rospy.Duration(0.5))
        print(trans)
        trans = trans.transform
        translation = [trans.translation.x, trans.translation.y, trans.translation.z]
        q = [trans.rotation.w, trans.rotation.x, trans.rotation.y, trans.rotation.z]
        # rot_mat = np.array([[1 - 2*(q[2]**2 + q[3] ** 2), 2*(q[1]*q[2] - q[0]*q[3]), 2*(q[1]*q[3] + q[0]*q[2])],
        #                                     [2*(q[1]*q[2] + q[0]*q[3]), 1 - 2*(q[1]**2 + q[3]**2), 2*(q[2]*q[3] + q[0]*q[1])],
        #                                     [2*(q[1]*q[3] + q[0]*q[2]), 2*(q[2]*q[3] + q[0]*q[1]), 1 - 2*(q[1]**2 + q[2]**2)]])
        rot_mat = np.array([[2*(q[0]**2 + q[1]**2) - 1, 2*(q[1]*q[2] - q[0]*q[3]), 2*(q[1]*q[3] + q[0]*q[2])],
                                            [2*(q[1]*q[2] + q[0]*q[3]), 2*(q[0]**2 + q[2]**2)-1, 2*(q[2]*q[3] + q[0]*q[1])],
                                            [2*(q[1]*q[3] + q[0]*q[2]), 2*(q[2]*q[3] + q[0]*q[1]), 2*(q[0]**2 + q[3]**2)-1]])
        p = [-0.06478093564510345, 0.213662251830101, 0.506777822971344]
        rotation_comp = np.matmul(rot_mat, np.array(p).T)
        translation_comp = np.array(translation)
        points = rotation_comp + translation_comp
        print('rot_mat\n', rot_mat)
        # print('p\n', np.array(p))
        print('rotation\n', rotation_comp)
        print('translation\n', translation_comp)
        print('points\n', points)
        # print('points\n', points)
        points = points.T #+ [0.2446, -1.1947, -1.2629]
        pointX = points[0]
        pointY = points[1]
        pointZ = points[2]
        points = [pointX, pointY, pointZ]
        goals.append(points)
        # print('goals\n', goals)

        
      except (tf2_ros.LookupException, tf2_ros.ConnectivityException) as e:
        print(e)
        pass
    return goals


if __name__ == '__main__':
  # Check if the node has received a signal to shut down
  # If not, run the talker method

  #Run this program as a new node in the ROS computation graph 
  #called /turtlebot_controller.
  rospy.init_node('transformation', anonymous=False)

  try:
    AR_TAG = ['ar_marker_12']
    base = 'camera_link'
    #base = 'ar_marker_5'
    transformation(base,AR_TAG)
  except rospy.ROSInterruptException:
    pass