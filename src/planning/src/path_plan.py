#!/usr/bin/env python
import sys
from intera_interface import Limb
import rospy
import numpy as np
import traceback

from moveit_msgs.msg import OrientationConstraint
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

from path_planner import PathPlanner
from intera_interface import gripper as robot_gripper
import moveit_commander as mc


try:
    from controller import Controller
except ImportError:
    pass

import argparse

import intera_interface
import intera_external_devices

from intera_interface import CHECK_VERSION
import multiprocessing as mp
from geometry_msgs.msg import Point, PointStamped


slot = 0.0

def main():
    """
    Main path planning script, in here we listen for inputs as transfromed points in our robots frame 
    coming from the object detection and depth calculations done from depth_csv which acts as a publisher.
    """
    #Slot for a given dish
    global slot

    #Create planner 
    planner = PathPlanner("right_arm")

    #Make a smooth and accurate PID controller
    Kp = 2 * np.array([0.4, 2, 1.7, 1.5, 2, 2, 3])
    Kd = 0.01 * np.array([2, 1, 2, 0.5, 0.8, 0.8, 0.8])
    Ki = 0.1 * np.array([1.4, 1.4, 1.4, 1, 0.6, 0.6, 0.6])
    Kw = np.array([0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9])

    #Set up robot limb, controller, gripper, and joints
    limb = intera_interface.Limb("right")
    controller =  Controller(Kp,Kd,Ki,Kw,limb)
    right_gripper = robot_gripper.Gripper('right_gripper')
    joints = limb.joint_names()
    
    #Set up offsets values for different 
    xaoff,yaoff,zaoff = 0.02,0.0,0.0
    xboff,yboff,zboff = 0.04,+0.28,0.13
    xcoff,ycoff,zcoff = -0.02,0.03,0.04
    
    
    #Add orientation constraints and the obstacle for planner depending on workspace settup
    #Create a path constraint for the chosen arm
    orien_const = OrientationConstraint()
    orien_const.link_name = "right_gripper"
    orien_const.header.frame_id = "base"
    orien_const.orientation.y = -1.0
    orien_const.absolute_x_axis_tolerance = 0.1
    orien_const.absolute_y_axis_tolerance = 0.1
    orien_const.absolute_z_axis_tolerance = 0.1
    orien_const.weight = 1.0
    orien_const = [orien_const]
    
    #Add different workspace constraints if needed
    pose = PoseStamped()
    pose.header.frame_id = "base"
    pose.pose.position.x = 0.5
    pose.pose.position.y = 0
    pose.pose.position.z = 0
    pose.pose.orientation.x = 0.0
    pose.pose.orientation.y = 0.0
    pose.pose.orientation.z = 0.0
    pose.pose.orientation.w = 1.0
    #planner.add_box_obstacle(np.array([0.4,1.2,0.1]), "Wall", pose)
    
    bottom = np.array([0.2*1.25, 0.41*1.25, 0.05])
    name = 'box'
    pose1 = PoseStamped()
    pose1.header.frame_id = 'base'
    pose1.pose.position.x = 0.710 #0.746
    pose1.pose.position.y = -0.478 #0.167
    pose1.pose.position.z =  -0.120 #-0.133
    pose1.pose.orientation.x = 0.0
    pose1.pose.orientation.y = 0.0
    pose1.pose.orientation.z = 0.0
    pose1.pose.orientation.w = 0.0
    #planner.add_box_obstacle(bottom, name, pose1)

    size2 = np.array([0.1, 1.0, 3.0])
    name2 = 'wall'
    pose2 = PoseStamped()
    pose2.header.frame_id = 'base'
    pose2.pose.position.x = -0.6
    pose2.pose.position.y = 0.0
    pose2.pose.position.z = 0.0
    pose2.pose.orientation.x = 0.0
    pose2.pose.orientation.y = 0.0
    pose2.pose.orientation.z = 0.0
    pose2.pose.orientation.w = 0.0
    #planner.add_box_obstacle(size2, name2, pose2)


    #Method to move to a given state, with the option to set a desired final orientation and/or orientation constraints
    def move_to_state(x,y,z,o_const=[],ox=0.0,oy=-1.0,oz=0.0,ow=0.0):
        while not rospy.is_shutdown():
            try:
                goal = PoseStamped()
                goal.header.frame_id = "base"

                #x, y, and z position
                goal.pose.position.x = x
                goal.pose.position.y = y
                goal.pose.position.z = z

                #Orientation as a quaternion
                goal.pose.orientation.x = ox
                goal.pose.orientation.y = oy
                goal.pose.orientation.z = oz
                goal.pose.orientation.w = ow

                # Might have to edit this . . . 
                plan = planner.plan_to_pose(goal, o_const)
                for i in range(50):
                    p = planner.plan_to_pose(goal, o_const) 
                    if (len(plan[1].joint_trajectory.points) > len(p[1].joint_trajectory.points)):
                        plan = p
                #input("Press <Enter> to move the right arm to goal pose 1: ")
                if not controller.execute_plan(plan[1],log=False): 
                    raise Exception("Execution failed")
            except Exception as e:
                print(e)
                traceback.print_exc()
            else:
                break

    #Gripper methods
    def gcalibrate():
        # Calibrate the gripper (other commands won't work unless you do this first)
        print('Calibrating...')
        right_gripper.calibrate()
        rospy.sleep(2.0)
    def gclose():
        # Close the right gripper
        print('Closing...')
        right_gripper.close()
        rospy.sleep(1.0)
    def gopen():
        # # Open the right gripper
        print('Opening...')
        right_gripper.open()
        rospy.sleep(1.0)
        print('Done!')

    #FK methods to set joint value directly or by adding a delta to current joint
    def set_j(limb, joint_name, delta):
        current_position = limb.joint_angle(joint_name)
        joint_command = {joint_name: delta}
        print("Executing" + str(joint_command))
        limb.set_joint_position_speed(0.05)
        limb.set_joint_positions(joint_command)
        print(joint_name)
        print(current_position)

    def set_j_d(limb, joint_name, delta):
        current_position = limb.joint_angle(joint_name)
        joint_command = {joint_name: current_position + delta}
        print("Executing" + str(joint_command))
        limb.set_joint_position_speed(0.05)
        limb.set_joint_positions(joint_command)
        print(joint_name)
        print(current_position)


    #Pick and place methods for main objects
    #1)points from yolo are taken in
    #2)we add offests to accomidate the distance from our transform point and the base of the robot as well a safe hover hight
    #3)we drop down and grab the object
    #4)pick the object up (and if needed reoriant in the air)
    #5)if reoriantation needed(row2/r2/bottom rack) drop down and then go to the dropoff point else go directly to drop off point
    #6)hold desired constraints during dropoff and after drop, open gripper to drop, pull back if needed and go back to base point to grab again
    def pick_and_place_cup(message):
        global slot
        #print(move_group.MoveGroupCommander.get_jacobian_matrix (move_group.MoveGroupCommander.get_current_joint_values()))
        #print(message)
        gcalibrate()
        x,y,z = message.x+xcoff,message.y+ycoff,message.z+zcoff
        move_to_state(x, y, z, [])
        move_to_state(x, y, z-0.08, [])
        gclose()
        move_to_state(0.691, 0.159, 0.385, [])
        move_to_state(0.691, 0.159, 0.385, [],0.7071068, 0.0, 0.0, 0.7071068)
        move_to_state(0.620+(slot*.125), -0.530, 0.165, [],0.7071068, 0.0, 0.0, 0.7071068)
        move_to_state(0.620+(slot*.125), -0.530, 0.165, [], 0.8167701, 0.0079977, 0.0089975, 0.5768376)
        gopen()
        move_to_state(0.691, 0.159, 0.385, [])
        slot = slot + 1.0

    def pick_and_place_cup_r2(message):
        global slot
        #print(move_group.MoveGroupCommander.get_jacobian_matrix (move_group.MoveGroupCommander.get_current_joint_values()))
        #print(message)
        gcalibrate()
        x,y,z = message.x+xcoff,message.y+ycoff,message.z+zcoff
        move_to_state(x, y, z, [])
        move_to_state(x, y, z-0.08, [])
        gclose()
        move_to_state(0.691, 0.159, 0.385, [])
        move_to_state(0.691, 0.159, 0.047, [],0.7071068, 0.0, 0.0, 0.7071068)
        move_to_state(0.620+(slot*.125), -0.530, -0.04, [],0.7071068, 0.0, 0.0, 0.7071068)
        move_to_state(0.620+(slot*.125), -0.530, -0.04, [], 0.8167701, 0.0079977, 0.0089975, 0.5768376)
        #set_j(limb, joints[4], 0.25)
        gopen()
        move_to_state(0.619, -0.356, 0.026, [], 0.8167701, 0.0079977, 0.0089975, 0.5768376)
        move_to_state(0.691, 0.159, 0.385, [])
        slot = slot + 1.0


    def pick_and_place_bowl_plate(message):
        global slot
        #print(move_group.MoveGroupCommander.get_jacobian_matrix (move_group.MoveGroupCommander.get_current_joint_values()))
        #print(message)
        gcalibrate()
        x,y,z = message.x+xboff,message.y+yboff,message.z+zboff
        move_to_state(x, y, z, [])
        move_to_state(x, y, z-0.12, [])
        gclose()
        move_to_state(0.691, 0.159, 0.385, [])
        move_to_state(0.620+(slot*.145), -0.530, 0.25, [])
        move_to_state(0.620+(slot*.145), -0.530, 0.165, [])
        gopen()
        move_to_state(0.620+(slot*.145), -0.530, 0.25, [])
        move_to_state(0.691, 0.159, 0.385, [])
        slot = slot + 1.0


    def pick_and_place_bowl_plate_r2(message):
        global slot
        #print(move_group.MoveGroupCommander.get_jacobian_matrix (move_group.MoveGroupCommander.get_current_joint_values()))
        #print(message)
        gcalibrate()
        x,y,z = message.x+xboff,message.y+yboff,message.z+zboff
        move_to_state(x, y, z, [])
        move_to_state(x, y, z-0.12, [])
        gclose()
        move_to_state(0.691, 0.159, 0.385, [])
        move_to_state(0.691, 0.159, 0.385, [],0.7071068, 0.0, 0.0, 0.7071068)
        move_to_state(0.620+(slot*.145), -0.530, 0.25, [],0.7071068, 0.0, 0.0, 0.7071068)
        move_to_state(0.620+(slot*.145), -0.530, 0.25, [], 0.8167701, 0.0079977, 0.0089975, 0.5768376)
        gopen()
        move_to_state(0.620+(slot*.145), -0.530, 0.25, [])
        move_to_state(0.691, 0.159, 0.385, [])
        slot = slot + 1.0


    def pick_and_place_utensil(message):
        global slot
        #print(move_group.MoveGroupCommander.get_jacobian_matrix (move_group.MoveGroupCommander.get_current_joint_values()))
        #print(message)
        gcalibrate()
        x,y,z = message.x+xaoff,message.y+yaoff,message.z+zaoff
        move_to_state(x, y, z, [])
        move_to_state(x, y, z-0.08, [])
        gclose()
        move_to_state(0.691, 0.159, 0.385, [])
        move_to_state(0.691, 0.159, 0.385, [],0.7071068, 0.0, 0.0, 0.7071068)
        move_to_state(0.620+(slot*.100), -0.530, 0.165, [],0.7071068, 0.0, 0.0, 0.7071068)
        move_to_state(0.620+(slot*.100), -0.530, 0.165, [], 0.8167701, 0.0079977, 0.0089975, 0.5768376)
        gopen()
        move_to_state(0.691, 0.159, 0.385, [])
        slot = slot + 1.0

        
    def pick_and_place_utensil_r2(message):
        global slot
        #print(move_group.MoveGroupCommander.get_jacobian_matrix (move_group.MoveGroupCommander.get_current_joint_values()))
        #print(message)
        gcalibrate()
        x,y,z = message.x+xcoff,message.y+ycoff,message.z+zcoff
        move_to_state(x, y, z, [])
        move_to_state(x, y, z-0.08, [])
        gclose()
        move_to_state(0.691, 0.159, 0.385, [])
        move_to_state(0.691, 0.159, 0.047, [],0.7071068, 0.0, 0.0, 0.7071068)
        move_to_state(0.620+(slot*.100), -0.530, -0.04, [],0.7071068, 0.0, 0.0, 0.7071068)
        move_to_state(0.620+(slot*.100), -0.530, -0.04, [], 0.8167701, 0.0079977, 0.0089975, 0.5768376)
        #set_j(limb, joints[4], 0.25)
        gopen()
        move_to_state(0.619, -0.356, 0.026, [], 0.8167701, 0.0079977, 0.0089975, 0.5768376)
        move_to_state(0.691, 0.159, 0.385, [])
        slot = slot + 1.0

    def runner(message):
        global slot
        ros_point = Point()
        ros_point.x = message.point.x
        ros_point.y = message.point.y
        ros_point.z = message.point.z
        if(message.header.frame_id.lower() == "cup"):
            if(slot <= 5):
                pick_and_place_cup(ros_point)
            else:
                pick_and_place_cup_r2(ros_point)
        if(message.header.frame_id.lower() == "bowl"):
            if(slot <= 5):
                pick_and_place_bowl_plate(ros_point)
            else:
                pick_and_place_bowl_plate_r2(ros_point)
        if(message.header.frame_id.lower() == "plate"):
            if(slot <= 5):
                pick_and_place_bowl_plate(ros_point)
            else:
                pick_and_place_bowl_plate_r2(ros_point)
        if(message.header.frame_id.lower() == "fork"):
            if(slot <= 5):
                pick_and_place_utensil(ros_point)
            else:
                pick_and_place_utensil_r2(ros_point)
        if(message.header.frame_id.lower() == "spoon"):
            if(slot <= 5):
                pick_and_place_utensil(ros_point)
            else:
                pick_and_place_utensil_r2(ros_point)
    #set up a subscriber to listen for points and send point in
    rospy.Subscriber("yolo_coords", PointStamped, runner)
    # Wait for messages to arrive on the subscribed topics, and exit the node
    # when it is killed with Ctrl+C
    rospy.spin()

if __name__ == '__main__':
    rospy.init_node('moveit_node')
    main()

