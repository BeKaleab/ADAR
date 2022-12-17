#!/usr/bin/env python
import pyrealsense2 as rs
import numpy as np
import cv2
import sys
from timeit import default_timer as timer
import cv2, math
from decimal import Decimal, ROUND_HALF_UP
import rospy
import csv
from geometry_msgs.msg import Point, PointStamped, TransformStamped
import tf2_ros
import tf2_geometry_msgs

# Initialize the parameters
confThreshold = 0.5
nmsThreshold = 0.4
inpWidth = 416
inpHeight = 416
scale = 1/255
classesFile = "coco.names"
modelConfiguration = "yolov3.cfg"
modelWeights = "yolov3.weights"
classes = None

# Setup pipline and config for streams
pipeline = rs.pipeline()
config = rs.config()

# Get device product line for setting a supporting resolution
pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)
device = pipeline_profile.get_device()
device_product_line = str(device.get_info(rs.camera_info.product_line))

#Set up camera
found_rgb = False
for s in device.sensors:
    if s.get_info(rs.camera_info.name) == 'RGB Camera':
        found_rgb = True
        break
if not found_rgb:
    print("The demo requires Depth camera with Color sensor")
    sys.exit()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
if device_product_line == 'L500':
    config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30)
else:
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

#Start streaming via the pipline
profile = pipeline.start(config)

# get camera depth sensor, depth scale, and intrinsics
depth_sensor = pipeline_profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()
intr = profile.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()  

# Set up allignment object for the depth frame to later be alligned to the color frame
align_to = rs.stream.color
align = rs.align(align_to)  
#get depth frame
dpt_frame = pipeline.wait_for_frames().get_depth_frame().as_depth_frame()

#For a given network get the layer names 
def getOutputsNames(net):
    layersNames = net.getLayerNames()
    return [layersNames[int(i)-1] for i in net.getUnconnectedOutLayers()]

#Set up the bounding boxes and draw them, find the avreged center point across a subsegment of the bounding box
#Smaller bounding box aims to be a simpler and less compute intensive mask (see MaskRCNN) 
#allowing us to avredge the depths across an object to get a point more representative of the objcts center that acounts for the geometry
#helps reduce error coming from hardware (gaps in depth point cloud) via avredging point and scans multiple point to get the most common reading
def drawPredicted(input):
    #Setup
    points = None
    classId, conf, left, top, right, bottom, frame,x ,y = input
    #get distance and depth reading for an inital point
    dist = dpt_frame.get_distance(x,y)
    depth = dist * profile.get_device().first_depth_sensor().get_depth_scale()
    Xtemp,Ytemp,Ztemp = rs.rs2_deproject_pixel_to_point(intr, [x,y], dist)
    #set up center coordinates
    center_coordinates_array = []
    center_coordinates_array.append([x,y])
    #set up smaller boundin box (sub box get points across)
    ln, tn, rn, bn = int((left+((left+right)/2))/2),int((top+((top+bottom)/2))/2), int((right+((left+right)/2))/2),int((bottom+((top+bottom)/2))/2)
    #draw
    cv2.rectangle(frame, (ln,tn), (rn,bn), (255,178,50),3)
    cv2.rectangle(frame, (left,top), (right,bottom), (255,178,50),3)
    cv2.circle(frame,(x,y),radius=1,color=(0,0,254), thickness=5)
    label = '%.2f' % conf
    #Set up and get classes and make labels
    if classes:
        assert(classId < len(classes))
        label = '%s' %(classes[classId])
    labelSize, baseLine = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
    top = max(top, labelSize[1])
    cv2.putText(frame, label,(left,top-5), cv2.FONT_HERSHEY_SIMPLEX,0.75,(255,255,0),2)
    distance_string = "Dist: " + str(round(dist,2)) + " m away"
    cv2.putText(frame,distance_string,(left,top+30), cv2.FONT_HERSHEY_SIMPLEX,0.75,(255,255,0),2)
    #set up final coordinates
    Finalcoordinates = []
    if (Xtemp != 0 and Ytemp != 0 and Ztemp != 0):
        #get transfrom data from csv
        rows = []
        with open('AR_output.csv', 'r') as csvfile:
            csvreader = csv.reader(csvfile)
            for row in csvreader:
                rows += row
        rows= [float(i) for i in rows]
        avg_dist = []
        #look through points from the smaller bounding box
        ln, tn, rn, bn = max(ln, 0),max(tn, 0),max(rn, 0),max(bn, 0) 
        for j in range(ln,rn):
            for k in range(tn,bn):
                #set up dictionary for most common points
                dist_block = {}
                tempDist = -1
                #read multiple times and pick the most common point 
                for d in range(9):
                    tempDist = dpt_frame.get_distance(j,k)
                    if tempDist in dist_block:
                        dist_block[tempDist] += 1
                    else:
                        dist_block[tempDist] = 1
                tempDistVal = -1
                for di in dist_block.items():
                    if di[1] > tempDistVal:
                        tempDistVal = di[0]
                #if a common point is found and is not a miss read(depth outside workspace) add it to the points that will be avredged
                if(tempDistVal != 0 and tempDistVal != None and tempDistVal <= 0.95):
                    avg_dist.append(tempDistVal)
        #go through the points genretated and avredge them
        if (len(avg_dist)> 0):
            out = 0
            for l in avg_dist:
                out += l
            Ztemp = out/len(avg_dist)
            Xtemp,Ytemp,Ztemp = rs.rs2_deproject_pixel_to_point(intr, [x,y], Ztemp)
        #Transfrom the avredge point
        p = [Xtemp,Ytemp,Ztemp]
        print(p)
        #rot_mat = np.array([[rows[0], rows[1], rows[2]],[rows[3], rows[4], rows[5]],[rows[6], rows[7], rows[8]]])
        tfs = TransformStamped()
        print(rows)
        tfs.transform.rotation.x = rows[0]
        tfs.transform.rotation.y = rows[1]
        tfs.transform.rotation.z = rows[2]
        tfs.transform.rotation.w = rows[3]
        tfs.transform.translation.x = rows[4]
        tfs.transform.translation.y = rows[5]
        tfs.transform.translation.z = rows[6]
        #rot_mat = np.array([[rows[0], rows[1], rows[2]],[rows[3], rows[4], rows[5]],[rows[6], rows[7], rows[8]]])
        #rotation_comp = np.matmul(rot_mat.T, np.array(p).T)
        #translation_comp = np.array([rows[9], rows[10], rows[11]])
        #points = rotation_comp + translation_comp
        #points = points.T
        #points = [points[0], -points[1], -points[2]]
        
        #Create a point to send out and return the final point
        p = Point(Xtemp,Ytemp,Ztemp)
        points = tf2_geometry_msgs.do_transform_point(PointStamped(point=p),
            tfs).point
        points = [points.x, points.y, -points.z, classId]
        print(points)
        # Finalcoordinates.append(points)
    return points
#Method to process the detected objects and return data about class, location, confidence
def process_detection(frame, outs, classes):
    #setup
    frameHeight = frame.shape[0]
    frameWidth = frame.shape[1]
    classIds = []
    confidences = []
    boxes = []
    center_coordinates_array = []
    #Go through outputs and if a detected object falls within our required confidence threashold
    #find the important details about the objects location
    for out in outs:
        for detection in out:
            scores = detection[5:]
            classId = np.argmax(scores)
            confidence = scores[classId]
            if confidence > confThreshold and classes[classId] != None and (classes[classId].lower() == "cup" or classes[classId].lower() == "bowl" or classes[classId].lower() == "plate" or classes[classId].lower() == "spoon" or classes[classId].lower() == "fork"):
                center_x = int(detection[0]*frameWidth)
                center_y = int(detection[1]*frameHeight)
                width = int(detection[2]*frameWidth)
                height = int(detection[3]*frameHeight)
                left = int(center_x - width/2)
                top = int(center_y - height/2)
                classIds.append(classId)
                confidences.append(float(confidence))
                boxes.append([left,top,width,height])
    indices = cv2.dnn.NMSBoxes(boxes, confidences, confThreshold, nmsThreshold)
    #Go through each detection and draw them to the screen
    for i in indices:
        i = int(i)
        box = boxes[i]
        left = box[0]
        top = box[1]
        width = box[2]
        height = box[3]
        x = int(left+width/2)
        y = int(top+ height/2)
        pd = (classIds[i], confidences[i], left, top, left+width, top+height,frame,x,y)
        drawPredicted(pd)
        return (classIds[i], confidences[i], left, top, left+width, top+height,frame,x,y)


def talker():

    # Create an instance of the rospy.Publisher object which we can  use to
    # publish messages to a topic. This publisher publishes messages of type
    # geometry_msgs/Point to the topic /yolo_coords
    pub = rospy.Publisher('yolo_coords', PointStamped, queue_size=0)
    
    # Create a timer object that will sleep long enough to result in a 10Hz
    # publishing rate
    r = rospy.Rate(10) # 10hz

    pub_string = None
    with open(classesFile, "rt") as f:
        classes = f.read().rstrip('\n').split('\n')
    net = cv2.dnn.readNetFromDarknet(modelConfiguration, modelWeights)
    net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
    net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
    try:
        while True:
            # Wait for a coherent pair of frames: depth and color
            frames = pipeline.wait_for_frames()
            
            # Align the depth frame to color frame
            aligned_frames = align.process(frames)

            # Get aligned frames
            aligned_depth_frame = aligned_frames.get_depth_frame()
            aligned_color_frame = aligned_frames.get_color_frame()

            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()
            if not aligned_depth_frame or not aligned_color_frame:
                continue
            # Convert images to numpy arrays
            depth_image = np.asanyarray(aligned_depth_frame.get_data())
            color_image = np.asanyarray(aligned_color_frame.get_data())
            blob = cv2.dnn.blobFromImage(color_image, scale, (inpWidth, inpHeight),0,True,crop=False)
            net.setInput(blob)
            outs = net.forward(getOutputsNames(net))
            # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
            depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
            pd = process_detection(color_image,outs,classes)
            depth_colormap_dim = depth_colormap.shape
            color_colormap_dim = color_image.shape
            # If depth and color resolutions are different, resize color image to match depth image for display
            if depth_colormap_dim != color_colormap_dim:
                resized_color_image = cv2.resize(color_image, dsize=(depth_colormap_dim[1], depth_colormap_dim[0]), interpolation=cv2.INTER_AREA)
                images = np.hstack((resized_color_image, depth_colormap))
            else:
                images = np.hstack((color_image, depth_colormap))
            #check if we have a processed detection and if so publish the point for the path planner
            if pd != None :
                pub_string = drawPredicted(pd)
                if (pub_string != None):
                    print("Point: ", pub_string[:3])
                    if( classes != None):
                        print("Class: ", classes[pub_string[3]])
                else:
                    print("Point: ", pub_string)
            if pub_string != None:  
                # Publish our string to the 'yolo_coords' topic

                ros_point = PointStamped()
                ros_point.point.x = pub_string[0]
                ros_point.point.y = pub_string[1]
                ros_point.point.z = pub_string[2]
                ros_point.header.frame_id = classes[pub_string[3]]
                pub.publish(ros_point)
                
                #r.sleep()
            # Show images
            cv2.imshow('Cam', images)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        # Stop streaming
        pipeline.stop()
    

if __name__ == "__main__":
    rospy.init_node("talker", anonymous=False)
    try:
        talker()
    except rospy.ROSInterruptException: pass