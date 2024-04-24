#!/usr/bin/env python3
# written by Mason's UROP

import cv2 as cv
import rospy
import cv_bridge
import yolov7_package as yolo
import sensor_msgs.msg as sensor_msgs
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Header
from vision_msgs.msg import BoundingBox2D
from vision_msgs.msg import Detection2D
from vision_msgs.msg import Detection2DArray
from vision_msgs.msg import ObjectHypothesisWithPose
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
import numpy as np
import cv2



class Detect2D():
    def __init__(self):
        self.node_name = rospy.get_name()
        self.bridge = cv_bridge.CvBridge()
        self.detector = yolo.Yolov7Detector()
        self.detector.conf_thres = 0.75

        # set up sub and pub
        self.img_sub = rospy.Subscriber("RR04/t265/fisheye1/image_raw/compressed", sensor_msgs.CompressedImage, self.img_proc_cb, queue_size=10)

        self.img_pub = rospy.Publisher("image_with_bbox", Image, queue_size=1)
        self.detections_pub = rospy.Publisher("detection2Ds", Detection2DArray, queue_size=1)
        
    def img_proc_cb(self, msg):

        img_msg = msg

        np_arr = np.fromstring(img_msg.data, np.uint8)
        # compressed image to cv image
        img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        # convert image to np.uint8
        img_undist = np.array(img, dtype=np.uint8)

        # undistort image
        h, w = img_undist.shape[:2]
        scale_factor = 1.0
        balance = 1.0
        img_dim_out = (int(w*scale_factor), int(h*scale_factor))

        K = np.array([
            [285.726501, 0.000000, 425.540405],
            [0.000000, 285.773010, 400.354095],
            [0.000000, 0.000000, 1.000000]])
        

        # self.K = [285.726501, 0.000000, 425.540405, 0.000000, 285.773010, 400.354095, 0.000000, 0.000000, 1.000000] #RR04
        # self.distortion_params = [-0.006606, 0.042409, -0.040681, 0.007675] #RR04

        D = np.array([
            [-0.006606], 
            [0.042409], 
            [-0.040681], 
            [0.007675]
        ])

        nk = K.copy()
        nk[0,0]=K[0,0]/2
        nk[1,1]=K[1,1]/2

        map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), nk, img_dim_out, cv2.CV_32FC1)
        undistorted_img = cv2.remap(img_undist, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
        img = undistorted_img

        # convert img to cv2 with bgr8 encoding
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

        print("callback called")
        #img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")  # serialize, returns cv::Mat
        _classes, _boxes, _confidences = self.detector.detect(img)
        detections = Detection2DArray()
        detections.header = msg.header
        for classes, boxes, confidences in zip(_classes, _boxes, _confidences):
            for cls, box, conf in zip(classes, boxes, confidences):
                if self.detector.names[cls] == 'backpack':
                    img = cv.rectangle(img, (int(box[0]), int(box[1])), (int(box[2]), int(box[3])), (0,0,255), 2)
                    # make bbox; msg measurements are in pixels
                    center = Pose2D()
                    center.x = (box[0]+box[2])/2
                    center.y = (box[1]+box[3])/2
                    center.theta = 0
                    bbox = BoundingBox2D()
                    bbox.center = center
                    bbox.size_x = box[2]-box[0]
                    bbox.size_y = box[3]-box[1]

                    # object hypothesis
                    obj_hyp = ObjectHypothesisWithPose()
                    obj_hyp.id = cls
                    obj_hyp.score = conf
                    
                    #Detection 2D
                    detection = Detection2D()
                    detection.header = msg.header
                    detection.bbox = bbox
                    detection.results = [obj_hyp]           #TODO: yolo might give multiple estimates for object find out what to do with this array

                    # append to detections array
                    detections.detections.append(detection)
                    
        self.detections_pub.publish(detections)
        print("Number of detections: ", len(detections.detections))
        self.img_pub.publish(self.bridge.cv2_to_imgmsg(img, encoding="bgr8"))

if __name__ == "__main__":
    rospy.init_node('yolo_node', anonymous=False)
    node = Detect2D()
    print("2D Detector Started")
    rospy.spin()

# shape of yolo data
# [[62, 63, 57, 16]]
# [[
#     [0.984375, 2.731640625, 449.66249999999997, 297.08437499999997],
#     [1.7718749999999999, 390.403125, 217.35, 724.696875],
#     [2712.15, 4.1343749999999995, 4028.85, 1776.6],
#     [1595.475, 1010.5593749999999, 2616.075, 1762.425]
# ]]
# [[0.477783203125, 0.53369140625, 0.84716796875, 0.9345703125]]
