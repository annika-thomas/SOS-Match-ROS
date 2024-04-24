#!/usr/bin/env python
import rospy
from active_slam.SAM_data_association.blob_SAM_node import BlobSAMNode
import numpy as np
import cv2

# ROS imports
import rospy
import cv_bridge
import message_filters
from geometry_msgs.msg import Pose2D, PoseWithCovariance

# ROS msgs
import nav_msgs.msg as nav_msgs
import sensor_msgs.msg as sensor_msgs
import active_slam.msg as active_slam_msgs
import std_msgs.msg as std_msgs
import geometry_msgs.msg as geometry_msgs

# import rotation for pose stuff
from scipy.spatial.transform import Rotation as Rot

from active_slam.SAM_data_association.BlobTrackerRT import BlobTracker
from active_slam.SAM_data_association.BlobSfm2 import BlobSfm
from active_slam.SAM_data_association.SamDetectorDescriptorAndSizeComparer import SamDetectorDescriptorAndSizeComparer
from active_slam.SAM_data_association.SamFeatDdc import SamFeatDdc
from active_slam.SAM_data_association.FastSamWrapper import FastSamWrapper
from active_slam.SAM_data_association.utils import readConfig, getLogger, plotErrorEllipse

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

from utils import T_2_pose_msg, pose_msg_2_T

class gt_locations_node:

    def __init__(self):
        
        # internal variables
        self.bridge = cv_bridge.CvBridge()
        self.counter = 0
        self.K = rospy.get_param('~K', [284.5398, 0, 422.2144, 0, 285.4696, 388.6827, 0, 0, 1])

        # ros params
        self.num_meas_new_obj = rospy.get_param("~num_meas_new_obj", 3) # number of objects needed 
                                                                        # to create new object, default: 3
        
        # ros subscribers
        # subs = [
        #     message_filters.Subscriber("airsim_node/Multirotor/odom_local_ned", nav_msgs.Odometry, queue_size=100),
        #     message_filters.Subscriber("airsim_node/Multirotor/front_center_custom/Scene",
        #                                sensor_msgs.Image, queue_size=1),
        #     # message_filters.Subscriber("/airsim_node/Multirotor/front_center_custom/Scene/camera_info", 
        #     #                            sensor_msgs.CameraInfo),
        # ]

        # for highbay
        subs = [
            #message_filters.Subscriber("NX04/odometry", nav_msgs.Odometry, queue_size=100),
            message_filters.Subscriber("OBJ1/world", geometry_msgs.PoseStamped, queue_size=10),
            message_filters.Subscriber("OBJ2/world", geometry_msgs.PoseStamped, queue_size=10),
            message_filters.Subscriber("OBJ3/world", geometry_msgs.PoseStamped, queue_size=10),
            message_filters.Subscriber("OBJ4/world", geometry_msgs.PoseStamped, queue_size=10),
            message_filters.Subscriber("OBJ5/world", geometry_msgs.PoseStamped, queue_size=10),
            message_filters.Subscriber("OBJ6/world", geometry_msgs.PoseStamped, queue_size=10),
            message_filters.Subscriber("OBJ7/world", geometry_msgs.PoseStamped, queue_size=10),
            message_filters.Subscriber("OBJ8/world", geometry_msgs.PoseStamped, queue_size=10),
            message_filters.Subscriber("OBJ9/world", geometry_msgs.PoseStamped, queue_size=10),
            message_filters.Subscriber("OBJ10/world", geometry_msgs.PoseStamped, queue_size=10),
            message_filters.Subscriber("OBJ11/world", geometry_msgs.PoseStamped, queue_size=10),
            message_filters.Subscriber("OBJ12/world", geometry_msgs.PoseStamped, queue_size=10),
            message_filters.Subscriber("OBJ13/world", geometry_msgs.PoseStamped, queue_size=10),
            #message_filters.Subscriber("OBJ14/world", geometry_msgs.PoseStamped, queue_size=10),
            message_filters.Subscriber("OBJ15/world", geometry_msgs.PoseStamped, queue_size=10),
            message_filters.Subscriber("OBJ16/world", geometry_msgs.PoseStamped, queue_size=10),
            #message_filters.Subscriber("OBJ17/world", geometry_msgs.PoseStamped, queue_size=10),
            #message_filters.Subscriber("OBJ18/world", geometry_msgs.PoseStamped, queue_size=10),
            #message_filters.Subscriber("OBJ19/world", geometry_msgs.PoseStamped, queue_size=10),
            message_filters.Subscriber("OBJ20/world", geometry_msgs.PoseStamped, queue_size=10),
            message_filters.Subscriber("OBJ21/world", geometry_msgs.PoseStamped, queue_size=10),
            message_filters.Subscriber("OBJ22/world", geometry_msgs.PoseStamped, queue_size=10),
            message_filters.Subscriber("OBJ23/world", geometry_msgs.PoseStamped, queue_size=10),
            message_filters.Subscriber("OBJ27/world", geometry_msgs.PoseStamped, queue_size=10),
        ]

        # Approximate Time Synchronizer
        print("Creating Approximate Time Synchronizer")
        self.ts = message_filters.ApproximateTimeSynchronizer(subs, queue_size=100, slop=3, allow_headerless=True)
        print("Registering Callback")
        self.ts.registerCallback(self.cb) # registers incoming messages to callback

        # ros publishers
        self.marker_pub = rospy.Publisher("/object_locations", Marker, queue_size=100)

    # Callback for first goal reached
    def first_goal_reached_cb(self, msg):
        """
        This function gets called when the first goal is reached.
        """
        self.ts.registerCallback(self.cb) # registers incoming messages to callback

    def make_marker(self, msg, id):

        objmsg = msg
        marker = Marker()
        marker.header = objmsg.header
        #marker.header.frame_id = odom_msg.header.frame_id
        marker.id = id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.orientation.x = objmsg.pose.orientation.x
        marker.pose.orientation.y = objmsg.pose.orientation.y
        marker.pose.orientation.z = objmsg.pose.orientation.z
        marker.pose.orientation.w = objmsg.pose.orientation.w
        marker.pose.position.x = objmsg.pose.position.x
        marker.pose.position.y = objmsg.pose.position.y
        marker.pose.position.z = objmsg.pose.position.z
        marker.scale.x = 0.2  # Point size
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 1.0  # Alpha (transparency)
        marker.color.r = 1.0  # Color (red)
        marker.color.g = 0.0
        marker.color.b = 0.0

        print(f"Object at: {objmsg.pose.position.x}, {objmsg.pose.position.y}, {objmsg.pose.position.z}")

        # Publish the marker
        self.marker_pub.publish(marker)

    def cb(self, *msgs):
        """
        This function gets called every time synchronized odometry, image message, and camera info 
        message are available.
        """
        print("Callback called")
        # obj1msg, obj2msg, obj3msg, obj4msg, obj5msg, obj6msg, obj8msg, obj10msg, obj12msg, obj13msg, obj14msg, obj15msg, obj16msg, obj17msg, obj18msg, obj19msg, obj20msg, obj21msg, obj22msg = msgs
        #, obj5msg, obj6msg, obj7msg, obj8msg, obj9msg, obj10msg, obj11msg, obj12msg, obj13msg, obj14msg, obj15msg, obj16msg, obj17msg, obj18msg, obj19msg, obj20msg, obj21msg, obj22msg, obj23msg, obj27msg = msgs
        #obj_msg1, obj_msg2, obj_msg3 = msgs

        #obj1msg, obj2msg, obj3msg, obj4msg, obj5msg, obj6msg, obj13msg, obj14msg, obj15msg, obj16msg, obj17msg, obj18msg, obj19msg, obj20msg, obj21msg, obj22msg = msgs

        # obj1msg, obj2msg, obj3msg, obj4msg, obj5msg, obj6msg, obj7msg, obj11msg, obj12msg, obj13msg, obj14msg, obj15msg, obj16msg, obj17msg, obj18msg, obj19msg = msgs

        obj1msg, obj2msg, obj3msg, obj4msg, obj5msg, obj6msg, obj7msg, obj8msg, obj9msg, obj10msg, obj11msg, obj12msg, obj13msg, obj15msg, obj16msg, obj20msg, obj21msg, obj22msg, obj23msg, obj27msg = msgs

        # Create a marker for each object
        self.make_marker(obj1msg, 1)
        self.make_marker(obj2msg, 2)
        self.make_marker(obj3msg, 3)
        self.make_marker(obj4msg, 4)
        self.make_marker(obj5msg, 5)
        self.make_marker(obj6msg, 6)
        self.make_marker(obj7msg, 7)
        self.make_marker(obj11msg, 11)
        self.make_marker(obj12msg, 12)
        self.make_marker(obj13msg, 13)
        # self.make_marker(obj14msg, 14)
        self.make_marker(obj15msg, 15)
        self.make_marker(obj16msg, 16)
        # self.make_marker(obj17msg, 17)
        # self.make_marker(obj18msg, 18)
        # self.make_marker(obj19msg, 19)
        self.make_marker(obj20msg, 20)
        self.make_marker(obj21msg, 21)
        self.make_marker(obj22msg, 22)
        self.make_marker(obj23msg, 23)
        self.make_marker(obj27msg, 27)

        return
    
def main():

    rospy.init_node('gt_locations_node')
    node = gt_locations_node()
    rospy.spin()

if __name__ == "__main__":
    main()