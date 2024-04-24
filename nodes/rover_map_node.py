#!/usr/bin/env python
import rospy
from active_slam.SAM_data_association.blob_SAM_node import BlobSAMNode
import numpy as np

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
import matplotlib.pyplot as plt

import cv2

from utils import T_2_pose_msg, pose_msg_2_T

class rover_map_node:

    def __init__(self):
        
        # internal variables
        self.bridge = cv_bridge.CvBridge()
        self.counter = 0
        # TODO: make rosparam
        # self.K = rospy.get_param('~K', [284.5398, 0, 422.2144, 0, 285.4696, 388.6827, 0, 0, 1]) #RR08
        # self.distortion_params = rospy.get_param('~distortion_params', [-0.004301, 0.039112, -0.037137, 0.006241]) #RR08

        self.K = [285.726501, 0.000000, 425.540405, 0.000000, 285.773010, 400.354095, 0.000000, 0.000000, 1.000000] #RR04
        #self.distortion_params = [-0.006606, 0.042409, -0.040681, 0.007675] #RR04
        self.distortion_params = [0, 0, 0, 0]

        fx = self.K[0]
        fy = self.K[4]
        cx = self.K[2]
        cy = self.K[5]
        s = 0
        k1 = self.distortion_params[0]
        k2 = self.distortion_params[1]
        p1 = self.distortion_params[2]
        p2 = self.distortion_params[3]

        # ros params
        self.num_meas_new_obj = rospy.get_param("~num_meas_new_obj", 3) # number of objects needed 
                                                                        # to create new object, default: 3
        
        # ros subscribers
        subs = [
            message_filters.Subscriber("RR04/world", geometry_msgs.PoseStamped, queue_size=10),
            message_filters.Subscriber("RR04/t265/fisheye1/image_raw/compressed",
                                       sensor_msgs.CompressedImage, queue_size=10),
        ]

        # SAM params      
        matchingScoreLowerLimit = 0
        fTestLimit = 4. # 2.0
        numFramesToSearchOver = 5
        pixelMsmtNoiseStd = 3.0
        numObservationsRequiredForTriang = 3
        huberParam = 0.5

        # FastSAM params
        similaritymethod = 'size'
        pathToCheckpoint = "./FastSAM/Models/FastSAM-x.pt"
        device = "cpu"
        conf = 0.5
        iou = 0.9
        imgsz = 256
        samModel = FastSamWrapper(pathToCheckpoint, device, conf, iou, imgsz)

        logger = getLogger()

        maxSizeDiff = 3.
        ddc = SamFeatDdc(samModel, maxSizeDiff)
        #ddc = SamDetectorDescriptorAndSizeComparer(samModel, maxSizeDiff)

        # Instantiate blob tracker
        self.blobTracker = BlobTracker(ddc, fTestLimit, matchingScoreLowerLimit, numFramesToSearchOver, logger)
        print("BlobTracker instantiated")

        # Instantiate BlobSAMNode
        self.blob_sam_node = BlobSAMNode(image=None, T=None, filename=None, blobTracker=self.blobTracker)  # Instantiate BlobSAMNode class
        print("BlobSAMNode instantiated")

        self.blob_sfm =  BlobSfm(fx, fy, s, cx, cy, k1, k2, p1, p2, pixelMsmtNoiseStd, huberParam, numObservationsRequiredForTriang)
        print("BlobSfm instantiated")

        # subscribe to /first_goal_reached topic
        #self.first_goal_reached_sub = rospy.Subscriber("/first_goal_reached", std_msgs.Bool, self.first_goal_reached_cb)

        # Approximate Time Synchronizer
        self.ts = message_filters.ApproximateTimeSynchronizer(subs, queue_size=10, slop=1, allow_headerless=True)
        self.ts.registerCallback(self.cb) # registers incoming messages to callback

        # ros publishers
        self.meas_pub = rospy.Publisher("measurement_packet", active_slam_msgs.MeasurementPacket, queue_size=5)
        self.marker_pub = rospy.Publisher("/landmarks", Marker, queue_size=100)
        self.vis_pub = rospy.Publisher("/frame_vis", sensor_msgs.Image, queue_size=5)
        self.undistorted_img_pub = rospy.Publisher("/undistorted_vis", sensor_msgs.Image, queue_size=5)

        # initialize last image and pose
        self.last_image = None
        self.last_pose = None
        self.noise_amount = 0.1

    # Callback for first goal reached
    def first_goal_reached_cb(self, msg):
        """
        This function gets called when the first goal is reached.
        """
        self.ts.registerCallback(self.cb) # registers incoming messages to callback

    def cb(self, *msgs):
        """
        This function gets called every time synchronized odometry, image message, and camera info 
        message are available.
        """
        odom_msg, img_msg, = msgs #cam_info_msg = msgs

        counter = self.counter


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


        undist_img_msg = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
        undist_img_msg.header = img_msg.header
        self.undistorted_img_pub.publish(undist_img_msg)
        # extract camera intrinsics
        # K = np.array([cam_info_msg.K]).reshape((3,3))

        # conversion from ros msg to cv img
        #img = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')

        # extract camera intrinsics
        # K = np.array([cam_info_msg.K]).reshape((3,3))

        # extract pose from odom msg using position and orientation
        R = Rot.from_quat([odom_msg.pose.orientation.x, odom_msg.pose.orientation.y, \
        odom_msg.pose.orientation.z, odom_msg.pose.orientation.w])
        t = np.array([odom_msg.pose.position.x, odom_msg.pose.position.y, odom_msg.pose.position.z])
        T = np.eye(4); T[:3,:3] = R.as_matrix(); T[:3,3] = t


        tf_RR08 = np.array([
            [-0.009958163284760874, 0.022134946937857836, 0.9997053961583144, 0.21267445380719252], 
            [-0.9994378596291769, -0.03222655749690187, -0.009241954974415656, 0.05027555519476915], 
            [0.032012493246298124, -0.9992354542929096, 0.022443421312237893, 0.08365516650207658],
            [0.0, 0.0, 0.0, 1]])

        tf_RR04 = np.array([
            [-0.00842962690343002, 0.04733400415605965, 0.9988435480298314, 0.15566182289060196],
            [-0.9995538355095872, -0.029022096747316317, -0.007060298751584426, 0.06198987145979305],
            [0.028654341875903933, -0.9984574151915258, 0.047557530851751406, 0.07511768096997164],
            [0.0, 0.0, 0.0, 1]])
        
        # transform with tf by multiplying them
        T = np.dot(T, tf_RR04)

        # get latest keyframe index
        keyframe = self.blob_sam_node.blobTracker.latestKeyframeIndex
        if keyframe == None:
            keyframe = 0

        # create a noise transformation matrix
        pose_pos_cov = 0.01 # meters
        pose_rot_cov = 0.01 # degrees
        rpy_cov = np.deg2rad(pose_rot_cov)**2
        xyz_cov = pose_pos_cov**2
        noise_matrix = np.eye(4)
        noise_matrix[:3, :3] = Rot.from_euler('xyz', np.random.multivariate_normal([0,0,0], np.diag([rpy_cov, rpy_cov, rpy_cov]))).as_matrix()
        noise_matrix[:3, 3] = np.random.multivariate_normal([0,0,0], np.diag([xyz_cov, xyz_cov, xyz_cov])) 

        # image and pose in BlobSAMNode
        self.blob_sam_node.image = img
        self.blob_sam_node.T = T # TODO: blob_sam is using noiseless T for now, if we switch to noisy we need
                                 # to keep this transform separate from the one we are using to compute the incremental_pose.
                                 # Noise should be applied to true relative pose
        self.blob_sam_node.filename = self.blob_sam_node.blobTracker.latestKeyframeIndex
        self.blob_sam_node.blobTracker = self.blobTracker

        print(f"latest keyframe index: {keyframe}")

        # Process image and get tracks
        tracks = self.blob_sam_node.process_image()

        # Create measurement packet
        packet = active_slam_msgs.MeasurementPacket()
        packet.header = img_msg.header
        packet.sequence = np.int32(counter)

        # Add relative pose measurement
        if self.last_pose is None: # for initial pose
            packet.incremental_pose = PoseWithCovariance()
            # TODO: right now sending the absolute pose as the first message, a bit hacky
            packet.incremental_pose.pose = T_2_pose_msg(T) # not adding noise to initial pose for now
            packet.incremental_pose.covariance = np.diag([xyz_cov, xyz_cov, xyz_cov, rpy_cov, rpy_cov, rpy_cov]).reshape(-1)
        else:
            packet.incremental_pose = PoseWithCovariance()
            incremental_pose = np.linalg.inv(self.last_pose) @ T
            packet.incremental_pose.pose = T_2_pose_msg(noise_matrix @ incremental_pose) # noise needs to be applied after finding the true incremental pose
            packet.incremental_pose.covariance = np.diag([xyz_cov, xyz_cov, xyz_cov, rpy_cov, rpy_cov, rpy_cov]).reshape(-1)

        for track in self.blobTracker.tracks:

            # print("Track ID: ", track.trackId)
            print("Counter: ", counter)

            frames_where_seen = track.framesWhereSeen
            # num track id is length of frames_where_seen
            num_track_id = len(frames_where_seen)
            print(f"Track ID: {track.trackId}, Number of times seen: {num_track_id}")

            print("Frames wheres seen: ", frames_where_seen)

            # only run if seen in latest frame:
            if counter not in frames_where_seen:
                continue

            current_px_coords, current_desc = track.getPxcoordsAndDescriptorsForFrame(counter)
            current_sift = current_desc[1]
            #print(f"current sift: {current_sift}")

            # If the track has been seen in 3 frames and pixel coordinates is not none, add all three to the measurement packet
            if num_track_id == 3 and current_px_coords is not None:
                # make a segment measurement for each counter in frames where seen
                for frame in frames_where_seen:
                    print(f"Adding one of three segment measurements to packet at sequence {frame} for track {track.trackId}")

                    #print(f"Frame: {frame}")

                    px_coords, desc = track.getPxcoordsAndDescriptorsForFrame(frame)
                    sift = desc[1]
                    #print(f"Pixel Coordinates: {px_coords}")
                    
                    segmentMeasurement = active_slam_msgs.SegmentMeasurement()
                    segmentMeasurement.id = track.trackId
                    segmentMeasurement.center = Pose2D(x=px_coords[0], y=px_coords[1], theta=0)
                    segmentMeasurement.sequence = np.int32(frame) 
                    # TODO: make rosparam pixel covariance
                    segmentMeasurement.covariance = np.diag([3., 3.]).reshape(-1)
                    # segmentMeasurement.latest_sift = sift
                    packet.segments.append(segmentMeasurement)

            if num_track_id > 3 and current_px_coords is not None:
                print(f"Adding one new segment measurement to packet at sequence {counter} for track {track.trackId}")

                #print(f"Frame: {counter}")

                #print(f"Pixel Coordinates: {current_px_coords}")

                segmentMeasurement = active_slam_msgs.SegmentMeasurement()
                segmentMeasurement.id = track.trackId
                segmentMeasurement.center = Pose2D(x=current_px_coords[0], y=current_px_coords[1], theta=0)
                segmentMeasurement.sequence = np.int32(counter) 
                # TODO: make rosparam pixel covariance
                segmentMeasurement.covariance = np.diag([3., 3.]).reshape(-1)
                # segmentMeasurement.latest_sift = current_sift
                packet.segments.append(segmentMeasurement)


        poseHist = self.blobTracker.getPoseHistory()
        poseNoiseHist = self.blobTracker.getPoseNoiseHistory()
        tracks = self.blobTracker.getFeatureTracks()
        frameVis = self.blobTracker.visualizeFrameROS(counter)

        # Publish frame visualization
        frame_vis_msg = self.bridge.cv2_to_imgmsg(frameVis, encoding="bgr8")
        frame_vis_msg.header = img_msg.header
        self.vis_pub.publish(frame_vis_msg)

        (initialError, finalError) = self.blob_sfm.reconstruct(poseHist, poseNoiseHist, tracks)
        print(f"Initial error={initialError}, final error={finalError}")

        reconstruction_start_time = rospy.Time.now()
        landmarkMAPmeans, landmarkMAPcovs, poseMAPmeans, poseMAPcovs, landmarkSizes = self.blob_sfm.getReconstructionResults()
        # reconstruction_end_time = rospy.Time.now()
        # print(f"Reconstruction took {reconstruction_end_time.secs - reconstruction_start_time.secs} seconds")

        print("landmarkMAPmeans: ", landmarkMAPmeans)

        for idx, components in landmarkMAPmeans.items():
            marker = Marker()
            marker.header = img_msg.header
            marker.header.frame_id = odom_msg.header.frame_id
            marker.id = idx
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.2  # Point size
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            marker.color.a = 1.0  # Alpha (transparency)
            marker.color.r = 0.0  # Color (red)
            marker.color.g = 1.0
            marker.color.b = 0.0


            # Set point coordinates
            p = Point()
            x, y, z = components
            print(f"Landmark {idx} at x: {x}, y: {y}, z: {z}")

            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = z

            # p.x = x  # Replace with your point coordinates
            # p.y = y
            # p.z = z
            # marker.points.append(p)

            # Publish the marker
            self.marker_pub.publish(marker)

        
        # print(packet)
        self.meas_pub.publish(packet)

        self.last_pose = T
        self.counter = self.counter + 1

        return
    
def main():

    rospy.init_node('rover_map_node')
    node = rover_map_node()
    rospy.spin()

if __name__ == "__main__":
    main()