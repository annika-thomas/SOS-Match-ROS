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
# import active_slam.msg as active_slam_msgs
import motlee_msgs.msg as motlee_msgs
import std_msgs.msg as std_msgs
import geometry_msgs.msg as geometry_msgs

# import rotation for pose stuff
from scipy.spatial.transform import Rotation as Rot

from active_slam.SAM_data_association.BlobTrackerRT import BlobTracker
from active_slam.SAM_data_association.BlobSfm import BlobSfm
from active_slam.SAM_data_association.SamDetectorDescriptorAndSizeComparer import SamDetectorDescriptorAndSizeComparer
from active_slam.SAM_data_association.SamFeatDdc import SamFeatDdc
from active_slam.SAM_data_association.FastSamWrapper import FastSamWrapper
from active_slam.SAM_data_association.utils import readConfig, getLogger, plotErrorEllipse, compute_3d_position_of_centroid
from active_slam.SAM_data_association.flat_world_reconstructor import FlatWorldReconstructor

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

from utils import T_2_pose_msg, pose_msg_2_T

class SAM_DA_node:

    def __init__(self):
        
        # internal variables
        self.bridge = cv_bridge.CvBridge()
        self.counter = 0
        # TODO: make rosparam
        #self.K = rospy.get_param('~K', [320.0, 0.0, 320.0, 0.0, 320.0, 240.0, 0.0, 0.0, 1.0]) - for airsim
        #self.K = rospy.get_param('~K' [718.856, 0.0, 607.1928, 0.0, 718.856, 185.2157, 0.0, 0.0, 1.0]) # for KITTI
        #self.K = rospy.get_param('~K', [286.3395, 0.0, 420.897, 0.0, 286.24, 404.23, 0, 0, 1.0]) # for highbay my bag
        self.K = rospy.get_param('~K', [284.864, 0, 419.02, 0, 285.92, 409.17, 0, 0, 1]) #for puma bag
        #self.K = rospy.get_param('~K', [284.5398, 0, 422.2144, 0, 285.4696, 388.6827, 0, 0, 1])
        #self.distortion_params = rospy.get_param('~distortion_params', [0.0, 0.0, 0.0, 0.0])
        #self.distortion_params = rospy.get_param('~distortion_params', [-0.011009, 0.046768, 0.044220, 0.0008363]) # for highbay my bag
        self.distortion_params = rospy.get_param('~distortion.params', [-0.005343, 0.039268, -0.037734, 0.006587]) # puma bag
        #self.distortion_params = rospy.get_param('~distortion_params', [-0.004301, 0.039112, -0.037137, 0.006241])
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
            message_filters.Subscriber("NX08/world", geometry_msgs.PoseStamped, queue_size=10),
            message_filters.Subscriber("NX08/t265/fisheye1/image_raw",
                                       sensor_msgs.Image, queue_size=10),
            # message_filters.Subscriber("/airsim_node/Multirotor/front_center_custom/Scene/camera_info", 
            #                            sensor_msgs.CameraInfo),
        ]

        # SAM params      
        matchingScoreLowerLimit = 0
        fTestLimit = 2.0
        numFramesToSearchOver = 3 
        pixelMsmtNoiseStd = 3.0
        numObservationsRequiredForTriang = 3
        huberParam = 0.5

        # FastSAM params
        similaritymethod = 'feat'
        pathToCheckpoint = "./FastSAM/Models/FastSAM-x.pt"
        device = "cpu"
        conf = 0.5
        iou = 0.9
        imgsz = 256
        samModel = FastSamWrapper(pathToCheckpoint, device, conf, iou, imgsz)

        logger = getLogger()

        maxSizeDiff = 0.5
        ddc = SamFeatDdc(samModel, maxSizeDiff)

        # Instantiate blob tracker
        self.blobTracker = BlobTracker(ddc, fTestLimit, matchingScoreLowerLimit, numFramesToSearchOver, logger)
        print("BlobTracker instantiated")

        # Instantiate BlobSAMNode
        self.blob_sam_node = BlobSAMNode(image=None, T=None, filename=None, blobTracker=self.blobTracker)  # Instantiate BlobSAMNode class
        print("BlobSAMNode instantiated")

        self.blob_sfm =  BlobSfm(fx, fy, s, cx, cy, 0 ,0, 0, 0, pixelMsmtNoiseStd, huberParam, numObservationsRequiredForTriang)
        print("BlobSfm instantiated")

        self.flat_world_reconstructor = FlatWorldReconstructor(fx, fy, s, cx, cy, 0, 0, 0, 0, minNumObservations=3)

        # subscribe to /first_goal_reached topic
        #self.first_goal_reached_sub = rospy.Subscriber("/first_goal_reached", std_msgs.Bool, self.first_goal_reached_cb)

        # Approximate Time Synchronizer
        self.ts = message_filters.ApproximateTimeSynchronizer(subs, queue_size=10, slop=1, allow_headerless=True)
        self.ts.registerCallback(self.cb) # registers incoming messages to callback

        # ros publishers
        self.meas_pub = rospy.Publisher("measurement_packet", active_slam_msgs.MeasurementPacket, queue_size=5)
        self.marker_pub = rospy.Publisher("/landmarks", Marker, queue_size=100)
        self.marker_text_pub = rospy.Publisher("/landmarks_text", Marker, queue_size=100)
        self.vis_pub = rospy.Publisher("/frame_vis", sensor_msgs.Image, queue_size=5)
        self.undistorted_img_pub = rospy.Publisher("/undistorted_vis", sensor_msgs.Image, queue_size=5)
        self.pose_transformed_pub = rospy.Publisher("/pose_transformed", geometry_msgs.PoseStamped, queue_size=5)
        self.obj_pub = rospy.Publisher("object_locations_packet", motlee_msgs.ObjArray, queue_size=5)

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

        # conversion from ros msg to cv img
        img = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')

        # convert image to np.uint8
        img_undist = np.array(img, dtype=np.uint8)

        # undistort image
        h, w = img_undist.shape[:2]
        scale_factor = 1.0
        balance = 1.0
        img_dim_out = (int(w*scale_factor), int(h*scale_factor))

        K = np.array([
            [284.864, 0, 419.02],
            [0.0, 285.92, 409.17],
            [0, 0, 1]
        ])

        D = np.array([
            [-0.005343],
            [0.039268],
            [-0.037734],
            [0.006587]
        ])

        nk = K.copy()
        nk[0,0]=K[0,0]*0.55
        nk[1,1]=K[1,1]*0.55

        map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), nk, img_dim_out, cv2.CV_32FC1)
        undistorted_img = cv2.remap(img_undist, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
        img = undistorted_img

        # Convert img to RGB color space from BGR
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

        h, w, c = img.shape
        print("Width:", w)
        print("Height:", h)

        # Define the size of the crop desired
        crop_width = 500
        crop_height = 500

        # Ensure the crop size is not larger than the image size
        crop_width = min(crop_width, w)
        crop_height = min(crop_height, h)

        # Calculate the starting point for cropping to keep the crop centered
        crop_start_x = (w - crop_width) // 2
        crop_start_y = (h - crop_height) // 2

        # Perform the cropping
        cropped_image = img[crop_start_y:(crop_start_y + crop_height), crop_start_x:(crop_start_x + crop_width)]

        # Get the shape of the cropped image
        hc, wc, cc = cropped_image.shape
        print("Cropped Width:", wc)
        print("Cropped Height:", hc)
        print("nk: ", nk)

        # If needed to use cropped_image in the subsequent part of your program
        img = cropped_image

        undist_img_msg = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
        undist_img_msg.header = img_msg.header
        self.undistorted_img_pub.publish(undist_img_msg)

        cropped_u = 250
        cropped_v = 250

        k_crop = np.array([[nk[0,0], 0, cropped_u], [0, nk[1,1], cropped_v], [0, 0, 1]])

        R = Rot.from_quat([odom_msg.pose.orientation.x, odom_msg.pose.orientation.y, odom_msg.pose.orientation.z, odom_msg.pose.orientation.w]).as_matrix()

        # R = Rot.from_matrix(rotated_matrix)
        t = np.array([odom_msg.pose.position.x, odom_msg.pose.position.y, odom_msg.pose.position.z])
        T = np.eye(4); T[:3,:3] = R; T[:3,3] = t

        # Publish the pose transformed
        pose_transformed_msg = geometry_msgs.PoseStamped()
        pose_transformed_msg.header = odom_msg.header
        pose_transformed_msg.pose.position.x = T[0,3]
        pose_transformed_msg.pose.position.y = T[1,3]
        pose_transformed_msg.pose.position.z = T[2,3]
        pose_transformed_msg.pose.orientation.x = (Rot.from_matrix(R)).as_quat()[0]
        pose_transformed_msg.pose.orientation.y = (Rot.from_matrix(R)).as_quat()[1]
        pose_transformed_msg.pose.orientation.z = (Rot.from_matrix(R)).as_quat()[2]
        pose_transformed_msg.pose.orientation.w = (Rot.from_matrix(R)).as_quat()[3]
        self.pose_transformed_pub.publish(pose_transformed_msg)

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

        #(initialError, finalError) = self.blob_sfm.reconstruct(poseHist, poseNoiseHist, tracks)
        (initialError, finalError) = self.flat_world_reconstructor.reconstruct(poseHist, poseNoiseHist, tracks, k_crop)
        print(f"Initial error={initialError}, final error={finalError}")

        # reconstruction_start_time = rospy.Time.now()
        landmarkMAPmeans = self.flat_world_reconstructor.getReconstructionResults()
        #landmarkMAPmeans, landmarkMAPcovs, poseMAPmeans, poseMAPcovs, landmarkSizes = self.blob_sfm.getReconstructionResults()

        # reconstruction_end_time = rospy.Time.now()
        # print(f"Reconstruction took {reconstruction_end_time.secs - reconstruction_start_time.secs} seconds")

        print("landmarkMAPmeans: ", landmarkMAPmeans)

        # Create measurement packet
        obj_packet = active_slam_msgs.ObjArray()
        obj_packet.header = img_msg.header

        for idx, components in landmarkMAPmeans.items():
            # fuse landmarks that are less than 0.2 meters apart
            for idx2, components2 in landmarkMAPmeans.items():
                if idx == idx2:
                    continue
                x1, y1, z1 = components
                x2, y2, z2 = components2
                dist = np.linalg.norm(np.array([x1, y1, z1]) - np.array([x2, y2, z2]))
                if dist < 0.2:
                    landmarkMAPmeans[idx] = (x1 + x2) / 2, (y1 + y2) / 2, (z1 + z2) / 2
                    landmarkMAPmeans[idx2] = (x1 + x2) / 2, (y1 + y2) / 2, (z1 + z2) / 2
                    #del landmarkMAPmeans[idx2]
                    break

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


            # Marker for the text
            text_marker = Marker()
            text_marker.header = img_msg.header
            text_marker.id = idx
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.scale.z = 0.5  # Text size
            text_marker.color.a = 1.0
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.text = f"{idx}"

            # Set point coordinates
            p = Point()
            x, y, z = components
            print(f"Landmark {idx} at x: {x}, y: {y}, z: {z}")

            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = z

            text_marker.pose.position.x = x + 0.1
            text_marker.pose.position.y = y + 0.1
            text_marker.pose.position.z = z + 0.1

            # Publish the marker
            self.marker_pub.publish(marker)
            self.marker_text_pub.publish(text_marker)

            Obj = motlee_msgs.Obj()
            Obj.id = idx
            Obj.class_name = "landmark"
            Obj.position = geometry_msgs.Point(x=x, y=y, z=z)
            Obj.covariance = np.diag([0.0, 0.0, 0.0]).reshape(-1)
            Obj.width = 0.0
            Obj.height = 0.0
            Obj.ell = 0.0

            obj_packet.objects.append(Obj)

        
        # print(packet)
        self.meas_pub.publish(packet)

        self.obj_pub.publish(obj_packet)

        self.last_pose = T
        self.counter = self.counter + 1

        return
    
def main():

    rospy.init_node('SAM_DA_node')
    node = SAM_DA_node()
    rospy.spin()

if __name__ == "__main__":
    main()