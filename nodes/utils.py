from scipy.spatial.transform import Rotation as Rot
import numpy as np

import geometry_msgs.msg as geometry_msgs

def pose_msg_2_T(pose_msg):
    R = Rot.from_quat([pose_msg.orientation.x, pose_msg.orientation.y, \
        pose_msg.orientation.z, pose_msg.orientation.w])
    t = np.array([pose_msg.position.x, pose_msg.position.y, pose_msg.position.z])
    T = np.eye(4); T[:3,:3] = R.as_matrix(); T[:3,3] = t
    return T

def T_2_pose_msg(T: np.array):
    pose = geometry_msgs.Pose()
    pose.position.x, pose.position.y, pose.position.z = T[:3,3]
    q = Rot.from_matrix(T[:3,:3]).as_quat()
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = q
    return pose

def transform_vec(T, vec):
    unshaped_vec = vec.reshape(-1)
    resized_vec = np.concatenate(
        [unshaped_vec, np.zeros((T.shape[0] - 1 - unshaped_vec.shape[0]))]).reshape(-1)
    resized_vec = np.concatenate(
        [resized_vec, np.ones((T.shape[0] - resized_vec.shape[0]))]).reshape((-1, 1))
    transformed = T @ resized_vec
    return transformed.reshape(-1)[:unshaped_vec.shape[0]].reshape(vec.shape) 

def transform(T, vecs, stacked_axis=1):
    if len(vecs.reshape(-1)) == 2 or len(vecs.reshape(-1)) == 3:
        return transform_vec(T, vecs)
    vecs_horz_stacked = vecs if stacked_axis==1 else vecs.T
    zero_padded_vecs = np.vstack(
        [vecs_horz_stacked, np.zeros((T.shape[0] - 1 - vecs_horz_stacked.shape[0], vecs_horz_stacked.shape[1]))]
    )
    one_padded_vecs = np.vstack(
        [zero_padded_vecs, np.ones((1, vecs_horz_stacked.shape[1]))]
    )
    transformed = T @ one_padded_vecs
    transformed = transformed[:vecs_horz_stacked.shape[0],:] 
    return transformed if stacked_axis == 1 else transformed.T

def xyz_2_pixel(xyz, K, axis=0):
    """
    Converts xyz point array to pixel coordinate array

    Args:
        xyz (np.array, shape=(3,n) or (n,3)): 3D coordinates in RDF camera coordinates
        K (np.array, shape=(3,3)): camera intrinsic calibration matrix
        axis (int, optional): 0 or 1, axis along which xyz coordinates are stacked. Defaults to 0.

    Returns:
        np.array, shape=(2,n) or (n,2): Pixel coordinates (x,y) in RDF camera coordinates
    """
    if axis == 0:
        xyz_shaped = np.array(xyz).reshape((-1,3)).T
    elif axis == 1:
        xyz_shaped = np.array(xyz).reshape((3,-1))
    else:
        assert False, "only axis 0 or 1 supported"
        
    pixel = K @ xyz_shaped / xyz_shaped[2,:]
    pixel = pixel[:2,:]
    if axis == 0:
        pixel = pixel.T
    return pixel

def compute_3d_position_of_centroid(pixel, pose, camera, K):

    # get x_im from pixel
    x_im = np.array([pixel[0], pixel[1], 1.0])

    if K is None:
        # get camera intrinsics
        print("get K matrix - make sure K is updated in compute_3d_position_of_centroid()")
        if camera == 'voxl':
            # voxl camera calibration matrix (intrinsic parameters - look /data/modalai/opencv_tracking_intrinsics.yml)
            # K = np.array([[273.90235382142345, 0.0, 315.12271705027996], [0., 274.07405600616045, 241.28160498854680], [0.0, 0.0, 1.0]])
            # need to use a new K that is generated after undistortion
            K = np.array([[135.48433328, 0., 310.83524106], [0., 135.56926484, 241.39230258], [0., 0., 1.]])
        elif camera == 't265_fisheye1':
            K = np.array([[85.90185242, 0., 420.89700317], [0., 85.87307739, 404.22949219], [0., 0., 1.]])
        elif camera == 't265_fisheye2':
            K = np.array([[86.09328003, 0., 418.48440552], [0., 86.08716431, 400.93289185], [0., 0., 1.]])
        else:
            raise ValueError('Invalid camera name')

    # Transformatoin matrix T^b_w (from world to body)
    T_b_w = np.zeros((4,4))
    T_b_w[:3, :3] = np.linalg.inv(Rot.from_quat([pose[3], pose[4], pose[5], pose[6]]).as_matrix()) #from_quat() takes as input a quaternion in the form [x, y, z, w]
    T_b_w[:3, 3] = -np.matmul(T_b_w[:3,:3], np.array(pose[0:3]))
    T_b_w[3, 3] = 1.0

    # Transformation matrix T^c_b (from body to camera)
    T_c_b = get_transformation_from_body_to_camera(camera)

    # Transformation matrix T^c_w (from world to camera)
    T_c_w = np.matmul(T_c_b, T_b_w)

    # Get T and R from T_c_w
    R_c_w = T_c_w[:3, :3]
    t_c_w = T_c_w[:3, 3]

    # rotate R 180 degrees around z axis
    #R_c_w = np.matmul(Rot.from_euler('z', 180, degrees=True).as_matrix(), R_c_w)

    # Get X_o from pose (or you can get it by - np.linalg.inv(R_c_w) @ t_c_w)
    # X_o = np.array(pose[0:3]) + camera_translation # which is the same as - np.linalg.inv(R_c_w) @ t_c_w
    X_o = - np.linalg.inv(R_c_w) @ t_c_w

    # compute lambda (depth) in X_w = X_o + lambda * (K * R_c_w)^-1 * x_im using flat earth assumption
    lambda_ = (0.0 - X_o[2]) / (np.matmul(np.linalg.inv(np.matmul(K, R_c_w)), x_im)[2])

    # compute X_w
    X_w = X_o + lambda_ * np.matmul(np.linalg.inv(np.matmul(K, R_c_w)), x_im)

    if lambda_ < 0:
        # raise ValueError(f"lambda_ {lambda_} < 0 \n pixel: {pixel}")
        #print(f"lambda_ {lambda_} < 0 \n pixel: {pixel}")
        return 
    
    if abs(X_w[2]) > 1e-2:
        raise ValueError(f"X_w[2] {X_w[2]} is not 0")

    return [X_w[0], X_w[1]]

def get_transformation_from_body_to_camera(camera):
    T_c_b = np.zeros((4,4))
    if camera == 'voxl':
        T_c_b[:3, :3] = get_rotation_matrix(0, -135, 90)
        camera_translation = np.array([0.09, 0.0, -0.03])
    elif camera == 't265_fisheye1':
        T_c_b[:3, :3] = get_rotation_matrix(0, 180, 90) #https://www.intelrealsense.com/wp-content/uploads/2019/09/Intel_RealSense_Tracking_Camera_Datasheet_Rev004_release.pdf
        camera_translation = np.array([-0.07, -0.03, -0.015])
    elif camera == 't265_fisheye2':
        T_c_b[:3, :3] = get_rotation_matrix(0, 180, 90) #https://www.intelrealsense.com/wp-content/uploads/2019/09/Intel_RealSense_Tracking_Camera_Datasheet_Rev004_release.pdf
        camera_translation = np.array([-0.07, 0.03, -0.015])
    elif camera == 'sim_camera':
        T_c_b[:3, :3] = get_rotation_matrix(0, 180, 90)
        camera_translation = np.array([0.0, 0.0, 0.0])
    else:
        raise ValueError(f"Invalid camera name in get_transformation_from_body_to_camera(): {camera}")
    
    T_c_b[:3, 3] = -np.matmul(T_c_b[:3, :3], camera_translation)
    T_c_b[3, 3] = 1.0
    
    return T_c_b

def get_rotation_matrix(roll, pitch, yaw):
    roll = np.deg2rad(roll)
    pitch = np.deg2rad(pitch)
    yaw = np.deg2rad(yaw)

    R_r = np.array([[1, 0, 0], [0, np.cos(roll), -np.sin(roll)], [0, np.sin(roll), np.cos(roll)]])
    R_p = np.array([[np.cos(pitch), 0, np.sin(pitch)], [0, 1, 0], [-np.sin(pitch), 0, np.cos(pitch)]])
    R_y = np.array([[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]])

    R = np.matmul(R_y, np.matmul(R_p, R_r))
    return R