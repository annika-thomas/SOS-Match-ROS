import numpy as np
from box import Box
import yaml
import time
import os
import logging
import sys
from active_slam.SAM_data_association.Open3dVisualizer import Open3dVisualizer
from scipy.spatial.transform import Rotation as Rot

def readConfig(pathToConfigFile):
    with open(pathToConfigFile, "r") as ymlfile:
        cfg = Box(yaml.safe_load(ymlfile))
    return cfg

def buildProjectionMatrixFromParams(fx, fy, cx, cy, s):
    K = np.eye(3)
    K[0,0] = fx
    K[1,1] = fy
    K[0,2] = cx
    K[1,2] = cy
    K[1,0] = s

    return K

def rotAndTransFromT(T):
    R = T[0:3,0:3]
    t = T[0:3,3]
    return (R,t)

def transfFromRotAndTransl(R,t):
    T = np.eye(4)
    T[0:3,0:3] = R
    T[0:3,3] = t
    return T

def compute_blob_mean_and_covariance(binary_image):

    # Create a grid of pixel coordinates.
    y, x = np.indices(binary_image.shape)

    # Threshold the binary image to isolate the blob.
    blob_pixels = (binary_image > 0).astype(int)

    # Compute the mean of pixel coordinates.
    mean_x, mean_y = np.mean(x[blob_pixels == 1]), np.mean(y[blob_pixels == 1])
    mean = np.array([mean_x, mean_y])

    # Stack pixel coordinates to compute covariance using Scipy's cov function.
    pixel_coordinates = np.vstack((x[blob_pixels == 1], y[blob_pixels == 1]))

    # Compute the covariance matrix using Scipy's cov function.
    covariance_matrix = np.cov(pixel_coordinates)

    return mean, covariance_matrix

def plotErrorEllipse(ax,x,y,covariance,color=None,stdMultiplier=1,showMean=True,idText=None,marker='.'):

    covariance = np.asarray(covariance)

    (lambdas,eigenvectors) = np.linalg.eig(covariance)
    
    t = np.linspace(0,np.pi*2,30)
    
    lambda1 = lambdas[0]
    lambda2 = lambdas[1]
    
    scaledEigenvalue1 = stdMultiplier*np.sqrt(lambda1)*np.cos(t)
    scaledEigenvalue2 = stdMultiplier*np.sqrt(lambda2)*np.sin(t)
    
    scaledEigenvalues = np.vstack((scaledEigenvalue1,scaledEigenvalue2))
    
    ellipseBorderCoords = eigenvectors @ scaledEigenvalues
   
    ellipseBorderCoords_x = x+ellipseBorderCoords[0,:]
    ellipseBorderCoords_y = y+ellipseBorderCoords[1,:]
        
    if (color is not None):
        p = ax.plot(ellipseBorderCoords_x,ellipseBorderCoords_y,color=color)
    else:
        p = ax.plot(ellipseBorderCoords_x,ellipseBorderCoords_y)

    if (showMean):
        ax.plot(x,y,marker,color=p[0].get_color())

    if (idText is not None):
        ax.text(x,y,idText,bbox=dict(boxstyle='square, pad=-0.1',facecolor='white', alpha=0.5, edgecolor='none'),fontsize=8)

def to_homogeneous_coordinates(points):
    # Get the number of points and the dimension of the points
    num_points, dimension = points.shape

    # Create an array of ones with shape (numPoints, 1)
    ones_column = np.ones((num_points, 1), dtype=points.dtype)

    # Concatenate the ones_column to the right of the points array
    homogeneous_points = np.hstack((points, ones_column))

    return homogeneous_points


def from_homogeneous_coordinates(homogeneous_points, scale):
    regular_points = homogeneous_points[:, :-1]

    if (scale):
        scaling_factors = homogeneous_points[:, -1]
        regular_points = regular_points/scaling_factors[:, np.newaxis]

    return regular_points

def estimate_pixel_coordinates_from_pose_t2(points_t1, T1, T2, K):
    # Convert points_t1 to a Numpy array (Nx2 matrix)
    #points_t1 = np.array(points_t1)

    # Extract the rotation matrices from T1 and T2
    R1 = T1[:3, :3]
    R2 = T2[:3, :3]

    # Compute the relative rotation between T1 and T2
    R_rel = np.matmul(R2, R1.T)

    # Invert the camera intrinsic matrix K
    K_inv = np.linalg.inv(K)

    # Convert points_t1 to homogeneous coordinates (add a column of ones)
    points_t1_homogeneous = np.hstack((points_t1, np.ones((points_t1.shape[0], 1))))

    # Create an empty array to store the estimated pixel coordinates in pose T2
    points_t2_estimated = np.empty_like(points_t1)

    # Iterate through each point observed from pose T1
    for i in range(points_t1.shape[0]):
        # Create a 3D point P1 in the camera coordinate system of pose T1
        P1 = np.matmul(K_inv, points_t1_homogeneous[i,:])

        print("P1=")
        print(P1)

        # Transform the 3D point P1 from pose T1 to pose T2
        P2 = np.matmul(R_rel, P1)

        # Project the 3D point P2 back to pixel coordinates in pose T2
        uv2_estimated = np.matmul(K, P2)
        u2_estimated, v2_estimated = uv2_estimated[:2] / uv2_estimated[2]

        # Store the estimated pixel coordinates
        points_t2_estimated[i] = [u2_estimated, v2_estimated]

    return points_t2_estimated



def compute_relative_rotation(T1, T2):
    # Extract the rotation matrices from T1 and T2 (upper-left 3x3 sub-matrices)
    R1 = T1[:3, :3]
    R2 = T2[:3, :3]

    # Compute the relative rotation matrix R_rel
    R_rel = np.dot(R2, R1.T)

    return R_rel

def compute_relative_translation(T1, T2):
    # Extract the translation vectors from T1 and T2
    t1 = T1[:3, 3]
    t2 = T2[:3, 3]

    # Compute the relative translation vector t_rel
    t_rel = t2 - np.dot(T2[:3, :3], t1)

    return t_rel

def getLogger(outputFolderPath=None,filenameBody=None):
    # Returns a logging.logger object that stores output to specified folder (if other than None) with specified filename body
    starttime_str = time.strftime("%Y_%m_%d_%H_%M")
    logger = logging.getLogger(f"blob_logger_{starttime_str}")
    logger.setLevel(logging.INFO)
    loggingFormatter = logging.Formatter('%(asctime)s %(message)s')
    consoleHandler = logging.StreamHandler(sys.stdout)
    consoleHandler.setLevel(logging.INFO)
    consoleHandler.setFormatter(loggingFormatter)
    logger.addHandler(consoleHandler)

    # If output folder path and filename body were given, create a handler for file output.
    if (outputFolderPath is not None and filenameBody is not None):
        if (not os.isdir(outputFolderPath)):
            os.makedirs(outputFolderPath)
        logFilename = os.path.join(outputFolderPath,f"{filenameBody}_{starttime_str}.log")
        fileHandler = logging.FileHandler(logFilename)
        fileHandler.setFormatter(loggingFormatter)
        logger.addHandler(fileHandler)
    
    return logger

def skew_symmetric_matrix(t):
    return np.array([[0, -t[2], t[1]],
                     [t[2], 0, -t[0]],
                     [-t[1], t[0], 0]])

def compute_fundamental_matrix(K, T1, T2, inv=True):
    if (inv):
        T1 = np.linalg.inv(T1)
        T2 = np.linalg.inv(T2)
    K_inv = np.linalg.inv(K)
    R = T2[:3, :3] @ np.linalg.inv(T1[:3, :3])
    t = T2[:3, 3] - T1[:3, 3]
    hat_t = skew_symmetric_matrix(t)
    F = K_inv.T @ hat_t.T @ R @ K_inv
    return F

def blobTouchesBorder(mask):
    sumOfBorderPixelValues = np.sum([mask[0,:]]) + np.sum(mask[-1,:]) + np.sum(mask[:,0]) + np.sum(mask[:,-1])
    if (sumOfBorderPixelValues > 0):
        return True
    else:
        return False

def covSize(cov, squareRoot=True):
    eigenvalues, eigenvectors = np.linalg.eig(cov)
    maxEigenvalue = np.max(eigenvalues)

    if (squareRoot):
        return np.sqrt(maxEigenvalue)
    else:
        return maxEigenvalue


def visualizeMAP(landmarkMeansDict, landmarkCovsDict, posesDict, poseCovsDict):
    vis3d = Open3dVisualizer()

    for key in landmarkMeansDict:
        lm_position = landmarkMeansDict[key]
        lm_cov = landmarkCovsDict[key]

        vis3d.addPoint(lm_position, lm_cov, f"L{key}", [1.0,0.0,0.0])
    
    for key in posesDict:
        T = posesDict[key]
        #T_cov = poseCovsDict[key]
        T_cov = None

        vis3d.addPose(T, T_cov, f"X{key}")
    
    vis3d.render()

def rotate_3d_points_around_z(points, angle_deg):
    # Convert angle to radians
    angle_rad = np.radians(angle_deg)
    
    # Calculate centroid
    centroid = np.mean(points, axis=0)
    
    # Translation to move centroid to origin
    translated_points = points - centroid
    
    # Rotation matrix around z-axis
    rotation_matrix = np.array([[np.cos(angle_rad), -np.sin(angle_rad), 0],
                                [np.sin(angle_rad), np.cos(angle_rad), 0],
                                [0, 0, 1]])
    
    # Rotate the translated points
    rotated_points = np.dot(translated_points, rotation_matrix.T)
    
    # Translate the points back to original position
    final_points = rotated_points + centroid
    
    return final_points

def compute_3d_position_of_centroid(pixel, pose, camera, K):

    # get x_im from pixel
    x_im = np.array([pixel[0], pixel[1], 1.0])

    R = pose[:3, :3]
    t = pose[:3, 3]


    # Transformatoin matrix T^b_w (from world to body)
    T_b_w = np.zeros((4,4))
    #T_b_w[:3, :3] = np.linalg.inv(Rot.from_quat([pose[0], pose[1], pose[2], pose[3]]).as_matrix()) #from_quat() takes as input a quaternion in the form [x, y, z, w]
    T_b_w[:3, :3] = np.linalg.inv(R)
    #T_b_w[:3, 3] = -np.matmul(T_b_w[:3,:3], np.array(pose[0:3]))
    T_b_w[:3, 3] = -np.matmul(T_b_w[:3,:3], t)
    T_b_w[3, 3] = 1.0

    #T_b_w = pose

    # Transformation matrix T^c_b (from body to camera)
    T_c_b = get_transformation_from_body_to_camera('t265_fisheye1')

    # # Transformation matrix T^c_w (from world to camera)
    T_c_w = np.matmul(T_c_b, T_b_w)

    #T_c_w = T_b_w

    # Get T and R from T_c_w
    R_c_w = T_c_w[:3, :3]
    t_c_w = T_c_w[:3, 3]

    #R_c_w = np.matmul(Rot.from_euler('z', 180, degrees=True).as_matrix(), R_c_w)


    # Get X_o from pose (or you can get it by - np.linalg.inv(R_c_w) @ t_c_w)
    # X_o = np.array(pose[0:3]) + camera_translation # which is the same as - np.linalg.inv(R_c_w) @ t_c_w
    X_o = - np.linalg.inv(R_c_w) @ t_c_w

    #print("X_o: ", X_o)

    # compute lambda (depth) in X_w = X_o + lambda * (K * R_c_w)^-1 * x_im using flat earth assumption
    lambda_ = (0 - X_o[2]) / (np.matmul(np.linalg.inv(np.matmul(K, R_c_w)), x_im)[2])

    # compute X_w
    X_w = X_o + lambda_ * np.matmul(np.linalg.inv(np.matmul(K, R_c_w)), x_im)

    if lambda_ < 0:
        # raise ValueError(f"lambda_ {lambda_} < 0 \n pixel: {pixel}")
        #print(f"lambda_ {lambda_} < 0 \n pixel: {pixel}")
        return 
    
    if abs(X_w[2]) > 1e-2:
        raise ValueError(f"X_w[2] {X_w[2]} is not 0")

    return [X_w[0], X_w[1], 0]

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
    
    R_c_w_flip = np.array([[-1, 0, 0], [0, -1, 0], [0, 0, 1]])

    T_c_b[:3, :3] = np.matmul(R_c_w_flip, T_c_b[:3, :3])
    
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
