import numpy as np
import gtsam
from gtsam import (Cal3_S2, Cal3DS2, DoglegOptimizer,
    GenericProjectionFactorCal3DS2, Marginals,
    NonlinearFactorGraph, PinholeCameraCal3_S2, Point2, Point3,
    Pose3, PriorFactorPoint3, PriorFactorPose3, Rot3, Values, BetweenFactorPose3, symbol_shorthand)
from active_slam.SAM_data_association.utils import transfFromRotAndTransl
from active_slam.SAM_data_association.BlobTrackerRT import BlobTracker
import scipy.optimize as opt

class BlobSfm:
    def __init__(self, fx, fy, s, u0, v0, k1, k2, p1, p2, pixelMsmtNoiseStd, huberParam=1.0, minNumObservations=3, priorConfig=None):
        self.fx = fx
        self.fy = fy
        self.s = s
        self.u0 = u0
        self.v0 = v0
        self.k1 = k1
        self.k2 = k2
        self.p1 = p1
        self.p2 = p2
        self.pixelMsmtNoiseStd = pixelMsmtNoiseStd
        self.huberParam = huberParam
        self.K_gtsam = Cal3DS2(self.fx,self.fy,self.s,self.u0,self.v0,self.k1,self.k2,self.p1,self.p2)
        self.minNumObservations = minNumObservations
        self.priorConfig = priorConfig
    
    def reconstruct(self, poses, poseNoises, tracks):

        L = symbol_shorthand.L
        X = symbol_shorthand.X

        #cameraMeasurementNoise_noHuber = gtsam.noiseModel.Isotropic.Sigma(2, self.pixelMsmtNoiseStd)
        #cameraMeasurementNoise = gtsam.noiseModel.Robust.Create(gtsam.noiseModel.mEstimator.Huber.Create(self.huberParam ), cameraMeasurementNoise_noHuber)
        cameraMeasurementNoise = gtsam.noiseModel.Isotropic.Sigma(2, self.pixelMsmtNoiseStd)

        self.tracksIncludedInSolution = []
        self.poses = poses

        self.landmarkMAPmeans = dict()
        self.landmarkMAPcovs = dict()
        self.landmarkSizes = dict()

        # Triangulate point positions using linear triangulation, to provide an initial guess for MAP
        for track in tracks:

            trackId = track.getTrackId()

            graph = NonlinearFactorGraph()
            initialValues = Values()

            framesWhereSeen, _, _ = track.getPxCoordsAndDescriptorsForAllFrames()
            
            gtsam_cams = []
            gtsam_observations = []
            projection_factors = []

            camera_centerpoints = []

            if (len(framesWhereSeen) >= self.minNumObservations):

                for frame in framesWhereSeen:
                    T = poses[frame]
                    T_gtsam = Pose3(T)

                    factor = gtsam.NonlinearEqualityPose3(X(frame), T_gtsam)
                    graph.push_back(factor)
                    initialValues.insert(X(frame), T_gtsam)

                    cam = gtsam.PinholeCameraCal3DS2(T_gtsam, self.K_gtsam) #TODO: Make fisheye camera model
                    #cam = gtsam.Cal3DS2(self.fx,self.fy,self.s,self.u0,self.v0,self.k1,self.k2,self.p1,self.p2)


                    pxCoords, _ = track.getPxcoordsAndDescriptorsForFrame(frame)                   

                    x_px = pxCoords[0]
                    y_px = pxCoords[1]

                    observation = Point2(x_px, y_px)

                    gtsam_cams.append(cam)
                    gtsam_observations.append(observation)

                    # Besides triangulating an initial guess, assemble a set of projection factors in case it turns out
                    # that triangulation succeeds for this track
                    proj_factor = GenericProjectionFactorCal3DS2(observation, cameraMeasurementNoise, X(frame), L(track.getTrackId()), self.K_gtsam)#, True, True)
                    projection_factors.append(proj_factor)

                    camera_centerpoints.append(T[0:3,3])

                measurements = gtsam.Point2Vector(gtsam_observations)
                cameras = gtsam.gtsam.CameraSetCal3DS2(gtsam_cams)

                #print("there are ", len(cameras), "cameras and ", len(measurements), "measurements")

                try:
                    #dlt_estimate = gtsam.triangulatePoint3(cameras, measurements, rank_tol=1e-9, optimize=True, model=cameraMeasurementNoise)
                    #print("track", track.getTrackId())
                    #print("cameras", cameras)
                    #print("measurements", measurements)
                    # print(cameras)
                    # print(measurements)
                    #dlt_estimate = tryDLT(cameras, measurements, cameraMeasurementNoise, track)
                    dlt_estimate = triangulate_2D(cameras, measurements, cameraMeasurementNoise, track)
                    #print("dlt estimate: ", dlt_estimate)

                    #print("cameras" , cameras)

                    # if dlt_estimate is None:
                    #     try_all_combinations(cameras, measurements, cameraMeasurementNoise, track)

                    while dlt_estimate is None:
                        #print("retrying")
                        # try different combinations of the cameras and measurements
                        estimates = []
                        for i in range(len(cameras)):
                            # # try first, last and middle of cameras and measurements
                            # cameras_subset = []
                            # measurements_subset = []
                            # start_idx = i
                            # end_idx = len(cameras)-1
                            # middle_idx = (i+end_idx)//2
                            # indices = [start_idx, middle_idx, end_idx]

                            # cameras_subset = [cameras[i] for i in indices]
                            # measurements_subset = [measurements[i] for i in indices]

                            # camera_set = gtsam.gtsam.CameraSetCal3DS2
                            # for camera in cameras_subset:
                            #     camera_set.append(camera)

                            # print("cameras_subset", cameras_subset)

                            # dlt_estimate = tryDLT(cameras_subset, measurements_subset, cameraMeasurementNoise, track)

                            # print("cameras_subset", cameras_subset)
                            # print("measurements_subset", measurements_subset)

                            for j in range(i+5, len(cameras)):
                                dlt_estimate = triangulate_2D(cameras[i:j], measurements[i:j], cameraMeasurementNoise, track)
                                if dlt_estimate is not None:
                                    estimates.append(dlt_estimate)
                            if dlt_estimate is not None:
                                estimates.append(dlt_estimate)
                        dlt_estimate = None
                        if len(estimates) > 0:
                            dlt_estimate = np.mean(estimates, axis=0)

                        # if magnitude of z component of dlt_estimate is greater than 10, set to None
                        if dlt_estimate is not None and abs(dlt_estimate[2]) > 10:
                            dlt_estimate = None
                            #print("z component too large")
                        break
                        # break
                    
                    if dlt_estimate is None:
                        #print("track", track.getTrackId(), "failed")
                        continue
                    #dlt_estimate = gtsam.triangulatePoint3(cameras, measurements, rank_tol=1e-5, optimize=True, model=cameraMeasurementNoise)
                    #print(f"dlt estimate for {track.getTrackId()}={dlt_estimate}")

                    # Add triangulation result as initial guess
                    initialValues.insert(L(trackId), dlt_estimate)

                    # Add projection factors corresponding to this track
                    for proj_factor in projection_factors:
                        graph.push_back(proj_factor)

                    params = gtsam.LevenbergMarquardtParams()
                    params.setVerbosityLM("TERMINATION")
                    optimizer = gtsam.LevenbergMarquardtOptimizer(graph, initialValues, params)
                    result = optimizer.optimize()

                    initialError = graph.error(initialValues)
                    finalError = graph.error(result)

                    marginals = gtsam.Marginals(graph, result)

                    lmMapMean = result.atPoint3(L(trackId))
                    lmMapCov = marginals.marginalCovariance(L(trackId))

                    self.landmarkMAPmeans[trackId] = lmMapMean
                    self.landmarkMAPcovs[trackId] = lmMapCov

                    # compute object size in meters, assuming it is a round disk
                    # whose normal is towards the line of observation
                    sizes_m = []
                    for camera_centerpoint, frameWhereSeen in zip(camera_centerpoints, framesWhereSeen):
                        dist = np.linalg.norm(lmMapMean - camera_centerpoint)
                        size_px = track.getSizeForFrame(frameWhereSeen)
                        size_m = 2*dist*size_px/(self.fx+self.fy)
                        sizes_m.append(size_m)

                    # Store size of track as meters
                    self.landmarkSizes[trackId] = np.mean(sizes_m)

                    # Make a note that this track is included in solution
                    self.tracksIncludedInSolution.append(track)

                except RuntimeError:
                    print(f"Track {track.getTrackId()} failed")
                    # If triangulation fails, runtime error is raised. Silently omit this.
                    # if len(cameras) > 5:
                    #     self.reconstruct(poses, poseNoises, tracks)

        return (None, None)

    def getReconstructionResults(self):
        return self.landmarkMAPmeans, self.landmarkMAPcovs, self.poses, None, self.landmarkSizes

def runSfmFromCacheFile(pathToCache, cameraConfig, priorConfig=None):

    pixelMsmtNoiseStd = 3.0
    numObservationsRequiredForTriang = 3
    huberParam = 2.0

    blobSfm = BlobSfm(cameraConfig.projection_parameters.fx, cameraConfig.projection_parameters.fy, cameraConfig.projection_parameters.s,
        cameraConfig.projection_parameters.cx, cameraConfig.projection_parameters.cy, cameraConfig.distortion_parameters.k1,
        cameraConfig.distortion_parameters.k2, cameraConfig.distortion_parameters.p1, cameraConfig.distortion_parameters.p2, pixelMsmtNoiseStd,
        huberParam, numObservationsRequiredForTriang, priorConfig)

    blobTracker = BlobTracker(None, None, None, None, None, None)
    blobTracker.load(pathToCache)

    poseHist = blobTracker.getPoseHistory()
    poseNoiseHist = blobTracker.getPoseNoiseHistory()
    
    rot_std_rad = np.deg2rad(2)
    transl_std_m = 0.5

    # todo: remove this hack
    for key in poseNoiseHist:
        poseNoiseHist[key] = np.array([rot_std_rad, rot_std_rad, rot_std_rad, transl_std_m, transl_std_m, transl_std_m])

    tracks = blobTracker.getFeatureTracks()

    (initialError, finalError) = blobSfm.reconstruct(poseHist, poseNoiseHist, tracks)
    #print(f"Initial error={initialError}, final error={finalError}")

    landmarkMAPmeans, landmarkMAPcovs, poseMAPmeans, poseMAPcovs, landmarkSizes = blobSfm.getReconstructionResults()
    return blobTracker, landmarkMAPmeans, landmarkMAPcovs, poseMAPmeans, poseMAPcovs, landmarkSizes, tracks

def tryDLT(cameras, measurements, cameraMeasurementNoise, track):
    try:
        dlt_estimate = gtsam.triangulatePoint3(cameras, measurements, rank_tol=1e-9, optimize=True, model=cameraMeasurementNoise)
        #print(f"track {track.getTrackId()} succeeded!")
    except RuntimeError:
        #print(f"Track {track.getTrackId()} failed")
        dlt_estimate = None

    return dlt_estimate


def triangulate_2D(cameras, measurements, cameraMeasurementNoise, track):
    # Objective function to minimize (sum of squared reprojection errors)
    def objective(x):
        error = 0
        for camera, measurement in zip(cameras, measurements):
            # Project the 3D point (x[0], x[1], 0) onto the image plane
            projected = camera.project(np.array([x[0], x[1], 0]))
            # Add the squared error
            error += np.sum((projected - measurement) ** 2)
        return error

    # Initial guess for x and y
    initial_guess = [0, 0]
    result = opt.minimize(objective, initial_guess)

    if result.success:
        return np.array([result.x[0], result.x[1], 0])  # Append the known z-coordinate
    else:
        raise RuntimeError("Optimization failed")
