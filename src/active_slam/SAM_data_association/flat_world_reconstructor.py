# Written by Annika Thomas

import numpy as np
import gtsam
from gtsam import (Cal3_S2, Cal3DS2, DoglegOptimizer,
    GenericProjectionFactorCal3DS2, Marginals,
    NonlinearFactorGraph, PinholeCameraCal3_S2, Point2, Point3,
    Pose3, PriorFactorPoint3, PriorFactorPose3, Rot3, Values, BetweenFactorPose3, symbol_shorthand)
from active_slam.SAM_data_association.utils import transfFromRotAndTransl, compute_3d_position_of_centroid
from active_slam.BlobTracker import BlobTracker

class FlatWorldReconstructor:
    def __init__(self, fx, fy, s, u0, v0, k1, k2, p1, p2, minNumObservations=3):
        self.fx = fx
        self.fy = fy
        self.s = s
        self.u0 = u0
        self.v0 = v0
        self.k1 = k1
        self.k2 = k2
        self.p1 = p1
        self.p2 = p2
        self.minNumObservations = minNumObservations
    
    def reconstruct(self, poses, poseNoises, tracks, nk):

        L = symbol_shorthand.L
        X = symbol_shorthand.X

        self.tracksIncludedInSolution = []
        self.poses = poses

        self.landmarkMAPmeans = dict()

        #K = np.array([[self.fx, self.s, self.u0], [0, self.fy, self.v0], [0, 0, 1]])
        K = nk

        # Triangulate point positions using linear triangulation, to provide an initial guess for MAP
        for track in tracks:

            #print("TRACK: ", track.getTrackId())

            trackId = track.getTrackId()

            framesWhereSeen, _, _ = track.getPxCoordsAndDescriptorsForAllFrames()
            
            camera_centerpoints = []

            estimated_locations = []

            if (len(framesWhereSeen) >= self.minNumObservations):

                for frame in framesWhereSeen:
                    #print("FRAME: ", frame)
                    T = poses[frame]

                    pxCoords, _ = track.getPxcoordsAndDescriptorsForFrame(frame)                   

                    x_px = pxCoords[0]
                    y_px = pxCoords[1]

                    observation = Point2(x_px, y_px)

                    camera_centerpoints.append(T[0:3,3])

                    #print("POSE: ", T)

                    position = compute_3d_position_of_centroid(observation, T, 't265_fisheye1', K)
                    #  if position not None
                    if position is not None:
                        estimated_locations.append(position)

                print("ESTIMATED LOCATIONS: ", estimated_locations)

                # ifestimated locations is not empty,
                if len(estimated_locations) > 0:
                    self.landmarkMAPmeans[trackId] = np.mean(estimated_locations, axis=0)
                

                # try:
                #     dlt_estimate = gtsam.triangulatePoint3(cameras, measurements, rank_tol=1e-9, optimize=True, model=cameraMeasurementNoise)
                #     #print(f"dlt estimate for {track.getTrackId()}={dlt_estimate}")

                #     # Add triangulation result as initial guess
                #     initialValues.insert(L(trackId), dlt_estimate)

                #     # Add projection factors corresponding to this track
                #     for proj_factor in projection_factors:
                #         graph.push_back(proj_factor)

                #     params = gtsam.LevenbergMarquardtParams()
                #     params.setVerbosityLM("TERMINATION")
                #     optimizer = gtsam.LevenbergMarquardtOptimizer(graph, initialValues, params)
                #     result = optimizer.optimize()

                #     initialError = graph.error(initialValues)
                #     finalError = graph.error(result)

                #     marginals = gtsam.Marginals(graph, result)

                #     lmMapMean = result.atPoint3(L(trackId))
                #     lmMapCov = marginals.marginalCovariance(L(trackId))
                
                #     self.landmarkMAPmeans[trackId] = lmMapMean
                #     self.landmarkMAPcovs[trackId] = lmMapCov

                #     # compute object size in meters, assuming it is a round disk
                #     # whose normal is towards the line of observation
                #     sizes_m = []
                #     for camera_centerpoint, frameWhereSeen in zip(camera_centerpoints, framesWhereSeen):
                #         dist = np.linalg.norm(lmMapMean - camera_centerpoint)
                #         size_px = track.getSizeForFrame(frameWhereSeen)
                #         size_m = 2*dist*size_px/(self.fx+self.fy)
                #         sizes_m.append(size_m)

                #     # Store size of track as meters
                #     self.landmarkSizes[trackId] = np.mean(sizes_m)

                #     # Make a note that this track is included in solution
                #     self.tracksIncludedInSolution.append(track)

                # except RuntimeError:
                #     print(f"Track {track.getTrackId()} failed")
                #     # If triangulation fails, runtime error is raised. Silently omit this.
                #     pass

        return (None, None)

    def getReconstructionResults(self):
        return self.landmarkMAPmeans