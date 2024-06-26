# Written by Annika Thomas

from abc import ABC, abstractmethod
import numpy as np
from active_slam.SAM_data_association.SamModel import SamModel
from active_slam.SAM_data_association.utils import compute_blob_mean_and_covariance, covSize, blobTouchesBorder, plotErrorEllipse
import cv2
from scipy.optimize import linear_sum_assignment
import time
import pickle
import matplotlib.pyplot as plt
import scipy
import time

class CameraModel:
    def __init__(self, fx, fy, s, u0, v0, k1, k2, p1, p2):
        self.fx = fx
        self.fy = fy
        self.s = s
        self.u0 = u0
        self.v0 = v0
        self.k1 = k1
        self.k2 = k2
        self.p1 = p1
        self.p2 = p2

class FeatureDetectorDescriptorAndComparer(ABC):
    def __init__(self):
        return
    
    @abstractmethod
    def detectAndDescribe(self, image):
        # Should return a tuple of lists where the first is a list of pixel coordinates of the features (as Numpy arrays of shape (,2))
        # and the second is a list of corresponding feature descriptors (type unspecified)
        pass

    @abstractmethod
    def scoreSimilarity(self, descriptor1, descriptor2):
        # Should return similarity estimated from the two given descriptors. 1.0 corresponds to high similarity and 0.0 to dissimilarity
        pass

    @abstractmethod
    def visualize(self, ax, image, detections, descriptions):
        # Should visualize given list of detections and descriptions on Matplotlib axis object ax
        pass

class FeatureTrack:
    def __init__(self, trackId: int):
        self.trackId = trackId
        self.framesWhereSeen = []
        self.pxCoords = []
        self.descriptors = []
        self.sizes = []
    
    def addDetection(self, frameIdx, pxCoords, descriptor, size):
        self.framesWhereSeen.append(frameIdx)
        self.pxCoords.append(pxCoords)
        self.descriptors.append(descriptor)
        self.sizes.append(size)

    def getFramesWhereDetected(self):
        return self.framesWhereSeen

    def getLatestFrameWhereDetected(self):
        return self.framesWhereSeen[-1]
    
    def trackSeenInFrame(self, idx):
        if idx in self.framesWhereSeen:
            return True
        return False

    def getPxcoordsAndDescriptorsForFrame(self, idx):
        for obs_idx, frameWhereSeen in enumerate(self.framesWhereSeen):
            if frameWhereSeen == idx:
                return self.pxCoords[obs_idx], self.descriptors[obs_idx]
        return None

    def getPxCoords(self, idx):
        for obs_idx, frameWhereSeen in enumerate(self.framesWhereSeen):
            if frameWhereSeen == idx:
                return self.pxCoords[obs_idx]
        return None
    
    def getPxCoordsAndDescriptorsForAllFrames(self):
        return self.framesWhereSeen, self.pxCoords, self.descriptors

    def getTrackId(self):
        return self.trackId
    
    def getSizeStatistics(self):
        return np.mean(self.sizes), np.std(self.sizes)
    
    def getSizes(self):
        return self.sizes

    def getSizeForFrame(self, idx):
        for obs_idx, frameWhereSeen in enumerate(self.framesWhereSeen):
            if (frameWhereSeen == idx):
                return self.sizes[obs_idx]
        return None

class BlobTracker:
    def __init__(self, detectorDescriptorAndComparer: FeatureDetectorDescriptorAndComparer, fTestMaxVal: float, matchingScoreLowerLimit: float, numKfsToGoBackWhenFindingCorrespondences: int, logger=None):
        self.totalTravel = 0
        self.T_prev = None
        self.travelAtLatestKeyframe = 0
        self.latestKeyframeIndex = None
        self.logger = logger
        self.numKfsToGoBackWhenFindingCorrespondences = numKfsToGoBackWhenFindingCorrespondences
        self.nextLandmarkId = 0
        self.ddc = detectorDescriptorAndComparer
        self.poseHistory = dict()
        self.poseNoiseHistory = dict()
        self.tracks = []
        self.keyframeImages = dict()
        self.detectionHistory = dict()
        self.detectionSizeHistory = dict()
        self.descriptorHistory = dict()
        self.frameNamesHistory = dict()
        self.fTestMaxVal = fTestMaxVal
        self.matchingScoreLowerLimit = matchingScoreLowerLimit
        self.numTracks = 0
        self.frameSiftFeatures = dict()

        return

    def getKeyframeNames(self):
        return self.frameNamesHistory
    
    def findFundamentalMatViaSiftPoints(self, image_bgr_prev, image_bgr, imageFrame, prevImageFrame):
        sift = cv2.SIFT.create()

        sift_feats_start_time = time.time()

        framesSiftFeatures = self.frameSiftFeatures

        if imageFrame in reversed(framesSiftFeatures):
            kp1, des1 = framesSiftFeatures[imageFrame]
            #print(f"Found frame {imageFrame} in dictionary of SIFT features in {(search_end_time-search_start_time)*1000:.2f} ms")

        else:
            sift = cv2.SIFT.create()
            start_time_new = time.time()
            kp1, des1 = sift.detectAndCompute(image_bgr, None)
            end_time_new = time.time()
            framesSiftFeatures[imageFrame] = [kp1, des1]
            #print(f"Adding frame {imageFrame} to dictionary of SIFT features in {(end_time_new-start_time_new)*1000:.2f} ms")

        if prevImageFrame in reversed(framesSiftFeatures):
            kp2, des2 = framesSiftFeatures[prevImageFrame]

        else:
            kp2, des2 = sift.detectAndCompute(image_bgr_prev, None)
            framesSiftFeatures[prevImageFrame] = [kp2, des2]
            #rint(f"Adding frame {prevImageFrame} to dictionary of SIFT features")

        sift_feats_end_time = time.time()

        #print(f"SIFT feature detection took {(sift_feats_end_time-sift_feats_start_time)*1000:.2f} ms")

        FLANN_INDEX_KDTREE = 0
        index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
        search_params = dict(checks = 50)

        sift_match_start_time = time.time()

        flann = cv2.FlannBasedMatcher(index_params, search_params)

        if (des1 is None or des2 is None):
            # If detecting descriptors failed, return None as all parameters
            return None, None, None

        matches = flann.knnMatch(des1,des2,k=2)
        #print(f"Number of matches: {len(matches)}")

        # store all the good matches as per Lowe's ratio test.
        good = []
        for m,n in matches:
            if m.distance < 0.7*n.distance:
                good.append(m)

        p1 = np.float32([ kp1[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
        p2 = np.float32([ kp2[m.trainIdx].pt for m in good ]).reshape(-1,1,2)

        sift_match_end_time = time.time()

        #print(f"SIFT matching took {(sift_match_end_time-sift_match_start_time)*1000:.2f} ms")

        F_matrix_start_time = time.time()

        #E, mask = cv2.findEssentialMat(p1, p2, sbcf.K, cv2.RANSAC, 0.999, 1.0)
        F, mask = cv2.findFundamentalMat(p1,p2, method=cv2.FM_RANSAC, ransacReprojThreshold=0.1)

        F_matrix_end_time = time.time()

        #print(f"Fundamental matrix computation took {(F_matrix_end_time-F_matrix_start_time)*1000:.2f} ms")

        # findFundamentalMat sometimes returns three possible fundamental matrices. If this is the case, just take the first one (silently)
        # See https://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html#findfundamentalmat
        if F is not None:
            F = F[0:3,0:3]

        mask = np.array(np.squeeze(mask),dtype=bool)
        p1 = np.squeeze(p1)
        p2 = np.squeeze(p2)

        masked_p1 = p1[mask,:]
        masked_p2 = p2[mask,:]

        changeInPxCoords = np.linalg.norm(masked_p2-masked_p1, axis=1)
        
        meanChangeInPxCoords = np.mean(changeInPxCoords)
        stdDevOfChangeInPxCoords = np.std(changeInPxCoords)

        return F, meanChangeInPxCoords, stdDevOfChangeInPxCoords

    def handleNewFrame(self, image, T, pose_noise_sigmas, frameName):
        # image: rgb image
        # T: pose in odometry frame
        # pose_noise_sigmas: pose noise
        # frameName: string stating the name of this frame (used for visualizations and debugging only)

        
        # handle new keyframe
        frameIsConsideredKeyframe = True

        if (self.latestKeyframeIndex is None):
            self.latestKeyframeIndex = 0
        else:
            self.latestKeyframeIndex += 1

        self.poseHistory[self.latestKeyframeIndex] = T
        self.poseNoiseHistory[self.latestKeyframeIndex] = pose_noise_sigmas
        self.keyframeImages[self.latestKeyframeIndex] = image
        self.frameNamesHistory[self.latestKeyframeIndex] = frameName

        #self.log(f"Running algorithm with frame idx {self.latestKeyframeIndex}, total travel={self.totalTravel}")

        ddc_start_time = time.time()

        keypoints, descriptors, sizes = self.ddc.detectAndDescribe(image)

        ddc_end_time = time.time()

        #self.log(f"Feature detection and description took {(ddc_end_time-ddc_start_time)*1000:.2f} ms")

        self.detectionHistory[self.latestKeyframeIndex] = keypoints
        self.descriptorHistory[self.latestKeyframeIndex] = descriptors
        self.detectionSizeHistory[self.latestKeyframeIndex] = sizes

        #self.log(f"At frame {self.latestKeyframeIndex}: Number of segments found: {len(keypoints)}")

        self.travelAtLatestKeyframe = self.totalTravel

        findCorrespondences_start_time = time.time()
    
        # Find correspondences
        self.findCorrespondences(keypoints, descriptors, sizes)

        findCorrespondences_end_time = time.time()

        #self.log(f"Finding correspondences took {(findCorrespondences_end_time-findCorrespondences_start_time)*1000:.2f} ms")

        self.T_prev = T

        return frameIsConsideredKeyframe

    def findCorrespondences(self, new_keypoints, new_descriptors, new_sizes):
        rangeStart = np.max((0,self.latestKeyframeIndex-self.numKfsToGoBackWhenFindingCorrespondences))

        # First, find "front" of each feature in history ("front" = the latest frame in which that track ID was detected;
        # include only track IDs visible in the set of frames that are included in the search window)
        tracksToMatch = []
        lastObservationsForTracks = []

        gettingLatestFrameWhereDetected_start_time = time.time()
        
        for track in self.tracks:
            latestFrameWhereDetected = track.getLatestFrameWhereDetected()
            #print(f"track {track.getTrackId()} was last observed at frame {latestFrameWhereDetected}")
            if (latestFrameWhereDetected >= rangeStart):
                tracksToMatch.append(track)
                lastObservationsForTracks.append(latestFrameWhereDetected)

        gettingLatestFrameWhereDetected_end_time = time.time()

        #self.log(f"Getting latest frame where detected took {(gettingLatestFrameWhereDetected_end_time-gettingLatestFrameWhereDetected_start_time)*1000:.2f} ms")

        # Go through every association of currently observed points and tracks we consider for matching.
        framesWithObservations = np.unique(lastObservationsForTracks)
        currentImage_rgb = self.keyframeImages[self.latestKeyframeIndex]
        currentImage_bgr = cv2.cvtColor(currentImage_rgb, cv2.COLOR_RGB2BGR)

        trackAndFrameCombinations = []

        similarityMatrix = None
        pixelDistanceMatrix = None

        siftFundTime = 0
        numFramesWithObservations = len(framesWithObservations)
        #print(f"Number of frames with observations: {numFramesWithObservations}")

        for frame_id in framesWithObservations:
            previousImage_bgr = cv2.cvtColor(self.keyframeImages[frame_id], cv2.COLOR_RGB2BGR)
            prevImageFrame = frame_id
            imageFrame = self.latestKeyframeIndex

            #print(f"Comparing frame {prevImageFrame} to frame {imageFrame}")

            siftFundStart = time.time()

            # Find fundamental matrix (TODO: replace with fundamental matrix computed from VIO poses and camera calibration matrix)
            F, meanChangeInPxCoords, stdDevOfChangeInPxCoords = self.findFundamentalMatViaSiftPoints(previousImage_bgr, currentImage_bgr, imageFrame, prevImageFrame)

            #print(f"F from SIFT: {F}")

            siftFundEnd = time.time()

            #print(f"Finding fundamental matrix of {frame_id} took {(siftFundEnd-siftFundStart)*1000:.2f} ms")

            siftFundTime += siftFundEnd-siftFundStart

            if (F is None):
                break

            # Loop through each track in history. If it was last observed in frame_id, consider this as a
            # potential association. Record f test value and descriptor similarity value.
            for track in tracksToMatch:
                if track.getLatestFrameWhereDetected() == frame_id:
                    #print(f"comparing to track {track.getTrackId()} which was last detected in {frame_id}")
                    track_px_coords, track_desc = track.getPxcoordsAndDescriptorsForFrame(frame_id)

                    track_px_coords_homog = np.ones((3,))
                    track_px_coords_homog[0:2] = track_px_coords

                    for newKeypointIdx, (newKeypoint, newDescriptor) in enumerate(zip(new_keypoints, new_descriptors)):

                        newKeypoint_homog = np.ones((3,))
                        newKeypoint_homog[0:2] = newKeypoint

                        try:
                            f_test_val = np.abs(newKeypoint_homog.T @ F @ track_px_coords_homog)
                        except ValueError:
                            print("Captured a value error!")
                            print("newKeypoint_homog shape=")
                            print(newKeypoint_homog.shape)
                            print("track_px_coords_homog shape=")
                            print(track_px_coords_homog.shape)
                            print("F shape=")
                            print(F.shape)
                            print("F=")
                            print(F)
                            assert False

                        # If f_test_val is too large, just skip considering this as an association (due to clear geometric inconsistency)
                        if (f_test_val < self.fTestMaxVal):

                            # Compute similarity
                            descriptor_similarity_val = self.ddc.scoreSimilarity(newDescriptor, track_desc)

                            # Compute change in position in pixels
                            change_of_position_px = np.linalg.norm(track_px_coords-newKeypoint)

                            pixelPositionChangeMahalanobisDist = np.abs(change_of_position_px-meanChangeInPxCoords)/stdDevOfChangeInPxCoords

                            trackAndFrameCombination = track, frame_id

                            if not trackAndFrameCombination in trackAndFrameCombinations:
                                # This track + frame combination is not in the list of combinations.
                                # Record this as one of the trackAndFrameCombinations we are considering for association.
                                trackAndFrameCombinations.append(trackAndFrameCombination)
                                # Add a row in the similarity matrix that represtents this combination.
                                if (similarityMatrix is not None):
                                    # If this was not the first one, add a row of zeroes.
                                    similarityMatrix = np.vstack((similarityMatrix, np.zeros((1,len(new_keypoints)))))
                                    pixelDistanceMatrix = np.vstack((pixelDistanceMatrix, np.zeros((1,len(new_keypoints)))))
                                else:
                                    # If this was the first one, initialize.
                                    similarityMatrix = np.zeros((1,len(new_keypoints)))
                                    pixelDistanceMatrix = np.zeros((1,len(new_keypoints)))
                            
                            # Define similarity using descriptor and Mahalanobis distance in pixel coordinates
                            if (pixelPositionChangeMahalanobisDist < 2.0): # CHANGED: WAS 2.0
                                similarity = descriptor_similarity_val
                            else:
                                similarity = 0

                            # Get index of track+frame combination
                            trackframeindex = trackAndFrameCombinations.index(trackAndFrameCombination)
                            # Record computed similarity
                            similarityMatrix[trackframeindex, newKeypointIdx] = similarity
                            # Record pixel distance as number of standard deviations from mean
                            pixelDistanceMatrix[trackframeindex, newKeypointIdx] = pixelPositionChangeMahalanobisDist
                        #else:
                        #    self.log(f"At frame {frame_id}, for point at pixel coordinates {newKeypoint}, track {track.getTrackId()} was considered infeasible (f_test_val={f_test_val:.3f}=")

        #self.log(f"Finding fund matrix took {siftFundTime*1000:.2f} ms")

        if (similarityMatrix is not None):
            h_simMat, w_simMat = similarityMatrix.shape
        else:
            h_simMat = 0
        
        newKeypointIndices = np.arange(len(new_keypoints))
        newKeypointIndices_withMatchesToPreviousTracks = []
        newKeypointIndices_filteredBasedOnScoreAfterHung = []

        # If any tracked features survived the fundamental matrix filtering step
        if (h_simMat > 0):
            # For each feature, find which track would provide the best matching score, individually. If this score is
            # below a limit, just assume that the feature is new.
            colsToKeep = []
            for newKeypointIdx in newKeypointIndices:
                maxScore = np.max(similarityMatrix[:,newKeypointIdx])
                
                if maxScore > self.matchingScoreLowerLimit:
                    # At least one of the scores in the association matrix shows a high score for this association hypothesis.
                    # Keep it as one of the potential associations.
                    colsToKeep.append(newKeypointIdx)

            # Drop the columns in which we detected low scores.
            newKeypointIndices_withMatchesToPreviousTracks = newKeypointIndices[colsToKeep]
            similarityMatrix_withMatchesToPreviousTracks = similarityMatrix[:,colsToKeep]
            pixelDistanceMatrix_withMatchesToPreviousTracks = pixelDistanceMatrix[:,colsToKeep]

            # For the features that had high maximum matching scores, find best assignments using the Hungarian algorithm.
            start_time = time.time()
            row_ind, col_ind = linear_sum_assignment(similarityMatrix_withMatchesToPreviousTracks, maximize=True)
            #print(f"Hungarian algorithm took {(time.time()-start_time)*1000:.2f} ms")

            # Based on output of Hungarian algorithm, add detections to existing tracks.
            for trackframeindex, ci in zip(row_ind, col_ind):
                newKeypointIdx = newKeypointIndices_withMatchesToPreviousTracks[ci]
                track, frame_id = trackAndFrameCombinations[trackframeindex]

                # Do a final check of similarity, in case the Hungarian algorithm lead to clearly faulty associations.
                score = similarityMatrix_withMatchesToPreviousTracks[trackframeindex,ci]

                # Also check if movement in pixels was suspiciously large.
                pxdist_scaled = pixelDistanceMatrix_withMatchesToPreviousTracks[trackframeindex,ci]

                # Compute the probability that pixels moved this much.
                pxdist_probability = (1 - scipy.stats.norm.cdf(pxdist_scaled))*2

                # Use probability information from pixel movement for scaling score.
                score *= pxdist_probability

                if (score > self.matchingScoreLowerLimit and pxdist_scaled < 2.0): # todo: add pxdist_scaled as parameter to this class!
                    track.addDetection(self.latestKeyframeIndex, new_keypoints[newKeypointIdx], new_descriptors[newKeypointIdx], new_sizes[newKeypointIdx])
                    #self.log(f"Adding detection of existing track {track.getTrackId()} at keyframe {self.latestKeyframeIndex} at pixel coordinates {new_keypoints[newKeypointIdx]}, score={score:.3f}, pixel dist={pxdist_scaled:.2f}, pxdist_prob={pxdist_probability:.2f}")
                else:
                    newKeypointIndices_filteredBasedOnScoreAfterHung.append(newKeypointIdx)

        # Add all features that are considered new as new tracks.
        for newKeypointIdx in newKeypointIndices:

            # Compute what was the maximum similarity score for this keypoint, before the Hungarian algorithm.
            if (similarityMatrix is not None):
                maxSimilarityScore = np.max(similarityMatrix[:, newKeypointIdx])
            else:
                maxSimilarityScore = 0.0

            if (newKeypointIdx not in newKeypointIndices_withMatchesToPreviousTracks or newKeypointIdx in newKeypointIndices_filteredBasedOnScoreAfterHung and maxSimilarityScore < self.matchingScoreLowerLimit):
                self.numTracks += 1
                ft = FeatureTrack(self.numTracks)
                ft.addDetection(self.latestKeyframeIndex, new_keypoints[newKeypointIdx], new_descriptors[newKeypointIdx], new_sizes[newKeypointIdx])
                #self.log(f"Adding detection of new track {ft.getTrackId()} at keyframe {self.latestKeyframeIndex} at pixel coordinates {new_keypoints[newKeypointIdx]}")
                #print("frame id: ", self.latestKeyframeIndex)
                self.tracks.append(ft)

    def getFeatureTracks(self):
        return self.tracks

    def getFrameImage(self, frameId):
        return self.keyframeImages[frameId]

    def visualizeFrame(self, frameId, ax):
        image = self.keyframeImages[frameId]

        detections = self.detectionHistory[frameId]
        descriptions = self.descriptorHistory[frameId]
        sizes = self.detectionSizeHistory[frameId]
        frameName = self.frameNamesHistory[frameId]

        cm = plt.get_cmap('tab20')

        # Visualize track IDs
        for track in self.tracks:
            if track.trackSeenInFrame(frameId):
                trackId = track.getTrackId()

                size_mean, size_std = track.getSizeStatistics()

                px_coords, descriptor = track.getPxcoordsAndDescriptorsForFrame(frameId)

                color = cm(trackId%20)
                
                props = dict(facecolor=color, alpha=0.5, linewidth=0)

                ax.text(px_coords[0],px_coords[1],f"{trackId}", bbox=props)

                circle1 = plt.Circle((px_coords[0],px_coords[1]), size_mean+size_std, color='r', linewidth=1.0, fill=False)
                circle2 = plt.Circle((px_coords[0],px_coords[1]), size_mean-size_std, color='r', linewidth=1.0, fill=False)
                ax.add_patch(circle1)
                ax.add_patch(circle2)

                #plotErrorEllipse(ax, px_coords[0], px_coords[1], descriptor, color=color, stdMultiplier=2)

        # Visualize detections and descriptions
        self.ddc.visualize(ax, image, detections, descriptions)
        ax.text(20,40,frameName)

    def visualizeFrameROS(self, frameId):
        image = self.keyframeImages[frameId]

        detections = self.detectionHistory[frameId]
        descriptions = self.descriptorHistory[frameId]
        sizes = self.detectionSizeHistory[frameId]
        frameName = self.frameNamesHistory[frameId]

        cm = plt.get_cmap('tab20')

        # Draw track IDs and ellipses using OpenCV
        for track in self.tracks:
            if track.trackSeenInFrame(frameId):
                trackId = track.getTrackId()
                size_mean, size_std = track.getSizeStatistics()
                px_coords, descriptor = track.getPxcoordsAndDescriptorsForFrame(frameId)

                color = tuple([int(255*x) for x in cm(trackId%20)[:3]])  # Convert matplotlib color to OpenCV BGR color

                # Draw text and ellipses
                cv2.putText(image, str(trackId), (int(px_coords[0]), int(px_coords[1])), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                cv2.circle(image, (int(px_coords[0]), int(px_coords[1])), int(size_mean + size_std), color, 2)
                cv2.circle(image, (int(px_coords[0]), int(px_coords[1])), int(size_mean - size_std), color, 2)

        return image

    def log(self,string):
        if (self.logger is not None):
            self.logger.info(string)
    
    def save(self, filename):
        # Save class to pickle
        with open(filename, "wb") as f:
            pickle.dump(self.__dict__, f)
    
    def load(self, filename):

        with open(filename, 'rb') as f:
            dataPickle = pickle.load(f)

        self.__dict__ = dataPickle

    def getPoseHistory(self):
        return self.poseHistory
    
    def getPoseNoiseHistory(self):
        return self.poseNoiseHistory
    
def findFundamentalMatViaPoses(T_prev, T):
    # T_prev and T are 4x4 pose matrices in the same reference frame
    # K is the camera calibration matrix
    # The dimensions of K are 3x3

    #print("T:", T)

    #print("T_prev:", T_prev)

    # k matrix
    K = np.array([[320, 0.0, 320],
                  [0.0, 320, 240],
                  [0.0, 0.0, 1.0]])
    
    # Extract rotation matrices and translation vectors
    R1, t1 = T[:3, :3], T[:3, 3]
    R2, t2 = T_prev[:3, :3], T_prev[:3, 3]

    # Compute relative rotation and translation
    R = R2 @ R1.T
    t = t2 - R @ t1

    # Skew-symmetric matrix of t
    t_x = np.array([[0, -t[2], t[1]],
                    [t[2], 0, -t[0]],
                    [-t[1], t[0], 0]])

    # Compute the essential matrix
    E = t_x @ R

    #print("E:", E)
    # Compute essential matrix
    #E = np.linalg.inv(T_prev) @ T

    # Compute fundamental matrix
    F = np.linalg.inv(K).T * np.mat(E) * np.linalg.inv(K)

    changeInPxCoords = 0
    stdDevOfChangeInPxCoords = 0

    return F, changeInPxCoords, stdDevOfChangeInPxCoords
    
def findFundamentalMatViaORBPoints(image_bgr_prev, image_bgr):

    orb = cv2.ORB_create()
    kp1, des1 = orb.detectAndCompute(image_bgr_prev, None)
    kp2, des2 = orb.detectAndCompute(image_bgr, None)
    BFMatcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

    if (des1 is None or des2 is None):
        # If detecting descriptors failed, return None as all parameters
        return None, None, None

    # Match descriptors
    try:
        matches = BFMatcher.knnMatch(des1, des2, k=2)
    except:
        matches = []

    # store all the good matches as per Lowe's ratio test.
    good = []
    try:
        for m, n in matches:
            if m.distance < 0.7 * n.distance:
                good.append(m)
    except:
        print("No matches found")

    p1 = np.float32([ kp1[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
    p2 = np.float32([ kp2[n.trainIdx].pt for n in good ]).reshape(-1,1,2)

    #E, mask = cv2.findEssentialMat(p1, p2, sbcf.K, cv2.RANSAC, 0.999, 1.0)
    F, mask = cv2.findFundamentalMat(p1,p2, method=cv2.FM_RANSAC, ransacReprojThreshold=0.1)

    # findFundamentalMat sometimes returns three possible fundamental matrices. If this is the case, just take the first one (silently)
    # See https://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html#findfundamentalmat
    if F is not None:
        F = F[0:3,0:3]

    mask = np.array(np.squeeze(mask),dtype=bool)
    p1 = np.squeeze(p1)
    p2 = np.squeeze(p2)

    masked_p1 = p1[mask,:]
    masked_p2 = p2[mask,:]

    changeInPxCoords = np.linalg.norm(masked_p2-masked_p1, axis=1)
    
    meanChangeInPxCoords = np.mean(changeInPxCoords)
    stdDevOfChangeInPxCoords = np.std(changeInPxCoords)

    return F, meanChangeInPxCoords, stdDevOfChangeInPxCoords

