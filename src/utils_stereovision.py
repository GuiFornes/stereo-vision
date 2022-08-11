import cv2
import numpy as np
import time

########################################
# Global variables preset
########################################
TOTAL_CALIB_PICS = 120  # nb of pictures used for calibration


def rectifyImages(imL, imR, calib_data='calib_data/calibration.npz'):
    """
    Rectify stereo pair
    :param imR: right original image
    :param imL: left original image
    :param calib_data: saved calibration data
    :return: rectified left and right images
    """
    # Load the saved data
    with np.load('calib_data/calibration.npz') as data:
        leftMapX = data['leftMapX']
        leftMapY = data['leftMapY']
        leftROI = data['leftROI']
        rightMapX = data['rightMapX']
        rightMapY = data['rightMapY']
        rightROI = data['rightROI']

    # Un-distort the images
    rectifiedL = cv2.remap(imL, leftMapX, leftMapY, cv2.INTER_LINEAR)
    rectifiedR = cv2.remap(imR, rightMapX, rightMapY, cv2.INTER_LINEAR)

    # Crop the images
    # imgL = imgL[leftROI[1]:leftROI[1] + leftROI[3], leftROI[0]:leftROI[0] + leftROI[2]]
    # imgR = imgR[rightROI[1]:rightROI[1] + rightROI[3], rightROI[0]:rightROI[0] + rightROI[2]]
    return rectifiedL, rectifiedR


def splitStereoImage(stereo_image):
    """
    Split a stereo pair
    :param stereo_image: a frame from utils_arducam.getFrame()
    :return: left and right original images
    """
    imgWidth = stereo_image.shape[1] // 2
    imgL = stereo_image[:, :imgWidth, :]
    imgR = stereo_image[:, imgWidth:, :]
    return imgL, imgR


def rectifyStereoImage(pair):
    """
    Split and rectify a stereo pair
    :param pair: a frame from utils_arducam.getFrame()
    :return: rectified left and right images
    """
    imgL, imgR = splitStereoImage(pair)
    rectifiedL, rectifiedR = rectifyImages(imgL, imgR)
    return rectifiedL, rectifiedR


class Matchers:
    w_size = 5
    max_disp = 5*16
    wls_lambda = 8000.0
    wls_sigma = 1.5

    def __init__(self, method='SGBM', filtered=True, verbose=True):
        self._method = method
        self._filtered = filtered
        self._verbose = verbose
        if self._verbose:
            print('[INFO] Instancing stereo-matcher, method used: ', self._method)
        if self._method == 'SGBM':
            self.left_matcher = cv2.StereoSGBM_create(minDisparity=0,
                                                      numDisparities=self.max_disp,
                                                      blockSize=self.w_size,
                                                      P1=24 * self.w_size * self.w_size,
                                                      P2=96 * self.w_size * self.w_size,
                                                      disp12MaxDiff=10,  # 12
                                                      preFilterCap=29,  # 32
                                                      uniquenessRatio=15,  # 10
                                                      speckleWindowSize=100,  # 50
                                                      speckleRange=2,  # 32
                                                      mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY)
        elif self._method == 'BM':
            self.w_size = 7
            self.left_matcher = cv2.StereoBM_create(numDisparities=self.max_disp,
                                                    blockSize=self.w_size)
            self.left_matcher.setMinDisparity(0)
            self.left_matcher.setPreFilterCap(29)
            self.left_matcher.setUniquenessRatio(15)
            self.left_matcher.setSpeckleWindowSize(100)
            self.left_matcher.setSpeckleRange(2)
            self.left_matcher.setTextureThreshold(13)
        else:
            print('[ERROR] Invalid matching method')
            exit(0)
        self.right_matcher = cv2.ximgproc.createRightMatcher(self.left_matcher)
        if self._filtered:
            if self._verbose:
                print('[INFO] Instancing WLS filter')
            self.wls_filter = cv2.ximgproc.createDisparityWLSFilter(matcher_left=self.left_matcher)
            self.wls_filter.setLambda(self.wls_lambda)
            self.wls_filter.setSigmaColor(self.wls_sigma)

    def __computeFeaturesMatching(self, imL, imR):
        """ Compute disparity map """
        matching_time = time.time()
        left_disp = self.left_matcher.compute(imL, imR)
        right_disp = self.right_matcher.compute(imR, imL)
        matching_time = time.time() - matching_time
        if self._verbose:
            print("[INFO] Matching time: ", matching_time)
        return left_disp, right_disp

    def __filterDisparity(self, imL, imR, left_disp, right_disp):
        """ Filter disparity map """
        if not self._filtered:
            return left_disp
        filtering_time = time.time()
        filtered_disp = self.wls_filter.filter(left_disp, imL, disparity_map_right=right_disp, right_view=imR)
        filtering_time = time.time() - filtering_time
        if self._verbose:
            print("[INFO] Filtering time: ", filtering_time)
        return filtered_disp

    ########################################################
    # Public methods
    ########################################################
    def computeRawDisparityMap(self, imL, imR):
        """
        Compute disparity map
        :param imL: left rectified image
        :param imR: right rectified image
        :return: raw disparity map (without filtering)
        """
        if self._verbose:
            print("[INFO] Computing raw disparity map...")
        left_disp, _ = self.__computeFeaturesMatching(imL, imR)
        left_disp_vis = cv2.ximgproc.getDisparityVis(left_disp)
        return left_disp_vis

    def computeFilteredDisparityMap(self, imL, imR):
        """ Compute disparity map """
        if self._verbose:
            print("[INFO] Computing filtered disparity map...")
        left_disp, right_disp = self.__computeFeaturesMatching(imL, imR)
        filtered_disp = self.__filterDisparity(imL, imR, left_disp, right_disp)
        filtered_disp_vis = cv2.ximgproc.getDisparityVis(filtered_disp)
        return filtered_disp_vis





