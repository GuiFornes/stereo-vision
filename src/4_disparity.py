import math
import cv2
import time
import numpy as np

wsize = 3
max_disp = 160
wls_lambda = 8000.0
wls_sigma = 1.5
fbs_spatial = 16.0
fbs_luma = 8.0
fbs_chroma = 8.0
fbs_lambda = 128.0

# Import the stereo camera calibration data
with np.load('calib_data/calibration.npz') as data:
    imageSize = data['imageSize']
    leftMapX = data['leftMapX']
    leftMapY = data['leftMapY']
    leftROI = data['leftROI']
    rightMapX = data['rightMapX']
    rightMapY = data['rightMapY']
    rightROI = data['rightROI']

# Preparing the images
stereoPair = cv2.imread('./scenes/photo.png')
imgWidth = stereoPair.shape[1] // 2
imgHeight = stereoPair.shape[0]
imgSize = (imgWidth, imgHeight)
imgL = stereoPair[:, :imgWidth, :]
imgR = stereoPair[:, imgWidth:, :]

rectifiedL = cv2.cvtColor(cv2.remap(imgL, leftMapX, leftMapY, cv2.INTER_LINEAR), cv2.COLOR_BGR2GRAY)
rectifiedR = cv2.cvtColor(cv2.remap(imgR, rightMapX, rightMapY, cv2.INTER_LINEAR), cv2.COLOR_BGR2GRAY)

"""
imgL = cv2.imread('left_test.jpg')
imgR = cv2.imread('right_test.jpg')
rectifiedL = cv2.cvtColor(imgL, cv2.COLOR_BGR2GRAY)
rectifiedR = cv2.cvtColor(imgR, cv2.COLOR_BGR2GRAY)
imgSize = (imgL.shape[1], imgL.shape[0])
"""
workingSize = imgSize  # (imgL.shape[1]//2, imgL.shape[0]//2)

# Downscale the images
left_for_matcher = cv2.resize(rectifiedL, workingSize, fx=0.5, fy=0.5)
right_for_matcher = cv2.resize(rectifiedR, workingSize, fx=0.5, fy=0.5)

# Create the SGBM
left_matcher = cv2.StereoSGBM_create(0, max_disp, wsize)
left_matcher.setP1(24 * wsize * wsize)
left_matcher.setP2(96 * wsize * wsize)
left_matcher.setPreFilterCap(63)
left_matcher.setMode(cv2.STEREO_SGBM_MODE_SGBM_3WAY)

left_matcher.setUniquenessRatio(-100)
left_matcher.setSpeckleWindowSize(100)
left_matcher.setSpeckleRange(8)
left_matcher.setDisp12MaxDiff(10)

right_matcher = cv2.ximgproc.createRightMatcher(left_matcher)

# Compute the disparity map
matching_time = time.time()
left_disp = left_matcher.compute(left_for_matcher, right_for_matcher)
right_disp = right_matcher.compute(right_for_matcher, left_for_matcher)
matching_time = time.time() - matching_time
print("Matching time: ", matching_time)

# Create the wls post-matching filter
wls_filter = cv2.ximgproc.createDisparityWLSFilter(left_matcher)
wls_filter.setLambda(wls_lambda)
wls_filter.setSigmaColor(wls_sigma)

# Filter the disparity map
filtering_time = time.time()
filtered_disp = wls_filter.filter(left_disp, rectifiedL, disparity_map_right=right_disp, right_view=rectifiedR)
filtering_time = time.time() - filtering_time
print("Filtering time: ", filtering_time)

conf_map = wls_filter.getConfidenceMap()
roi = wls_filter.getROI()

# Upscale back to the original size
left_disp_resized = 2 * cv2.resize(left_disp, imgSize, 2.0, 2.0, cv2.INTER_LINEAR_EXACT)
roi *= 2

"""
solving_time = time.time()
solved_disp = cv2.ximgproc.fastBilateralSolverFilter(rectifiedL,
                                                     left_disp_resized,
                                                     conf_map,
                                                     sigma_spatial=fbs_spatial,
                                                     sigma_luma=fbs_luma,
                                                     sigma_chroma=fbs_chroma,
                                                     lambda_=fbs_lambda)
solving_time = time.time() - solving_time
solved_filtered_disp = cv2.ximgproc.fastBilateralSolverFilter(rectifiedL,
                                                              filtered_disp,
                                                              conf_map,
                                                              sigma_spatial=fbs_spatial,
                                                              sigma_luma=fbs_luma,
                                                              sigma_chroma=fbs_chroma,
                                                              lambda_=fbs_lambda)
"""

raw_disp_vis = cv2.ximgproc.getDisparityVis(left_disp)
filtered_disp_vis = cv2.ximgproc.getDisparityVis(filtered_disp)
# solved_disp_vis = cv2.ximgproc.getDisparityVis(solved_disp)
# solved_filtered_disp_vis = cv2.ximgproc.getDisparityVis(solved_filtered_disp)
left_disp_resized_vis = cv2.ximgproc.getDisparityVis(left_disp_resized)
cv2.imshow('raw Disparity Map', raw_disp_vis)
cv2.imshow('Filtered Disparity Map', filtered_disp_vis)
# cv2.imshow('Solved Disparity Map', solved_disp_vis)
# cv2.imshow('Solved Filtered Disparity Map', solved_filtered_disp_vis)
cv2.imshow('left_disp', left_disp_resized_vis)
cv2.imshow('left view', imgL)
cv2.waitKey(0)
cv2.destroyAllWindows()
cv2.imwrite("disparity_filtered.png", filtered_disp_vis)
