import utils
import time
import cv2
import numpy as np

import ArducamSDK

# Variable parameters
wsize = 3
max_disp = 160
wls_lambda = 8000.0
wls_sigma = 1.5
fbs_spatial = 16.0
fbs_luma = 8.0
fbs_chroma = 8.0
fbs_lambda = 128.0

# Import calibration dataset
with np.load('calib_data/calibration.npz') as data:
    imageSize = data['imageSize']
    leftMapX = data['leftMapX']
    leftMapY = data['leftMapY']
    leftROI = data['leftROI']
    rightMapX = data['rightMapX']
    rightMapY = data['rightMapY']
    rightROI = data['rightROI']

# Create stereo matchers and filter
left_matcher = cv2.StereoSGBM_create(0, max_disp, wsize)
left_matcher.setP1(24 * wsize * wsize)
left_matcher.setP2(96 * wsize * wsize)
left_matcher.setPreFilterCap(63)
left_matcher.setMode(cv2.STEREO_SGBM_MODE_SGBM_3WAY)
left_matcher.setUniquenessRatio(0)
left_matcher.setSpeckleWindowSize(0)
left_matcher.setSpeckleRange(8)
left_matcher.setDisp12MaxDiff(10)
right_matcher = cv2.ximgproc.createRightMatcher(left_matcher)
wls_filter = cv2.ximgproc.createDisparityWLSFilter(left_matcher)
wls_filter.setLambda(wls_lambda)
wls_filter.setSigmaColor(wls_sigma)


def rectify_pair(pair):
    imgWidth = pair.shape[1] // 2
    imgL = pair[:, :imgWidth, :]
    imgR = pair[:, imgWidth:, :]
    rectifiedL = cv2.cvtColor(cv2.remap(imgL, leftMapX, leftMapY, cv2.INTER_LINEAR), cv2.COLOR_BGR2GRAY)
    rectifiedR = cv2.cvtColor(cv2.remap(imgR, rightMapX, rightMapY, cv2.INTER_LINEAR), cv2.COLOR_BGR2GRAY)
    return rectifiedL, rectifiedR


def compute_disparity_map(rectified_pair):
    left = rectified_pair[0]
    right = rectified_pair[1]
    left_disp = left_matcher.compute(left, right)
    right_disp = right_matcher.compute(right, left)
    filtered_disp = wls_filter.filter(left_disp, left, disparity_map_right=right_disp, right_view=right)
    return filtered_disp


if __name__ == "__main__":
    # Initialisation of the camera
    ret_val = utils.init()
    if ret_val:
        exit()

    # Initialisation of the displays
    cv2.namedWindow("Stereo")
    cv2.moveWindow("Stereo", 100, 600)
    cv2.namedWindow("Disparity Map")
    cv2.moveWindow("Disparity Map", 50, 20)

    # Variables declaration
    time0 = time.time()
    count = 0

    # Capturing loop
    while utils.running:
        image = utils.get_frame()

        # Compute fps
        time1 = time.time()
        if time1 - time0 >= 1:
            print("%s %d %s\n" % ("fps:", count, "/s"))
            count = 0
            time0 = time1
        count += 1

        # Compute disparity map
        rectified_pair = rectify_pair(image)
        disparity_map = compute_disparity_map(rectified_pair)

        # Show image
        cv2.imshow("Stereo", image)
        cv2.imshow("Disparity Map", disparity_map)
        cv2.waitKey(1)

    # Close the camera and release the resources
    utils.close()
