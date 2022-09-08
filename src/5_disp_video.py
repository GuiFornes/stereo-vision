import utils_arducam as arducam
import time
import cv2
import numpy as np
import utils_stereovision as stereovision
import matplotlib.pyplot as plt


if __name__ == "__main__":
    # Initialisation of the camera
    ret_val = arducam.init()
    if ret_val:
        exit()

    # Initialisation of the display windows
    cv2.namedWindow("left rectified")
    cv2.moveWindow("left rectified", 100, 600)
    cv2.namedWindow("Disparity Map")
    cv2.moveWindow("Disparity Map", 50, 20)

    # Variables declaration
    time0 = time.time()
    count = 0
    matchers = stereovision.Matchers(verbose=False)

    with np.load('calib_data/calibration.npz') as data:
        leftROI = data['leftROI']

    # Capturing loop
    while arducam.running:
        image = arducam.getFrame()

        # Compute fps
        time1 = time.time()
        if time1 - time0 >= 1:
            print("%s %d %s\n" % ("fps:", count, "/s"))
            count = 0
            time0 = time1
        count += 1

        # Compute disparity map
        rectifiedL, rectifiedR = stereovision.rectifyStereoImage(image)

        disparity_map = matchers.computeFilteredDisparityMap(rectifiedL, rectifiedR)
        # only visual factors operation
        disparity_map += 24
        disparity_map *= 2
        leftROI = (leftROI[0] * 1, leftROI[1] * 1, 960 - (960 - leftROI[2]) * 1, 720 - (720 - leftROI[3]) * 1)
        disparity_map = disparity_map[leftROI[1]:leftROI[1] + leftROI[3], leftROI[0]:leftROI[0] + leftROI[2]]
        rectifiedL = rectifiedL[leftROI[1]:leftROI[1] + leftROI[3], leftROI[0]:leftROI[0] + leftROI[2]]

        # Show image
        # cv2.imshow("Stereo", image)
        cv2.imshow("Disparity Map", cv2.applyColorMap(disparity_map, cv2.COLORMAP_JET))
        cv2.imshow("left rectified", rectifiedL)
        # cv2.imshow("right rectified", rectifiedR)
        cv2.waitKey(1)

    # Close the camera and release the resources
    arducam.close()
