import utils_arducam as arducam
import time
import cv2
import utils_stereovision as stereovision


if __name__ == "__main__":
    # Initialisation of the camera
    ret_val = arducam.init()
    if ret_val:
        exit()

    # Initialisation of the display windows
    cv2.namedWindow("Stereo")
    cv2.moveWindow("Stereo", 100, 600)
    cv2.namedWindow("Disparity Map")
    cv2.moveWindow("Disparity Map", 50, 20)

    # Variables declaration
    time0 = time.time()
    count = 0
    matchers = stereovision.Matchers(verbose=False)

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
        cv2.imshow("left rectified", rectifiedL)
        cv2.imshow("right rectified", rectifiedR)

        disparity_map = matchers.computeFilteredDisparityMap(rectifiedL, rectifiedR)

        # Show image
        cv2.imshow("Stereo", image)
        cv2.imshow("Disparity Map", disparity_map)
        cv2.waitKey(1)

    # Close the camera and release the resources
    arducam.close()
