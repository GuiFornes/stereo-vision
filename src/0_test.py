import utils_arducam as arducam
import time
import cv2

import ArducamSDK

if __name__ == "__main__":
    # Initialisation of the camera
    ret_val = arducam.init()
    if ret_val:
        exit()

    # Variables declaration
    time0 = time.time()
    count = 0

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
        # Save image
        if arducam.save_flag:
            cv2.imwrite("scenes/photo.png", image)
            arducam.save_flag = False
            print("Image saved to scenes/photo.png")

        # Show image
        cv2.imshow("ArduCam Demo", image)
        cv2.waitKey(1)

    # Close the camera and release the resources
    arducam.close()
