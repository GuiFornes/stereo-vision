import utils_arducam as arducam
from datetime import datetime
import time
import os
import cv2
from utils_stereovision import TOTAL_CALIB_PICS


if __name__ == "__main__":
    width = arducam.width
    height = arducam.height

    if not os.path.exists("scenes/"):
        os.makedirs("scenes/")
    counter = 0  # Counter for the number of photos taken
    countdown = 4  # countdown between each shoot

    pic_name = "./scenes/scene_" + str(width) + 'x' + str(height) + '_' + str(counter+1) + '.png'
    while os.path.exists(pic_name):
        counter += 1
        pic_name = "./scenes/scene_" + str(width) + 'x' + str(height) + '_' + str(counter+1) + '.png'

    if counter >= TOTAL_CALIB_PICS:
        print("All photos already taken, delete scenes folder to start again.")
        exit()
    print("at least ", counter, " photos already taken. At most ", TOTAL_CALIB_PICS - counter, " photos to take.")

    # Initialisation of the camera
    ret_val = arducam.init()
    if ret_val:
        exit()

    t2 = datetime.now()
    while arducam.running:
        frame = arducam.getFrame()
        t1 = datetime.now()
        cntdwn_timer = countdown - int((t1 - t2).total_seconds())

        # If countdown is zero - let's record next image
        if cntdwn_timer == -1:
            counter += 1
            filename = './scenes/scene_' + str(width) + 'x' + str(height) + '_' + \
                       str(counter) + '.png'
            while os.path.exists(filename) and counter < TOTAL_CALIB_PICS:
                counter += 1
                filename = './scenes/scene_' + str(width) + 'x' + str(height) + '_' + \
                           str(counter) + '.png'
            cv2.imwrite(filename, frame)
            print(' [' + str(counter) + ' of ' + str(TOTAL_CALIB_PICS) + '] ' + filename)
            t2 = datetime.now()
            time.sleep(1)
            cntdwn_timer = 0  # To avoid "-1" timer display

        # Draw cowntdown counter, seconds
        cv2.putText(frame, str(cntdwn_timer), (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 2.0, (0, 0, 255), 4, cv2.LINE_AA)
        cv2.imshow("pair", frame)
        key = cv2.waitKey(1) & 0xFF

        # Press 'Q' key to quit, or wait till all photos are taken
        if (key == ord("q")) | (counter >= TOTAL_CALIB_PICS):
            arducam.running = False
            print("End cycle, press 'q' + 'enter' to quit.")
            cv2.destroyAllWindows()
            # Close the camera and release the resources
            arducam.close()
            exit(0)
