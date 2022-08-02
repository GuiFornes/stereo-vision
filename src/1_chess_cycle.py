import utils
from datetime import datetime
import time
import os
import cv2
import json

import ArducamSDK

TOTAL_PHOTOS = 90  # total number of photos to take

if __name__ == "__main__":
    global running

    try:
        camera_params = json.load(open("camera_params.txt", "r"))
    except Exception as e:
        print(e)
        print("Please fill camera_params.txt first.")
        exit(-1)
    width = camera_params['width']
    height = camera_params['height']

    if not os.path.exists("scenes/"):
        os.makedirs("scenes/")
    counter = 0  # Counter for the number of photos taken
    countdown = 2  # countdown between each shoot

    pic_name = "./scenes/scene_" + str(width) + 'x' + str(height) + '_' + str(counter+1) + '.png'
    while os.path.exists(pic_name):
        counter += 1
        pic_name = "./scenes/scene_" + str(width) + 'x' + str(height) + '_' + str(counter+1) + '.png'

    if counter >= TOTAL_PHOTOS:
        print("All photos already taken, delete scenes folder to start again.")
        exit()
    print(counter, " photos already taken. ", TOTAL_PHOTOS - counter, " photos to take.")

    # Initialisation of the camera
    ret_val = utils.init()
    if ret_val:
        exit()

    t2 = datetime.now()
    while utils.running:
        frame = utils.get_frame()
        t1 = datetime.now()
        cntdwn_timer = countdown - int((t1 - t2).total_seconds())

        # If countdown is zero - let's record next image
        if cntdwn_timer == -1:
            counter += 1
            filename = './scenes/scene_' + str(width) + 'x' + str(height) + '_' + \
                       str(counter) + '.png'
            while os.path.exists(filename) and counter < TOTAL_PHOTOS:
                counter += 1
                filename = './scenes/scene_' + str(width) + 'x' + str(height) + '_' + \
                           str(counter) + '.png'
            cv2.imwrite(filename, frame)
            print(' [' + str(counter) + ' of ' + str(TOTAL_PHOTOS) + '] ' + filename)
            t2 = datetime.now()
            time.sleep(1)
            cntdwn_timer = 0  # To avoid "-1" timer display

        # Draw cowntdown counter, seconds
        cv2.putText(frame, str(cntdwn_timer), (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 2.0, (0, 0, 255), 4, cv2.LINE_AA)
        cv2.imshow("pair", frame)
        key = cv2.waitKey(1) & 0xFF

        ArducamSDK.Py_ArduCam_del(utils.handle)

        # Press 'Q' key to quit, or wait till all photos are taken
        if (key == ord("q")) | (counter >= TOTAL_PHOTOS):
            utils.running = False
            print("End cycle, press 'q' + 'enter' to quit.")
            cv2.destroyAllWindows()
            # Close the camera and release the resources
            utils.close()
            exit(0)

