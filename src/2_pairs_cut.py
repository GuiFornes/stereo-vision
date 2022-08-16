import cv2
import os
from utils_stereovision import TOTAL_CALIB_PICS, splitStereoImage
from utils_arducam import width, height

# Global variables preset
img_height = width
img_width = int(height / 2)
VERBOSE = True


def stereo_pair_cutter():
    photo_counter = 0
    if not os.path.isdir("./pairs"):
        os.makedirs("./pairs")
    while photo_counter != TOTAL_CALIB_PICS:
        photo_counter += 1
        filename = './scenes/scene_' + str(width) + 'x' + str(height) + \
                   '_' + str(photo_counter) + '.png'
        if not os.path.isfile(filename):
            print("[WARN] No file named " + filename)
            continue
        pair_img = cv2.imread(filename, -1)
        cv2.imshow("ImagePair", pair_img)
        cv2.waitKey(0)
        imgLeft, imgRight = splitStereoImage(pair_img)
        if imgRight.shape != imgLeft.shape:
            print("[WARN] Split images shape are not equal")
        leftName = './pairs/left_' + str(photo_counter).zfill(2) + '.png'
        rightName = './pairs/right_' + str(photo_counter).zfill(2) + '.png'
        cv2.imwrite(leftName, imgLeft)
        cv2.imwrite(rightName, imgRight)
        if VERBOSE:
            print('[INFO] Pair No ' + str(photo_counter) + ' saved.')
    cv2.destroyAllWindows()
    if VERBOSE:
        print('[INFO] End cycle')


if __name__ == '__main__':
    stereo_pair_cutter()
