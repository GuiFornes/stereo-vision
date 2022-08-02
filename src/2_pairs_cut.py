import cv2
import os
import json

try:
    camera_params = json.load(open("camera_params.txt", "r"))
except Exception as e:
    print(e)
    print("Please fill camera_params.txt first.")
    exit(-1)

# Global variables preset
TOTAL_PHOTOS = 90
photo_width = camera_params['width']
photo_height = camera_params['height']
img_height = photo_height
img_width = int(photo_width / 2)


def stereo_pair_cutter():
    photo_counter = 0
    if not os.path.isdir("./pairs"):
        os.makedirs("./pairs")
    while photo_counter != TOTAL_PHOTOS:
        photo_counter += 1
        filename = './scenes/scene_' + str(photo_width) + 'x' + str(photo_height) + \
                   '_' + str(photo_counter) + '.png'
        if not os.path.isfile(filename):
            print("No file named " + filename)
            continue
        pair_img = cv2.imread(filename, -1)

        cv2.imshow("ImagePair", pair_img)
        cv2.waitKey(0)
        imgLeft = pair_img[0:img_height, 0:img_width]
        imgRight = pair_img[0:img_height, img_width:photo_width]
        leftName = './pairs/left_' + str(photo_counter).zfill(2) + '.png'
        rightName = './pairs/right_' + str(photo_counter).zfill(2) + '.png'
        cv2.imwrite(leftName, imgLeft)
        cv2.imwrite(rightName, imgRight)
        print('Pair No ' + str(photo_counter) + ' saved.')

    print('End cycle')


if __name__ == '__main__':
    stereo_pair_cutter()
