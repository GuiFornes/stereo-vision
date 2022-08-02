import os
import numpy as np
import cv2

CHESSBOARD_SIZE = (6, 9)
SQUARE_SIZE = 0.725  # in cm
CHESSBOARD_OPTIONS = (cv2.CALIB_CB_ADAPTIVE_THRESH |
                      cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_FAST_CHECK)
DRAW_CHESSBOARD = 1
INTRINSEQUE_CALIBRATION_EVAL = 1
EXTRINSEQUE_CALIBRATION_EVAL = 1

WORKING_WIDTH = 640
WORKING_HEIGHT = 480
WORKING_SIZE = (WORKING_WIDTH, WORKING_HEIGHT)

SUBPIX_CRITERIA = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.1)

OBJECT_POINT_ZERO = np.zeros((CHESSBOARD_SIZE[0] * CHESSBOARD_SIZE[1], 3), np.float32)
OBJECT_POINT_ZERO[:, :2] = np.mgrid[0:CHESSBOARD_SIZE[0], 0:CHESSBOARD_SIZE[1]].T.reshape(-1, 2)

OPTIMIZE_ALPHA = 0.25

TERMINATION_CRITERIA = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

MAX_IMAGES = 90


def readImagesAndFindChessboards():
    """
    Reads images from the directory and finds chessboards in them.
    Returns a list of object points and a list of image points.
    """
    corner_coordinates = np.zeros((np.prod(CHESSBOARD_SIZE), 3), np.float32)
    corner_coordinates[:, :2] = np.indices(CHESSBOARD_SIZE).T.reshape(-1, 2)
    corner_coordinates *= SQUARE_SIZE
    objPoints = []
    imgPointsLeftPaired = []
    imgPointsRightPaired = []
    imgPointsLeftOnly = []
    imgPointsRightOnly = []
    objPointsRightOnly = []
    objPointsLeftOnly = []
    notfound = []

    photo_counter = 0
    width, height = None, None
    while photo_counter < MAX_IMAGES:
        photo_counter = photo_counter + 1
        leftName = './pairs/left_' + str(photo_counter).zfill(2) + '.png'
        rightName = './pairs/right_' + str(photo_counter).zfill(2) + '.png'
        if not os.path.exists(leftName) or not os.path.exists(rightName):
            print('Pair No ' + str(photo_counter) + ' invalid')
            photo_counter = photo_counter + 1
            continue
        # Opening the images
        imgL = cv2.imread(leftName)
        imgR = cv2.imread(rightName)
        print('Import pair No ' + str(photo_counter))


        grayL = cv2.cvtColor(imgL, cv2.COLOR_BGR2GRAY)
        grayR = cv2.cvtColor(imgR, cv2.COLOR_BGR2GRAY)
        assert grayL.shape == grayR.shape, "The images must have the same size."
        width, height = grayL.shape[:2]

        # Try to find the chessboard corners
        retL, cornersL = cv2.findChessboardCorners(grayL, CHESSBOARD_SIZE, CHESSBOARD_OPTIONS)
        retR, cornersR = cv2.findChessboardCorners(grayR, CHESSBOARD_SIZE, CHESSBOARD_OPTIONS)

        if retL:
            objPointsLeftOnly.append(corner_coordinates)
            cv2.cornerSubPix(grayL, cornersL, (3, 3), (-1, -1), SUBPIX_CRITERIA)
            imgPointsLeftOnly.append(cornersL)
        if retR:
            objPointsRightOnly.append(corner_coordinates)
            cv2.cornerSubPix(grayR, cornersR, (3, 3), (-1, -1), SUBPIX_CRITERIA)
            imgPointsRightOnly.append(cornersR)
        if not retL or not retR:
            print('Could not find chessboard in pair No ' + str(photo_counter) +
                  ': retL = ' + str(retL) + ', retR = ' + str(retR))
            notfound.append(photo_counter)
            continue

        '''
        # Here is our scaling trick! Higher res to better find chessboard, lower res for a speeder calibration!
        smallGrayL = cv2.resize(grayL, (WORKING_WIDTH, WORKING_HEIGHT))
        smallGrayR = cv2.resize(grayR, (WORKING_WIDTH, WORKING_HEIGHT))

        scale_ratio = height / WORKING_HEIGHT
        print("Scale ratio: ", scale_ratio)
        cornersL = cornersL * scale_ratio
        cornersR = cornersR * scale_ratio
        '''

        # Refine corners and add to array for processing
        objPoints.append(corner_coordinates)
        cv2.cornerSubPix(grayL, cornersL, (3, 3), (-1, -1), SUBPIX_CRITERIA)
        cv2.cornerSubPix(grayR, cornersR, (3, 3), (-1, -1), SUBPIX_CRITERIA)
        imgPointsLeftPaired.append(cornersL)
        imgPointsRightPaired.append(cornersR)

        if DRAW_CHESSBOARD:  # Draw and display the chessboard corners
            cv2.drawChessboardCorners(imgL, (6, 9), cornersL, retL)
            cv2.imshow('Corners LEFT', imgL)
            cv2.drawChessboardCorners(imgR, (6, 9), cornersR, retR)
            cv2.imshow('Corners RIGHT', imgR)
            key = cv2.waitKey(0)
            if key == ord("q"):
                exit(0)
            cv2.destroyAllWindows()

    print("Found corners in {0} out of {1} images"
          .format(len(imgPointsRightPaired), photo_counter - 1))
    print("Not found in ", notfound)

    np.savez_compressed('calib_data/found_corners_cache.npz', objPoints=objPoints,
                        imgPointsLeftPaired=imgPointsLeftPaired, imgPointsRightPaired=imgPointsRightPaired)
    return objPointsLeftOnly, imgPointsLeftOnly, objPointsRightOnly, imgPointsRightOnly, \
           objPoints, imgPointsLeftPaired, imgPointsRightPaired, (height, width)


def calibration_graphic_eval():
    # Load the saved data
    with np.load('calib_data/calibration.npz') as data:
        imageSize = data['imageSize']
        leftMapX = data['leftMapX']
        leftMapY = data['leftMapY']
        leftROI = data['leftROI']
        rightMapX = data['rightMapX']
        rightMapY = data['rightMapY']
        rightROI = data['rightROI']

    # Read the images
    stereoPair = cv2.imread('scenes/photo.png')
    # stereoPair = cv2.resize(stereoPair, imageSize)
    width = stereoPair.shape[1] // 2
    imgL = stereoPair[:, :width, :]
    imgR = stereoPair[:, width:, :]

    # Un-distort the images
    rectifiedL = cv2.remap(imgL, leftMapX, leftMapY, cv2.INTER_LINEAR)
    rectifiedR = cv2.remap(imgR, rightMapX, rightMapY, cv2.INTER_LINEAR)

    cv2.imwrite("rectified_left.png", rectifiedL)
    cv2.imwrite("rectified_right.png", rectifiedR)

    # Crop the images
    # imgL = imgL[leftROI[1]:leftROI[1] + leftROI[3], leftROI[0]:leftROI[0] + leftROI[2]]
    # imgR = imgR[rightROI[1]:rightROI[1] + rightROI[3], rightROI[0]:rightROI[0] + rightROI[2]]

    def draw_epipolar_lines(img, line, color=(0, 0, 255)):
        cv2.line(img, (0, line), (width, line), color, 1)
        return img

    for i in np.linspace(0, imgL.shape[0], 10):
        draw_epipolar_lines(rectifiedL, int(i))
        draw_epipolar_lines(rectifiedR, int(i))
        draw_epipolar_lines(imgL, int(i))
        draw_epipolar_lines(imgR, int(i))

    cv2.imshow('Left Before', imgL)
    cv2.imshow('Right Before', imgR)
    cv2.imshow('Left CALIBRATED', rectifiedL)
    cv2.imshow('Right CALIBRATED', rectifiedR)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


if __name__ == '__main__':
    (leftObjPointsOnly, leftImgPointsOnly, rightObjPointsOnly, rightImgPointsOnly,
     objectPoints, leftImagePoints, rightImagePoints, imageSize) = readImagesAndFindChessboards()

    print("Calibrating left camera...")
    _, leftCameraMatrix, leftDistortionCoefficients, rvecs, tvecs = cv2.calibrateCamera(
        leftObjPointsOnly, leftImgPointsOnly, imageSize, None, None)
    print("Calibrating right camera...")
    _, rightCameraMatrix, rightDistortionCoefficients, _, _ = cv2.calibrateCamera(
        rightObjPointsOnly, rightImgPointsOnly, imageSize, None, None)

    if INTRINSEQUE_CALIBRATION_EVAL:
        stereo = cv2.imread('scenes/scene_1920x720_1.png')
        imgL = stereo[:, :stereo.shape[1] // 2, :]
        imgR = stereo[:, stereo.shape[1] // 2:, :]
        undistortedL = cv2.undistort(imgL, leftCameraMatrix, leftDistortionCoefficients, None, None)
        undistortedR = cv2.undistort(imgR, rightCameraMatrix, rightDistortionCoefficients, None, None)
        cv2.imshow('Stereo Pair', stereo)
        cv2.imshow("undistortedL :", undistortedL)
        cv2.imshow("undistortedR :", undistortedR)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    print("Calibrating cameras together...")
    (_, _, _, _, _, rotationMatrix, translationVector, _, _) = cv2.stereoCalibrate(
        objectPoints, leftImagePoints, rightImagePoints,
        leftCameraMatrix, leftDistortionCoefficients,
        rightCameraMatrix, rightDistortionCoefficients,
        imageSize, None, None, None, None,
        cv2.CALIB_FIX_INTRINSIC, TERMINATION_CRITERIA)

    print("Rectifying cameras...")
    (leftRectification, rightRectification, leftProjection, rightProjection,
     disparityToDepthMap, leftROI, rightROI) = cv2.stereoRectify(
        leftCameraMatrix, leftDistortionCoefficients,
        rightCameraMatrix, rightDistortionCoefficients,
        imageSize, rotationMatrix, translationVector,
        None, None, None, None, None,
        cv2.CALIB_ZERO_DISPARITY, OPTIMIZE_ALPHA)

    print(disparityToDepthMap)
    print(translationVector)
    print(rotationMatrix)

    print("Saving calibration...")
    leftMapX, leftMapY = cv2.initUndistortRectifyMap(
        leftCameraMatrix, leftDistortionCoefficients, leftRectification,
        leftProjection, imageSize, cv2.CV_32FC1)
    rightMapX, rightMapY = cv2.initUndistortRectifyMap(
        rightCameraMatrix, rightDistortionCoefficients, rightRectification,
        rightProjection, imageSize, cv2.CV_32FC1)

    np.savez_compressed('calib_data/calibration.npz', imageSize=imageSize, disparityToDepthMap=disparityToDepthMap,
                        leftMapX=leftMapX, leftMapY=leftMapY, leftROI=leftROI,
                        rightMapX=rightMapX, rightMapY=rightMapY, rightROI=rightROI)

    if EXTRINSEQUE_CALIBRATION_EVAL:
        calibration_graphic_eval()

        # accuracy according to https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html
        mean_error = 0
        for i in range(len(objectPoints)):
            imgpoints2, _ = cv2.projectPoints(leftObjPointsOnly[i], rvecs[i], tvecs[i],
                                              leftCameraMatrix, leftDistortionCoefficients)
            error = cv2.norm(leftImgPointsOnly[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
            mean_error += error
        print("total error: {}".format(mean_error / len(objectPoints)))
