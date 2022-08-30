import os
import numpy as np
import cv2
import utils_stereovision as stereovision

###################################################
# Calibration and computation flags and parameters
###################################################
CHESSBOARD_SIZE = stereovision.CHESSBOARD_SIZE
SQUARE_SIZE = stereovision.SQUARE_SIZE
CHESSBOARD_OPTIONS = (cv2.CALIB_CB_ADAPTIVE_THRESH |
                      cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_FAST_CHECK)
OBJECT_POINT_ZERO = np.zeros((CHESSBOARD_SIZE[0] * CHESSBOARD_SIZE[1], 3), np.float32)
OBJECT_POINT_ZERO[:, :2] = np.mgrid[0:CHESSBOARD_SIZE[0], 0:CHESSBOARD_SIZE[1]].T.reshape(-1, 2)

SUBPIX_CRITERIA = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.1)
OPTIMIZE_ALPHA = 0.9
TERMINATION_CRITERIA = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

###################################################
# Execution parameters
###################################################
VERBOSE = True
DRAW_CHESSBOARD = False  # Show (or not) the result of chessboard detection
INTRINSIC_CALIBRATION_EVAL = True  # Display th result of intrinsic calibration
EXTRINSIC_CALIBRATION_EVAL = True  # Display the result of extrinsic calibration


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
    while photo_counter < stereovision.TOTAL_CALIB_PICS:
        photo_counter = photo_counter + 1
        leftName = './pairs/left_' + str(photo_counter).zfill(2) + '.png'
        rightName = './pairs/right_' + str(photo_counter).zfill(2) + '.png'
        if not os.path.exists(leftName) or not os.path.exists(rightName):
            print('[WARN] Pair No ' + str(photo_counter) + ' invalid')
            photo_counter = photo_counter + 1
            continue
        # Opening the images
        imgL = cv2.imread(leftName)
        imgR = cv2.imread(rightName)
        if VERBOSE:
            print('[INFO] Import pair No ' + str(photo_counter))

        grayL = cv2.cvtColor(imgL, cv2.COLOR_BGR2GRAY)
        grayR = cv2.cvtColor(imgR, cv2.COLOR_BGR2GRAY)
        assert grayL.shape == grayR.shape, "The images must have the same size."
        width, height = grayL.shape[:2]

        # Try to find the chessboard corners
        retL, cornersL = cv2.findChessboardCorners(grayL, CHESSBOARD_SIZE, CHESSBOARD_OPTIONS)
        retR, cornersR = cv2.findChessboardCorners(grayR, CHESSBOARD_SIZE, CHESSBOARD_OPTIONS)

        if retL:
            objPointsLeftOnly.append(corner_coordinates)
            cornersL = cv2.cornerSubPix(grayL, cornersL, (3, 3), (-1, -1), SUBPIX_CRITERIA)
            imgPointsLeftOnly.append(cornersL)
        if retR:
            objPointsRightOnly.append(corner_coordinates)
            cornersR = cv2.cornerSubPix(grayR, cornersR, (3, 3), (-1, -1), SUBPIX_CRITERIA)
            imgPointsRightOnly.append(cornersR)

        if DRAW_CHESSBOARD:  # Draw and display the chessboard corners
            cv2.namedWindow("Corners LEFT")
            cv2.moveWindow("Corners LEFT", 0, 20)
            cv2.namedWindow("Corners RIGHT")
            cv2.moveWindow("Corners RIGHT", 1100, 20)
            cv2.drawChessboardCorners(imgL, (6, 9), cornersL, retL)
            cv2.imshow('Corners LEFT', imgL)
            cv2.drawChessboardCorners(imgR, (6, 9), cornersR, retR)
            cv2.imshow('Corners RIGHT', imgR)
            key = cv2.waitKey(0)
            if key == ord("q"):
                exit(0)

        if not retL or not retR:
            if not retL and not retR:
                notfound.append(photo_counter)
            print('[WARN] Could not find chessboard in pair No ' + str(photo_counter) +
                  ': retL = ' + str(retL) + ', retR = ' + str(retR))
            continue

        # Reaching this part means that we found the chessboard in both images
        objPoints.append(corner_coordinates)
        imgPointsLeftPaired.append(cornersL)
        imgPointsRightPaired.append(cornersR)
        continue
    cv2.destroyAllWindows()
    if VERBOSE:
        print("[INFO] Found corners in both (stereo pair) for {0} out of {1} images"
              .format(len(imgPointsRightPaired), photo_counter))
        if len(notfound) > 0:
            print("[INFO] Not found in ", notfound)

    # Could be upgrade to not re-search corners each time if pictures as not changed
    np.savez_compressed('calib_data/found_corners_cache.npz', objPoints=objPoints,
                        imgPointsLeftPaired=imgPointsLeftPaired, imgPointsRightPaired=imgPointsRightPaired)
    return objPointsLeftOnly, imgPointsLeftOnly, objPointsRightOnly, imgPointsRightOnly, \
           objPoints, imgPointsLeftPaired, imgPointsRightPaired, (height, width)


def draw_epipolar_line(img, line, color=(0, 0, 255)):
    cv2.line(img, (0, line), (img.shape[1], line), color, 1)
    return img


def calibration_graphic_eval():
    # Read the images
    if not os.path.exists('scenes/photo.png'):
        print("[ERROR] 'scenes/photo.png' not found, launch 0_test.py and press 's' first")
        exit(0)
    stereo_pair = cv2.imread('scenes/photo.png')
    imageL, imageR = stereovision.splitStereoImage(stereo_pair)
    rectifiedL, rectifiedR = stereovision.rectifyImages(imageL, imageR)

    cv2.imwrite("rectified_left.png", rectifiedL)
    cv2.imwrite("rectified_right.png", rectifiedR)

    for row in np.linspace(0, imageL.shape[0], 10):
        draw_epipolar_line(rectifiedL, int(row))
        draw_epipolar_line(rectifiedR, int(row))
        draw_epipolar_line(imageL, int(row))
        draw_epipolar_line(imageR, int(row))

    cv2.imshow('Left Before', imageL)
    cv2.imshow('Right Before', imageR)
    cv2.imshow('Left CALIBRATED', rectifiedL)
    cv2.imshow('Right CALIBRATED', rectifiedR)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


if __name__ == '__main__':
    (leftObjPointsOnly, leftImgPointsOnly, rightObjPointsOnly, rightImgPointsOnly,
     objectPoints, leftImagePoints, rightImagePoints, imageSize) = readImagesAndFindChessboards()

    if VERBOSE:
        print("[INFO] Calibrating left camera...")
    _, leftCameraMatrix, leftDistortionCoefficients, rvecs, tvecs = cv2.calibrateCamera(
        leftObjPointsOnly, leftImgPointsOnly, imageSize, None, None)
    # leftCameraMatrix, roi_L = cv2.getOptimalNewCameraMatrix(  # Adjust the output image size with pixel lost or zoom
    #     leftCameraMatrix, leftDistortionCoefficients,
    #     imageSize=imageSize, alpha=1)  # alpha = 0 means no dead pixels, alpha = 1 means no lost pixels
    if VERBOSE:
        print("leftCameraMatrix: ", leftCameraMatrix)

    if VERBOSE:
        print("[INFO] Calibrating right camera...")
    _, rightCameraMatrix, rightDistortionCoefficients, _, _ = cv2.calibrateCamera(
        rightObjPointsOnly, rightImgPointsOnly, imageSize, None, None)
    # rightCameraMatrix, roi_R = cv2.getOptimalNewCameraMatrix(  # Same as above
    #     rightCameraMatrix, rightDistortionCoefficients,
    #     imageSize=imageSize, alpha=1)
    if VERBOSE:
        print("rightCameraMatrix: ", rightCameraMatrix)

    if INTRINSIC_CALIBRATION_EVAL:  # Visual evaluation of intrinsic calibration
        stereo_chess = cv2.imread('scenes/scene_1920x720_1.png')
        imL, imR = stereovision.splitStereoImage(stereo_chess)
        imL = cv2.rotate(imL, cv2.ROTATE_90_CLOCKWISE)
        imR = cv2.rotate(imR, cv2.ROTATE_90_CLOCKWISE)
        for r in np.linspace(0, imL.shape[0], 20):
            draw_epipolar_line(imL, int(r))
            draw_epipolar_line(imR, int(r))
        imL = cv2.rotate(imL, cv2.ROTATE_90_COUNTERCLOCKWISE)
        imR = cv2.rotate(imR, cv2.ROTATE_90_COUNTERCLOCKWISE)
        for r in np.linspace(0, imL.shape[0], 20):
            draw_epipolar_line(imL, int(r))
            draw_epipolar_line(imR, int(r))
        undistortedL = cv2.undistort(imL, leftCameraMatrix, leftDistortionCoefficients, None, None)
        undistortedR = cv2.undistort(imR, rightCameraMatrix, rightDistortionCoefficients, None, None)
        cv2.imshow('Stereo Pair', stereo_chess)
        cv2.imshow("undistortedL :", undistortedL)
        cv2.imshow("undistortedR :", undistortedR)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    if VERBOSE:
        print("[INFO] Calibrating cameras together...")
    (_, _, _, _, _, rotationMatrix, translationVector, essentialMatrix, fundamentalMatrix) = cv2.stereoCalibrate(
        objectPoints, leftImagePoints, rightImagePoints,
        leftCameraMatrix, leftDistortionCoefficients,
        rightCameraMatrix, rightDistortionCoefficients,
        imageSize, None, None, None, None,
        cv2.CALIB_FIX_INTRINSIC,  # this flag fixes the previously computed intrinsic parameters.
        TERMINATION_CRITERIA)

    if VERBOSE:
        print("[INFO] Rectifying cameras...")
    (leftRectification, rightRectification, leftProjection, rightProjection,
     disparityToDepthMap, leftROI, rightROI) = cv2.stereoRectify(
        leftCameraMatrix, leftDistortionCoefficients,
        rightCameraMatrix, rightDistortionCoefficients,
        imageSize, rotationMatrix, translationVector,
        flags=0, alpha=OPTIMIZE_ALPHA)

    print("disparityToDepthMap :", disparityToDepthMap)
    print("translationVector :", translationVector)
    print("rotationMatrix :", rotationMatrix)

    if VERBOSE:
        print("[INFO] Saving calibration...")
    leftMapX, leftMapY = cv2.initUndistortRectifyMap(
        leftCameraMatrix, leftDistortionCoefficients, leftRectification,
        leftProjection, imageSize, cv2.CV_32FC1)
    rightMapX, rightMapY = cv2.initUndistortRectifyMap(
        rightCameraMatrix, rightDistortionCoefficients, rightRectification,
        rightProjection, imageSize, cv2.CV_32FC1)

    np.savez_compressed('calib_data/calibration.npz', imageSize=imageSize, disparityToDepthMap=disparityToDepthMap,
                        leftMapX=leftMapX, leftMapY=leftMapY, leftROI=leftROI,
                        rightMapX=rightMapX, rightMapY=rightMapY, rightROI=rightROI)

    if EXTRINSIC_CALIBRATION_EVAL:
        calibration_graphic_eval()  # Visual evaluation of stereo/extrinsic calibration

        # Accuracy according to https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html
        mean_error = 0
        for i in range(len(objectPoints)):
            imgpoints2, _ = cv2.projectPoints(leftObjPointsOnly[i], rvecs[i], tvecs[i],
                                              leftCameraMatrix, leftDistortionCoefficients)
            error = cv2.norm(leftImgPointsOnly[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
            mean_error += error
        print("total error: {}".format(mean_error / len(objectPoints)))
