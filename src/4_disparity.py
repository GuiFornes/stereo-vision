import cv2
import utils_stereovision as stereovision

if __name__ == '__main__':
    # Preparing the images
    stereo_pair = cv2.imread('./scenes/photo.png')
    imgL, imgR = stereovision.splitStereoImage(stereo_pair)
    rectifiedL, rectifiedR = stereovision.rectifyImages(imgL, imgR)

    """
    # Using demonstration pictures instead of the project ones
    imgL = cv2.imread('left_test.jpg')
    imgR = cv2.imread('right_test.jpg')
    rectifiedL = cv2.cvtColor(imgL, cv2.COLOR_BGR2GRAY)
    rectifiedR = cv2.cvtColor(imgR, cv2.COLOR_BGR2GRAY)
    imgSize = (imgL.shape[1], imgL.shape[0])
    """

    # Compute disparity map
    matchers = stereovision.Matchers()
    raw_disp_vis = matchers.computeRawDisparityMap(rectifiedL, rectifiedR)
    filtered_disp_vis = matchers.computeFilteredDisparityMap(rectifiedL, rectifiedR)

    # Display results
    confidence = matchers.getConfidenceMap()
    cv2.imshow('Confidence', confidence)
    cv2.imshow('raw Disparity Map', raw_disp_vis)
    cv2.imshow('Filtered Disparity Map Visual', filtered_disp_vis)
    cv2.imshow('left view', imgL)
    cv2.imshow('rectified left', rectifiedL)
    cv2.imshow('rectified right', rectifiedR)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    cv2.imwrite("disparity_filtered.png", filtered_disp_vis)
    print("[INFO] disparity_filtered.png saved")

"""
# draft for applying fast bilateral filter (seems not to work)
fbs_spatial = 16.0
fbs_luma = 8.0
fbs_chroma = 8.0
fbs_lambda = 128.0
conf_map = wls_filter.getConfidenceMap()
roi = wls_filter.getROI()

solving_time = time.time()
solved_disp = cv2.ximgproc.fastBilateralSolverFilter(rectifiedL,
                                                     left_disp_resized,
                                                     conf_map,
                                                     sigma_spatial=fbs_spatial,
                                                     sigma_luma=fbs_luma,
                                                     sigma_chroma=fbs_chroma,
                                                     lambda_=fbs_lambda)
solving_time = time.time() - solving_time
solved_filtered_disp = cv2.ximgproc.fastBilateralSolverFilter(rectifiedL,
                                                              filtered_disp,
                                                              conf_map,
                                                              sigma_spatial=fbs_spatial,
                                                              sigma_luma=fbs_luma,
                                                              sigma_chroma=fbs_chroma,
                                                              lambda_=fbs_lambda)
solved_disp_vis = cv2.ximgproc.getDisparityVis(solved_disp)
solved_filtered_disp_vis = cv2.ximgproc.getDisparityVis(solved_filtered_disp)
cv2.imshow('Solved Disparity Map', solved_disp_vis)
cv2.imshow('Solved Filtered Disparity Map', solved_filtered_disp_vis)
"""
