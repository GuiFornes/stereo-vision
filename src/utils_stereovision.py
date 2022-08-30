import cv2
import numpy as np
import time
import open3d as o3d

########################################
# Global variables preset
########################################
TOTAL_CALIB_PICS = 303  # nb of pictures used for calibration
CHESSBOARD_SIZE = (6, 9)
SQUARE_SIZE = 22  # <- A4 printed chessboard, otherwise chessboard on my phone -> 7.25  # in mm


def rectifyImages(imL, imR, calib_data='calib_data/calibration.npz', cropped=False):
    """
    Rectify stereo pair
    :param imR: right original image
    :param imL: left original image
    :param calib_data: saved calibration data
    :param cropped: if True, the images are cropped on the ROI
    :return: rectified left and right images
    """
    # Load the saved data
    with np.load(calib_data) as data:
        leftMapX = data['leftMapX']
        leftMapY = data['leftMapY']
        leftROI = data['leftROI']
        rightMapX = data['rightMapX']
        rightMapY = data['rightMapY']
        rightROI = data['rightROI']

    # Un-distort the images
    rectifiedL = cv2.remap(imL, leftMapX, leftMapY, cv2.INTER_LINEAR)
    rectifiedR = cv2.remap(imR, rightMapX, rightMapY, cv2.INTER_LINEAR)

    # Crop the images
    if cropped:
        rectifiedL = rectifiedL[leftROI[1]:leftROI[1] + leftROI[3], leftROI[0]:leftROI[0] + leftROI[2]]
        rectifiedR = rectifiedR[rightROI[1]:rightROI[1] + rightROI[3], rightROI[0]:rightROI[0] + rightROI[2]]
    return rectifiedL, rectifiedR


def crop_ROI(img, ROI):
    """
    Crop an image
    :param img: input image
    :param ROI: region of interest
    :return: cropped image
    """
    print("ROI : ", ROI)
    return img[ROI[1]:ROI[1] + ROI[3], ROI[0]:ROI[0] + ROI[2]]


def splitStereoImage(stereo_image):
    """
    Split a stereo pair
    :param stereo_image: a frame from utils_arducam.getFrame()
    :return: left and right original images
    """
    imgWidth = stereo_image.shape[1] // 2
    imgL = stereo_image[:, :imgWidth, :]
    imgR = stereo_image[:, imgWidth:, :]
    return imgL, imgR


def rectifyStereoImage(pair):
    """
    Split and rectify a stereo pair
    :param pair: a frame from utils_arducam.getFrame()
    :return: rectified left and right images
    """
    imgL, imgR = splitStereoImage(pair)
    rectifiedL, rectifiedR = rectifyImages(imgL, imgR)
    return rectifiedL, rectifiedR


class Matchers:
    wls_lambda = 8000.0
    wls_sigma = 1.5

    def __init__(self, method='SGBM', filtered=True, verbose=True,
                 min_disp=0,
                 max_disp=11,
                 w_size=7,
                 disp12MaxDiff=10,
                 preFilterCap=29,
                 uniquenessRatio=15,
                 speckleWindowSize=100,
                 speckleRange=100):
        self._method = method
        self._filtered = filtered
        self._verbose = verbose
        self.w_size = w_size
        self.max_disp = max_disp
        if self._verbose:
            print('[INFO] Instancing stereo-matcher, method used: ', self._method)
        if self._method == 'SGBM':
            self.left_matcher = cv2.StereoSGBM_create(minDisparity=min_disp,
                                                      numDisparities=self.max_disp*16,
                                                      blockSize=self.w_size,
                                                      P1=24 * self.w_size * self.w_size,
                                                      P2=96 * self.w_size * self.w_size,
                                                      disp12MaxDiff=disp12MaxDiff,  # 12
                                                      preFilterCap=preFilterCap,  # 32
                                                      uniquenessRatio=uniquenessRatio,  # 10
                                                      speckleWindowSize=speckleWindowSize,  # 50
                                                      speckleRange=speckleRange,  # 32
                                                      mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY)
        elif self._method == 'BM':
            self.w_size = 7
            self.left_matcher = cv2.StereoBM_create(numDisparities=self.max_disp,
                                                    blockSize=self.w_size)
            self.left_matcher.setMinDisparity(0)
            self.left_matcher.setPreFilterCap(29)
            self.left_matcher.setUniquenessRatio(15)
            self.left_matcher.setSpeckleWindowSize(100)
            self.left_matcher.setSpeckleRange(2)
            self.left_matcher.setTextureThreshold(13)
        else:
            print('[ERROR] Invalid matching method')
            exit(0)
        self.right_matcher = cv2.ximgproc.createRightMatcher(self.left_matcher)
        if self._filtered:
            if self._verbose:
                print('[INFO] Instancing WLS filter')
            self.wls_filter = cv2.ximgproc.createDisparityWLSFilter(matcher_left=self.left_matcher)
            self.wls_filter.setLambda(self.wls_lambda)
            self.wls_filter.setSigmaColor(self.wls_sigma)

    def __computeFeaturesMatching(self, imL, imR):
        """ Compute disparity map """
        matching_time = time.time()
        left_disp = self.left_matcher.compute(imL, imR)
        right_disp = self.right_matcher.compute(imR, imL)
        matching_time = time.time() - matching_time
        if self._verbose:
            print("[INFO] Matching time: ", matching_time)
        return left_disp, right_disp

    def __filterDisparity(self, imL, imR, left_disp, right_disp):
        """ Filter disparity map """
        if not self._filtered:
            return left_disp
        filtering_time = time.time()
        filtered_disp = self.wls_filter.filter(left_disp, imL, disparity_map_right=right_disp, right_view=imR)
        filtering_time = time.time() - filtering_time
        if self._verbose:
            print("[INFO] Filtering time: ", filtering_time)
        return filtered_disp

    ########################################################
    # Public methods
    ########################################################
    def computeRawDisparityMap(self, imL, imR):
        """
        Compute disparity map
        :param imL: left rectified image
        :param imR: right rectified image
        :return: raw disparity map (without filtering)
        """
        if self._verbose:
            print("[INFO] Computing raw disparity map...")
        left_disp, _ = self.__computeFeaturesMatching(imL, imR)
        left_disp_vis = cv2.ximgproc.getDisparityVis(left_disp)
        return left_disp_vis

    def computeFilteredDisparityMap(self, imL, imR):
        """ Compute disparity map """
        if self._verbose:
            print("[INFO] Computing filtered disparity map...")
        left_disp, right_disp = self.__computeFeaturesMatching(imL, imR)
        filtered_disp = self.__filterDisparity(imL, imR, left_disp, right_disp)
        filtered_disp_vis = cv2.ximgproc.getDisparityVis(filtered_disp)
        return filtered_disp_vis

    def getConfidenceMap(self):
        """ Compute disparity map """
        return self.wls_filter.getConfidenceMap()


def write_ply(fn, verts, colors=None):
    """
    Write point cloud to PLY file.
    :param fn: output file name
    :param verts: coordinates of the points
    :param colors: corresponding colors
    """
    ply_header = '''ply
    format ascii 1.0
    element vertex %(vert_num)d
    property float x
    property float y
    property float z
    property uchar red
    property uchar green
    property uchar blue
    end_header
    '''
    out_colors = colors.copy()
    verts = verts.reshape(-1, 3)
    verts = np.hstack([verts, out_colors])
    with open(fn, 'wb') as f:
        f.write((ply_header % dict(vert_num=len(verts))).encode('utf-8'))
        np.savetxt(f, verts, fmt='%f %f %f %d %d %d ')


def sphere_crop_ply(pcd, radius=175):
    """
    Crop the given point cloud around a centered sphere.
    :param pcd: open3d point cloud
    :param radius: radius of the sphere
    :return: cropped point cloud
    """
    points = np.asarray(pcd.points)
    colors = np.asarray(pcd.colors)
    center = pcd.get_center()
    distances = np.linalg.norm(points - center, axis=1)
    indexes = np.where(distances <= radius)[0]
    pcd.points = o3d.utility.Vector3dVector(points[indexes])
    pcd.colors = o3d.utility.Vector3dVector(colors[indexes])
    return pcd


def display_inlier_outlier(cloud, ind):
    """
    Visualize the filtration step of the given point cloud.
    Showing outliers (red) and inliers (gray)
    :param cloud: open3d point cloud
    :param ind: index of deleted points
    """
    inlier_cloud = cloud.select_by_index(ind)
    outlier_cloud = cloud.select_by_index(ind, invert=True)
    outlier_cloud.paint_uniform_color([1, 0, 0])
    inlier_cloud.paint_uniform_color([0.8, 0.8, 0.8])
    o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud],
                                      zoom=0.3412,
                                      front=[0.4257, -0.2125, -0.8795],
                                      lookat=[2.6172, 2.0475, 1.532],
                                      up=[-0.0694, -0.9768, 0.2024])
