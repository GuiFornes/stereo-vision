import time
import open3d as o3d
import cv2
import numpy as np
import utils_stereovision as stereovision

###########################################################
# Execution parameters
###########################################################
SAVE_PLY = True  # save the output point cloud as ply file
FILTER_PLY = True  # filter the noise in the point cloud and crop it
DISPLAY_FILTERING_STEP = True  # display the point cloud filtering steps

###########################################################
# Compute disparity map
###########################################################
stereo_image = cv2.imread('./scenes/photo.png')
imL, imR = stereovision.splitStereoImage(stereo_image)
w, h = imL.shape[:2]
rectifiedL, rectifiedR = stereovision.rectifyImages(imL, imR)
matchers = stereovision.Matchers(verbose=False)
disp_map = matchers.computeFilteredDisparityMap(rectifiedL, rectifiedR)

# Getting datas from calibration to crop the disparity map and compute depth
with np.load('calib_data/calibration.npz') as data:
    Q = data['disparityToDepthMap']
    leftROI = data['leftROI']

print("dispMap size : ", disp_map.shape)
print("ROI : ", leftROI)
leftROI = (leftROI[0] * 1, leftROI[1] * 1, 960 - (960 - leftROI[2]) * 1, 720 - (720 - leftROI[3]) * 1)
disp_map = disp_map[leftROI[1]:leftROI[1] + leftROI[3], leftROI[0]:leftROI[0] + leftROI[2]]
rectifiedL = rectifiedL[leftROI[1]:leftROI[1] + leftROI[3], leftROI[0]:leftROI[0] + leftROI[2]]
print("disp_map cropped size : ", disp_map.shape)

cv2.imshow("disp_map", disp_map)
cv2.imshow("rectifiedL", rectifiedL)
cv2.waitKey(0)
cv2.destroyAllWindows()

if __name__ == "__main__":
    # Computing depth
    t = time.time()
    points = cv2.reprojectImageTo3D(disp_map, Q)
    t_bis = time.time()
    print("[INFO] Reprojection time : {}".format(t_bis - t))

    # Reflect on x-axis
    reflect_matrix = np.identity(3)
    reflect_matrix[0] *= -1
    points = np.matmul(points, reflect_matrix)

    # Extract colors from image
    colors = cv2.cvtColor(rectifiedL, cv2.COLOR_BGR2RGB)

    # Filter by min disparity
    mask = disp_map > disp_map.min()
    out_points = points[mask]
    out_colors = colors[mask]
    t = time.time()
    print("[INFO] Applying mask time : {}".format(t - t_bis))

    # Create open3d point cloud
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(out_points)
    norm_colors = out_colors / 255.0
    pcd.colors = o3d.utility.Vector3dVector(norm_colors)
    t_bis = time.time()
    print('[INFO] open3d point cloud instantiating time : {}'.format(t_bis - t))

    # Filter the point cloud
    if FILTER_PLY:
        blur_pcd = pcd.voxel_down_sample(voxel_size=0.05)
        print('[INFO] Point cloud blurred')
        filtered_pcd, ind1 = blur_pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=1.5)
        print('[INFO] Point cloud filtered')
        double_filtered_pcd, ind2 = filtered_pcd.remove_radius_outlier(nb_points=10, radius=3)
        print('[INFO] Point cloud double filtered')
        final_pcd = stereovision.sphere_crop_ply(double_filtered_pcd, radius=175)
        print('[INFO] Point cloud cropped')
        t = time.time()
        print('[INFO] Point cloud filtering time : {}'.format(t - t_bis))

        if DISPLAY_FILTERING_STEP:
            o3d.visualization.draw_geometries([pcd], window_name='Original point cloud',
                                              zoom=0.3412,
                                              front=[0.4257, -0.2125, -0.8795],
                                              lookat=[2.6172, 2.0475, 1.532],
                                              up=[-0.0694, -0.9768, 0.2024])
            o3d.visualization.draw_geometries([blur_pcd], window_name='Blurred point cloud',
                                              zoom=0.3412,
                                              front=[0.4257, -0.2125, -0.8795],
                                              lookat=[2.6172, 2.0475, 1.532],
                                              up=[-0.0694, -0.9768, 0.2024])
            stereovision.display_inlier_outlier(blur_pcd, ind1)
            stereovision.display_inlier_outlier(filtered_pcd, ind2)
            o3d.visualization.draw_geometries([double_filtered_pcd], window_name='Final point cloud,',
                                              zoom=0.3412,
                                              front=[0.4257, -0.2125, -0.8795],
                                              lookat=[2.6172, 2.0475, 1.532],
                                              up=[-0.0694, -0.9768, 0.2024])
            o3d.visualization.draw_geometries([final_pcd], window_name='Final point cloud,',
                                              zoom=0.3412,
                                              front=[0.4257, -0.2125, -0.8795],
                                              lookat=[2.6172, 2.0475, 1.532],
                                              up=[-0.0694, -0.9768, 0.2024])

    if SAVE_PLY:
        t_bis = time.time()
        filename = 'out.ply'
        stereovision.write_ply(filename, out_points, out_colors)
        t = time.time()
        print('[INFO] %s saved' % filename, ', ply saving time : {}'.format(t - t_bis))
