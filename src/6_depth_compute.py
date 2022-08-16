import time
import open3d as o3d
import cv2
import numpy as np
import utils_stereovision as stereovision

# Set up the images
stereo_image = cv2.imread('./scenes/photo.png')
imL, imR = stereovision.splitStereoImage(stereo_image)
w, h = imL.shape[:2]
rectifiedL, rectifiedR = stereovision.rectifyImages(imL, imR)
matchers = stereovision.Matchers()
disp_map = matchers.computeFilteredDisparityMap(rectifiedL, rectifiedR)

focal_length = 277.4182306527777  # 357.03910492

with np.load('calib_data/calibration.npz') as data:
    Q = data['disparityToDepthMap']

# Perspective transformation matrix
# This transformation matrix is from the openCV documentation, didn't seem to work for me.
Q1 = np.float32([[1, 0, 0, -w / 2.0],
                 [0, -1, 0, h / 2.0],
                 [0, 0, 0, -focal_length],
                 [0, 0, 1, 0]])

# This transformation matrix is derived from Prof. Didier Stricker's power point presentation on computer vision.
# Link : https://ags.cs.uni-kl.de/fileadmin/inf_ags/3dcv-ws14-15/3DCV_lec01_camera.pdf
Q2 = np.float32([[1, 0, 0, 0],
                 [0, -1, 0, 0],
                 [0, 0, focal_length * 0.05, 0],
                 [0, 0, 0, 1]])

print("Q : ", Q)
print("\nQ1 : ", Q1)
print("\nQ2 : ", Q2)



if __name__ == "__main__":
    t = time.time()
    points = cv2.reprojectImageTo3D(disp_map, Q)
    tbis = time.time()
    print("[INFO] Reprojection time : {}".format(tbis - t))
    # reflect on x-axis
    # reflect_matrix = np.identity(3)
    # reflect_matrix[0] *= -1
    # points = np.matmul(points, reflect_matrix)

    # extract colors from image
    colors = cv2.cvtColor(rectifiedL, cv2.COLOR_BGR2RGB)

    # filter by min disparity
    mask = disp_map > disp_map.min()
    out_points = points[mask]
    out_colors = colors[mask]
    t = time.time()
    print("[INFO] Applying mask time : {}".format(t - tbis))

    fname = 'out.ply'
    stereovision.write_ply(fname, out_points, out_colors)
    tbis = time.time()
    print('[INFO] %s saved' % 'out.ply', ', ply saving time : {}'.format(tbis - t))

    pcd = o3d.io.read_point_cloud(fname)
    o3d.visualization.draw_geometries([pcd])
