import json
import open3d as o3d
import cv2
import numpy as np
import utils_stereovision as stereovision

# Set up the images
stereo_image = cv2.imread('./scenes/photo.png')
imL, imR = stereovision.splitStereoImage(stereo_image)
w, h = imL.shape[:2]
rectifiedL, rectifiedR = stereovision.rectifyImages(imL, imR)
try:
    dispMap = cv2.imread('./disparity_filtered.png', cv2.IMREAD_GRAYSCALE)
except Exception as e:
    print(e)
    print("Please be sure to launch this script from './src/' directory, "
          "and run 4_disparity.py.")
    exit(-1)

focal_length = 337

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


def write_ply(fn, verts, colors):
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


if __name__ == "__main__":
    points = cv2.reprojectImageTo3D(dispMap, Q2)
    print("points : ", points.shape)

    # reflect on x-axis
    # reflect_matrix = np.identity(3)
    # reflect_matrix[0] *= -1
    # points = np.matmul(points, reflect_matrix)

    # extract colors from image
    colors = cv2.cvtColor(imL, cv2.COLOR_BGR2RGB)

    # filter by min disparity
    mask = dispMap > dispMap.min()
    out_points = points[mask]
    out_colors = colors[mask]
    print("out_points : ", out_points.shape)
    print("out_colors : ", out_colors.shape)

    write_ply('out.ply', out_points, out_colors)
    print('%s saved' % 'out.ply')

    # pcd = o3d.io.read_point_cloud('praxis_SGBM.ply')
    # o3d.visualization.draw_geometries([pcd])
