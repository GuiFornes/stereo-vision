import json
import open3d as o3d
import cv2
import numpy as np
import matplotlib.pyplot as plt


try:
    camera_params = json.load(open("camera_params.txt", "r"))
except Exception as e:
    print(e)
    print("Please complete camera_params.txt first.")
    exit(-1)
photo_width = camera_params['width']
photo_height = camera_params['height']
image_width = int(photo_width / 2)
image_height = photo_height

CAMERA_DISTANCE = camera_params['cams_distance']
FOCAL_LENGTH = camera_params['focal_length']  # In pixels /!\
X_A = camera_params['X_A']
X_B = camera_params['X_B']
Y = camera_params['Y']
DOFFS = X_B - X_A

try:
    dispMap = cv2.imread('./disparity_filtered.png', cv2.IMREAD_GRAYSCALE)
except Exception as e:
    print(e)
    print("Please be sure to launch this script from './src/' directory, "
          "and run 4_disparity.py.")
    exit(-1)

try:
    imL = cv2.imread('rectified_left.png')
    imR = cv2.imread('rectified_right.png')
except Exception as e:
    print(e)
    print("Please be sure to run 3_calibration_new.py first.")
    exit(-1)


if __name__ == "__main__":
    print("Calculating depth....", dispMap.shape)
    depth = np.zeros(dispMap.shape)
    coordinates = []
    h, w = dispMap.shape
    for r in range(0, h):
        for c in range(0, w):
            disparity = dispMap[r, c]
            Yoffset = ((h - r) * 2) - Y
            Xoffset = ((w - c) * 2) - X_A
            depth[r, c] = (CAMERA_DISTANCE * FOCAL_LENGTH) / (dispMap[r, c])
            # This will contain x,y,z coordinates with R,G,B values for the pixel
            ZZ = (CAMERA_DISTANCE * FOCAL_LENGTH) / (disparity + DOFFS)
            YY = (ZZ / FOCAL_LENGTH) * Yoffset
            XX = (ZZ / FOCAL_LENGTH) * Xoffset
            coordinates += [[XX, YY, ZZ, imL[r][c][2], imL[r][c][1], imL[r][c][0]]]
    depthmap = plt.imshow(depth, cmap='jet_r')
    plt.colorbar(depthmap)
    #plt.imshow(dispMap, cmap='jet_r')
    plt.show()

    # Saving it into points cloud
    filename = 'praxis_filtered.ply'
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
    with open(filename, 'w') as f:
        f.write(ply_header % dict(vert_num=len(coordinates)))
        np.savetxt(f, coordinates, '%f %f %f %d %d %d')
        f.close()

    # pcd = o3d.io.read_point_cloud('praxis_SGBM.ply')
    # o3d.visualization.draw_geometries([pcd])
