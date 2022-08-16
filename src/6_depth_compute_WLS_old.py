import json
import open3d as o3d
import cv2
import numpy as np
import matplotlib.pyplot as plt
import utils_stereovision as stereovision

try:
    camera_params = json.load(open("camera_params.txt", "r"))
except Exception as e:
    print(e)
    print("Please complete camera_params.txt first.")
    exit(-1)

CAMERA_DISTANCE = 21.50
FOCAL_LENGTH = 3150  # In pixels /!\
X_A = 600  # par rapport à la camera ?
X_B = 700
Y = 500
DOFFS = X_B - X_A

print("focal_length:", FOCAL_LENGTH)
print("cx:", X_A)
print("cy:", Y)
print("baseline:", CAMERA_DISTANCE)
print("doffs:", DOFFS)

# Set up the images
stereo_image = cv2.imread('./scenes/photo.png')
imL, imR = stereovision.splitStereoImage(stereo_image)
rectifiedL, rectifiedR = stereovision.rectifyImages(imL, imR)
matchers = stereovision.Matchers()
disp_map = matchers.computeFilteredDisparityMap(rectifiedL, rectifiedR)


if __name__ == "__main__":
    print("Calculating depth....", disp_map.shape)
    depth = np.zeros(disp_map.shape)
    coordinates = []
    corresponding_color = []
    h, w = disp_map.shape
    for r in range(0, h):
        for c in range(0, w):
            disparity = disp_map[r, c]
            Yoffset = ((h - r) * 2) - Y
            Xoffset = ((w - c) * 2) - X_A
            depth[r, c] = (CAMERA_DISTANCE * FOCAL_LENGTH) / (disp_map[r, c])
            # This will contain
            # x,y,z coordinates with R,G,B values for the pixel
            ZZ = (CAMERA_DISTANCE * FOCAL_LENGTH) / (disparity + DOFFS)
            YY = (ZZ / FOCAL_LENGTH) * Yoffset
            XX = (ZZ / FOCAL_LENGTH) * Xoffset
            coordinates += [[XX, YY, ZZ]]
            corresponding_color += [[rectifiedL[r][c][2], rectifiedL[r][c][1], rectifiedL[r][c][0]]]
    depthmap = plt.imshow(depth, cmap='jet_r')
    plt.colorbar(depthmap)
    #plt.imshow(dispMap, cmap='jet_r')
    plt.show()

    # Saving it into points cloud
    filename = 'praxis_filtered.ply'
    stereovision.write_ply(filename, coordinates, corresponding_color)


