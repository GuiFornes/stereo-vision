import time
import open3d as o3d
import cv2
import numpy as np
import utils_stereovision as stereovision
import utils_arducam as arducam


if __name__ == "__main__":
    # Initialisation of the camera
    ret_val = arducam.init()
    if ret_val:
        exit()
    matchers = stereovision.Matchers(verbose=False)
    with np.load('calib_data/calibration.npz') as data:
        Q = data['disparityToDepthMap']
    vis = o3d.visualization.Visualizer()
    vis.create_window()
    # geometry is the point cloud used in your animation
    geometry = o3d.geometry.PointCloud()
    vis.add_geometry(geometry)
    # Capturing loop
    while arducam.running:
        image = arducam.getFrame()
        imL, imR = stereovision.splitStereoImage(image)
        w, h = imL.shape[:2]
        rectifiedL, rectifiedR = stereovision.rectifyImages(imL, imR)
        disp_map = matchers.computeFilteredDisparityMap(rectifiedL, rectifiedR)
        points = cv2.reprojectImageTo3D(disp_map, Q)
        mask = disp_map > disp_map.min()
        out_points = points[mask]
        geometry.points = o3d.utility.Vector3dVector(points)
        vis.update_geometry(geometry)
        vis.poll_events()
        vis.update_renderer()
        # Show image
        cv2.imshow("ArduCam Demo", image)
        cv2.waitKey(1)

    # Close the camera and release the resources
    arducam.close()
