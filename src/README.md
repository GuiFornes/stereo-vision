# Overview

Modified version of demonstration with ArduCamSDK to compute a stereo depth map with OpenCV
after installing the ArduCamSDK, launch each by each the python scripts (look at the end of this README).

## Install Python 3 and python-opencv
- Install Python 3
 ```bash
 sudo apt-get install python
 ``` 

- Install opencv
```Bash
sudo pip3 install opencv-python
```

# Usage
This project is made of a series of python scripts, to launch from the src repository.
If you are not already in, do this:
```Bash
> $ cd src/
```
### The first 3 scripts depend on each others in the right order, and consist of the calibration of the stereo-system
- **_src/0_test.py_** : Test the camera and allow the user to take a picture with 's' + 'enter' 
(Needed for futur non-video scripts).
 ```bash
sudo python3 0_test.py ../Config/OV5647/stereo/OV5647_MIPI_2Lane_RAW8_8b_2592x1944_BA.cfg
```
- **_src/1_chess_cycle.py_** : Launch this one to take chessboard images for calibration. 
The number of images to take is set in `utils_stereovision.py`.
 ```bash
sudo python3 1_chess_cycle.py ../Config/OV5647/stereo/OV5647_MIPI_2Lane_RAW8_8b_2592x1944_BA.cfg
```
- **_src/2_pairs_cut.py_** : Cut the stereo images into left and right views.
 ```bash
python3 2_pairs_cut.py
```
- **_src/3_calibration_new.py_** : Calibrate the cameras intrinsic and extrinsic.
 ```bash
python3 3_calibration_new.py
```
### The following scripts are independent of each others and can be launched after that the calibration is realized.
- **_src/4_disparity/py_** : Compute and show the disparity map of `src/scenes/photo.png`.
 ```bash
python3 4_disparity.py
```
- **_src/5_disp_video.py_** : Display a live disparity map of the camera entry.
 ```bash
sudo python3 5_disp_video.py ../Config/OV5647/stereo/OV5647_MIPI_2Lane_RAW8_8b_2592x1944_BA.cfg
```
- **_src/6_depth_compute_WLS_old.py_** : Reconstruct the 3D point cloud scene 
from the disparity map of `src/scenes/photo.png`. 
The output is in `src/praxis_filtered.ply`, visionnable with MeshLab
 ```bash
python3 6_depth_compute.py
```