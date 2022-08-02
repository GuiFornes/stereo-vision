# Overview

Modified version of demonstration with ArduCamSDK to compute a stereo depth map with OpenCV
after installing the ArduCamSDK, launch each by each the python scripts (look at the end of this README).

# Install library and OpenCV Environment

```
- Install  
```Bash
  make install
```

## Downlaod and install the latest libusb 
- Download the [libusb](https://sourceforge.net/projects/libusb/files/libusb-1.0/) 
- Copy the libusb-xxx.tar.bz to the Pi then run the following command to unzip it.[xxx：version number]
```Bash
tar -jxvf  libusb-xxx.tar.bz2  
```
- Before compilation, Run the following commands to config it  
```Bash
cd libusb-xxx 
./configure --disable-udev
```
- Install the libusb library 
```Bash
sudo make install
```
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
This project is made of a series of python scripts, to launch from the src file.
> $ cd src/

- **_src/0_test.py_** : Test the camera and allow the user to take a picture with 's' + 'enter'.
> $ sudo python3 src/0_test.py ../Config/OV5647/stereo/OV5647_MIPI_2Lane_RAW8_8b_2592x1944_BA.cfg
- **_src/1_chess_cycle.py_** : Launch it to take chessboard images for calibration.
> $ sudo python3 src/1_chess_cycle.py ../Config/OV5647/stereo/OV5647_MIPI_2Lane_RAW8_8b_2592x1944_BA.cfg
- **_src/2_pairs_cut.py_** : Cut the stereo images into left and right views.
> $ python3 src/2_pairs_cut.py
- **_src/3_calibration_new.py_** : Calibrate the cameras intrinsic and extrinsic.
> $ python3 src/3_calibration_new.py
- **_src/4_disparity/py_** : Compute and show the disparity map of `src/scenes/photo.png`.
> $ python3 src/4_disparity/py/disparity.py
- **_src/5_disp_video.py_** : Display a live disparity map of the camera entry.
> $ sudo python3 src/5_disp_video.py ../Config/OV5647/stereo/OV5647_MIPI_2Lane_RAW8_8b_2592x1944_BA.cfg
- **_src/6_depth_compute_WLS_old.py_** : Reconstruct the 3D point cloud scene 
from the disparity map of `src/scenes/photo.png`. 
The output is in `src/praxis_filtered.ply`, visionable with Open3d or MeshLab
> $ python3 src/6_depth_compute_WLS_old.py