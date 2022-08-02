# _stereo-vision_

## Overview
This project is about stereo vision and 3D reconstruction, 
as part of a tactile sensor project from CoRo lab.

## Repository
- **_Config_** : Contains the config files of the OV5647 camera.
- **_src_** : Contains the source code of the project.
- **_src/scenes_** : Contains the calibration chess pictures.

## Build and install arducam_config_parser 
- Before use, Build :
```Bash
  git clone https://github.com/ArduCAM/arducam_config_parser.git
  cd arducam_config_parser/src
  make clean && make
```
## How to launch the project
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
The output is in `src/praxis_filtered.ply`, displayable with _Open3d_ or _MeshLab_
> $ python3 src/6_depth_compute_WLS_old.py