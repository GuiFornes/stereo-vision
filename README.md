# _stereo-vision_

## Overview
This project is about stereo vision and 3D reconstruction, 
as part of a tactile sensor project from CoRo lab.

## Repository
- **_Config_** : Contains the config files of the OV5647 camera.
- **_src_** : Contains the source code of the project.
- **_src/scenes_** : Contains the calibration chess pictures.
- **_src/PNGs_** : Contains example stereo scenes to reproject into 3D.
- **_src/PLYs_** : Contains the 3D reconstruction examples.

## Install Python 3 and OpenCV
- Install Python 3
 ```bash
 sudo apt-get install python
 ``` 

- Install OpenCV with pip3
```Bash
sudo pip3 install opencv-contrib-python
```

## Build and install arducam_config_parser 
- Before use, Build :
```Bash
  cd src/
  git clone https://github.com/ArduCAM/arducam_config_parser.git
  cd arducam_config_parser/src
  make clean && make
  make install
  cd ../../
```

## How to launch the project
This project is made of a series of python scripts, to launch from the src directory.
```Bash
cd src/
```
Next, watch to the `src/README.md` to see more about the scripts to launch.

## References

Those are different projects that I used to build this one.

 - **_ArduCAM_** : source files to launch a camera with arduCam module.

https://github.com/ArduCAM/arducam_config_parser.git  
https://github.com/ArduCAM/ArduCAM_USB_Camera_Shield.git  

 - **_Others stereo-vision projects_** :

https://github.com/kanavsinglaa/StereoVision.git  
https://github.com/ArduCAM/MIPI_Camera.git  
https://github.com/realizator/stereopi-fisheye-robot.git  
https://github.com/niconielsen32/ComputerVision.git  