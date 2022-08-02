import os
import sys
import threading
import time
import json

import ArducamSDK

import arducam_config_parser
from ImageConvert import *

global Width, Height, color_mode, running, handle, cfg, save_raw, save_flag, ct, rt
running = True
save_flag = False
save_raw = False
cfg = {}
handle = {}


def configBoard(config):
    """
    One of ArduCamSDK initialization function
    """
    global handle
    ArducamSDK.Py_ArduCam_setboardConfig(handle, config.params[0], config.params[1], config.params[2], config.params[3],
                                         config.params[4:config.params_length])


def camera_initFromFile(fileName):
    """
    One of ArduCamSDK initialization function
    :param fileName: path to camera config file
    """
    global cfg, handle, Width, Height, color_mode, save_raw
    # load config file
    # config = json.load(open(fialeName,"r"))
    config = arducam_config_parser.LoadConfigFile(fileName)

    camera_parameter = config.camera_param.getdict()
    Width = camera_parameter["WIDTH"]
    Height = camera_parameter["HEIGHT"]

    BitWidth = camera_parameter["BIT_WIDTH"]
    ByteLength = 1
    if 8 < BitWidth <= 16:
        ByteLength = 2
        save_raw = True
    FmtMode = camera_parameter["FORMAT"][0]
    color_mode = camera_parameter["FORMAT"][1]
    print("color mode", color_mode)

    I2CMode = camera_parameter["I2C_MODE"]
    I2cAddr = camera_parameter["I2C_ADDR"]
    TransLvl = camera_parameter["TRANS_LVL"]
    cfg = {"u32CameraType": 0x00,
           "u32Width": Width, "u32Height": Height,
           "usbType": 0,
           "u8PixelBytes": ByteLength,
           "u16Vid": 0,
           "u32Size": 0,
           "u8PixelBits": BitWidth,
           "u32I2cAddr": I2cAddr,
           "emI2cMode": I2CMode,
           "emImageFmtMode": FmtMode,
           "u32TransLvl": TransLvl}

    # ret,handle,rtn_cfg = ArducamSDK.Py_ArduCam_open(cfg,0)
    ret, handle, rtn_cfg = ArducamSDK.Py_ArduCam_autoopen(cfg)
    if ret == 0:

        # ArducamSDK.Py_ArduCam_writeReg_8_8(handle,0x46,3,0x00)
        usb_version = rtn_cfg['usbType']
        configs = config.configs
        configs_length = config.configs_length
        for i in range(configs_length):
            type = configs[i].type
            if ((type >> 16) & 0xFF) != 0 and ((type >> 16) & 0xFF) != usb_version:
                continue
            if type & 0xFFFF == arducam_config_parser.CONFIG_TYPE_REG:
                ArducamSDK.Py_ArduCam_writeSensorReg(handle, configs[i].params[0], configs[i].params[1])
            elif type & 0xFFFF == arducam_config_parser.CONFIG_TYPE_DELAY:
                time.sleep(float(configs[i].params[0]) / 1000)
            elif type & 0xFFFF == arducam_config_parser.CONFIG_TYPE_VRCMD:
                configBoard(configs[i])

        ArducamSDK.Py_ArduCam_registerCtrls(handle, config.controls, config.controls_length)
        # ArducamSDK.Py_ArduCam_setCtrl(handle, "setFramerate", 5)
        # ArducamSDK.Py_ArduCam_setCtrl(handle, "setExposure", 10)
        # ArducamSDK.Py_ArduCam_setCtrl(handle, "setExposureTime", 33000)
        # ArducamSDK.Py_ArduCam_setCtrl(handle, "setGain", 5)
        # ArducamSDK.Py_ArduCam_setCtrl(handle, "setAnalogueGain", 100)

        ArducamSDK.Py_ArduCam_writeSensorReg(handle, 0x3503, 0x02)

        ArducamSDK.Py_ArduCam_writeSensorReg(handle, 0x350A, 0b00011000)
        ArducamSDK.Py_ArduCam_writeSensorReg(handle, 0x350B, 0b00111100)


        rtn_val, datas = ArducamSDK.Py_ArduCam_readUserData(handle, 0x400 - 16, 16)
        print("Serial: %c%c%c%c-%c%c%c%c-%c%c%c%c" % (datas[0], datas[1], datas[2], datas[3],
                                                      datas[4], datas[5], datas[6], datas[7],
                                                      datas[8], datas[9], datas[10], datas[11]))

        return True
    else:
        print("open fail,rtn_val = ", ret)
        return False


def captureImage_thread():
    """
    This function is used to capture image from the camera and make them available in the handle
    :return:
    """
    global handle, running

    rtn_val = ArducamSDK.Py_ArduCam_beginCaptureImage(handle)
    if rtn_val != 0:
        print("Error beginning capture, rtn_val = ", rtn_val)
        running = False
        return
    else:
        print("Capture began, rtn_val = ", rtn_val)

    while running:
        # print "capture"
        rtn_val = ArducamSDK.Py_ArduCam_captureImage(handle)
        if rtn_val > 255:
            print("Error capture image, rtn_val = ", rtn_val)
            if rtn_val == ArducamSDK.USB_CAMERA_USB_TASK_ERROR:
                break

    running = False
    ArducamSDK.Py_ArduCam_endCaptureImage(handle)


try:
    camera_params = json.load(open("camera_params.txt", "r"))
except Exception as e:
    print(e)
    print("Please run 1_test.py first.")
    exit(-1)
width = camera_params['width']
height = camera_params['height']


def get_frame():
    """
    This function is used to get a frame from the handle
    :return:
    """
    while ArducamSDK.Py_ArduCam_availableImage(handle) <= 0:
        time.sleep(0.01)
    while ArducamSDK.Py_ArduCam_availableImage(handle) > 1:
        ArducamSDK.Py_ArduCam_del(handle)
    rtn_val, data, rtn_cfg = ArducamSDK.Py_ArduCam_readImage(handle)
    datasize = rtn_cfg['u32Size']
    if rtn_val != 0 or datasize == 0:
        print("Error read image, rtn_val = ", rtn_val)
        return get_frame()
    image = convert_image(data, rtn_cfg, color_mode)
    image = cv2.resize(image, (width, height))
    # image = image[:, 250:1670]

    # Delete the frame from the handle
    ArducamSDK.Py_ArduCam_del(handle)
    return image


def readImage_thread():
    """
    Initial function to read and process the image, replaced by our own for processing.
    """
    global handle, running, Width, Height, save_flag, cfg, color_mode, save_raw
    global COLOR_BayerGB2BGR, COLOR_BayerRG2BGR, COLOR_BayerGR2BGR, COLOR_BayerBG2BGR
    count = 0
    totalFrame = 0
    time0 = time.time()
    time1 = time.time()
    data = {}
    cv2.namedWindow("ArduCam Demo", 1)
    if not os.path.exists("images"):
        os.makedirs("images")
    while running:
        image = get_frame()
        if image is None:
            time.sleep(0.01)
            continue

        # Compute fps
        time1 = time.time()
        if time1 - time0 >= 1:
            print("%s %d %s\n" % ("fps:", count, "/s"))
            count = 0
            time0 = time1
        count += 1

        # Save image
        if save_flag:
            cv2.imwrite("images/image%d.jpg" % totalFrame, image)
            if save_raw:
                with open("images/image%d.raw" % totalFrame, 'wb') as f:
                    f.write(data)
            totalFrame += 1

        # Show image
        image = cv2.resize(image, (1280, 480), interpolation=cv2.INTER_LINEAR)
        cv2.imshow("ArduCam Demo", image)
        cv2.waitKey(10)


def commands_thread():
    """
    Launched in parallel thread, this function allows to execute orders from keyboards entries
    """
    global running, save_flag
    while running:
        input_kb = str(sys.stdin.readline()).strip("\n")

        if input_kb == 'q' or input_kb == 'Q':
            running = False
        if input_kb == 's' or input_kb == 'S':
            save_flag = True


def showHelp():
    print(" usage: sudo python ArduCam_Py_Demo.py <path/config-file-name>	\
        \n\n example: sudo python ArduCam_Py_Demo.py ../../../python_config/AR0134_960p_Color.json	\
        \n\n While the program is running, you can press the following buttons in the terminal:	\
        \n\n 'q' + Enter:Stop running the program.	\
        \n\n")


def init():
    """
    Initialize the camera, the handle, and the image-capturer and command threads
    """
    global ct, rt
    config_file_name = ""
    if len(sys.argv) > 1:
        config_file_name = sys.argv[1]

        if not os.path.exists(config_file_name):
            print("Config file does not exist.")
            return 1
    else:
        showHelp()
        return 1

    if camera_initFromFile(config_file_name):
        ArducamSDK.Py_ArduCam_setMode(handle, ArducamSDK.CONTINUOUS_MODE)
        ct = threading.Thread(target=captureImage_thread)
        rt = threading.Thread(target=commands_thread)
        ct.start()
        rt.start()
        return 0


def close():
    """
    Close the threads and the camera
    :return:
    """
    global ct, rt
    ct.join()
    rt.join()

    rtn_val = ArducamSDK.Py_ArduCam_close(handle)
    if rtn_val == 0:
        print("device close success!")
    else:
        print("device close fail!")


def increase_brightness(img, value=1):  # Unused yet
    """
    Increase the brightness of the image
    :param img: image entry
    :param value: value to increase the brightness
    :return: brighter image
    """
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(hsv)

    print("old v:", v, "value:", value)
    lim = 255 - value
    v[v > lim] = 255
    v[v <= lim] += np.uint8(value)
    print("new v:", v)

    final_hsv = cv2.merge((h, s, v))
    img = cv2.cvtColor(final_hsv, cv2.COLOR_HSV2BGR)
    return img
