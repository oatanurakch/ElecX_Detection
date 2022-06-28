import platform as pf

import cv2 as cv
import numpy as np

import mvsdk as sdk

class Detect_Camera:
    def __init__(self) -> None:
        pass
    
    # Brightness Stream
    def Brightness(self, img, value = 50):
        hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
        h, s, v = cv.split(hsv)
        lim = 255 - value
        v[v > lim] = 255
        v[v <= lim] += value

        final_hsv = cv.merge((h, s, v))
        img = cv.cvtColor(final_hsv, cv.COLOR_HSV2BGR)
        return img

    # Main loop for detect camera
    def main_loop(self):
        # Call method for detect camera
        self.DevList = sdk.CameraEnumerateDevice()
        self.DevList_quality = len(self.DevList)
        
        if self.DevList_quality < 1:
            print('Can\'t find camera!')
            return False
        else:
            print(f"Found: {self.DevList_quality} device{'s' * (self.DevList_quality > 1)}")
        for self.i, self.DevInfo in enumerate(self.DevList):
            print(f'{self.i}: {self.DevInfo.GetProductName()}')
        
        self.select_camera = 0 if self.DevList_quality == 1 else self.ChooseCamera()

    # select camera when found camera more than 1 device
    def ChooseCamera(self):
        while True:
            self.select_camera = input(f'Select camera: 0 - {self.DevList_quality - 1}: ')
            if self.select_camera.isnumeric():
                if int(self.select_camera) >= 0 and int(self.select_camera) < self.DevList_quality:
                    self.select_camera = int(self.select_camera)
                    break
                else:
                    print('Wrong index! Plese input again!')
            else:
                print('Wrong type! Please input integer again!')
        self.DevInfo = self.DevList[self.select_camera]
        print(f'Selected camera is {self.select_camera}: {(self.DevList[self.select_camera]).GetProductName()}')
    
    # Initial Camera
    def InitialCamera(self):
        self.h_camera = 0
        try:
            self.h_camera = sdk.CameraInit(self.DevInfo, -1, -1)
            print('Camera Init Success!')
        except sdk.CameraException:
            print(f'Camera Init Failed({sdk.CameraException.error_code}):{sdk.CameraException.message}')

    # Start Camera
    def StartCamera(self):
        self.cap = sdk.CameraGetCapability(self.h_camera)
        self.monoCamera = (self.cap.sIspCapacity.bMonoSensor != 0)
        if self.monoCamera:
            sdk.CameraSetIspOutFormat(self.h_camera, sdk.CAMERA_MEDIA_TYPE_MONO8)
        else:
            sdk.CameraSetIspOutFormat(self.h_camera, sdk.CAMERA_MEDIA_TYPE_BGR8)

        sdk.CameraSetTriggerMode(self.h_camera, 0)

        sdk.CameraSetAeState(self.h_camera, 0)
        sdk.CameraSetExposureTime(self.h_camera, 30 * 1000)

        sdk.CameraPlay(self.h_camera)
        
        self.FrameBufferSize = self.cap.sResolutionRange.iWidthMax * self.cap.sResolutionRange.iHeightMax * (1 if self.monoCamera else 3)

        self.pFrame = sdk.CameraAlignMalloc(self.FrameBufferSize, 16)

        while True:
            self.key = cv.waitKey(1)
            if self.key == 27:
                break
            try:
                self.pRawData, self.FrameHead = sdk.CameraGetImageBuffer(self.h_camera, 200)
                sdk.CameraImageProcess(self.h_camera, self.pRawData, self.pFrame, self.FrameHead)
                sdk.CameraReleaseImageBuffer(self.h_camera, self.pRawData)

                if pf.system() == 'windows':
                    sdk.CameraFlipFrameBuffer(self.pFrame, self.FrameHead, 1)
                
                self.frame_data = (sdk.c_ubyte * self.FrameHead.uBytes).from_address(self.pFrame)
                self.frame = np.frombuffer(self.frame_data, dtype = np.uint8)
                self.frame = self.frame.reshape((self.FrameHead.iHeight, self.FrameHead.iWidth, 1 if self.FrameHead.uiMediaType == sdk.CAMERA_MEDIA_TYPE_MONO8 else 3))
                # Increase Brightness
                # self.frame = self.Brightness(self.frame)
                self.frame = cv.resize(self.frame, (int(self.frame.shape[1] * 70 / 100), int(self.frame.shape[0] * 70 / 100)), interpolation = cv.INTER_AREA)
                cv.imshow('Test Call Camera', self.frame)
            except sdk.CameraException:
                if sdk.CameraException != sdk.CAMERA_STATUS_TIME_OUT:
                    print("CameraGetImageBuffer failed({}): {}".format(sdk.CameraException.error_code, sdk.CameraException.message))

        sdk.CameraUnInit(self.h_camera)
        sdk.CameraAlignFree(self.pFrame)

# Create Object for use class OOP
call_camera = Detect_Camera()
call_camera.main_loop()
call_camera.InitialCamera()
call_camera.StartCamera()

cv.destroyAllWindows()