""" 
System detection should run this file

"""

# GUI library
from ast import Add
from tkinter.tix import Control
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
from grpc import Channel

# Main GUI
from UI.gui import Ui_MainWindow
# Exposure setup
import UI.gui_exposure
# Manage model in system
import UI.gui_modelmanage
# Threshold window
import UI.gui_threshold
# Delay window
import UI.gui_delay
# IP Config windows
import UI.gui_ipconfig
# Setup Output Channel
import UI.gui_setupOutputChannel
# Selected I/O Module
import UI.gui_selectedIOModule

# Other library
import os
import sys
import cv2
import numpy as np
import sys
import os
import cv2
import json
import numpy as np
import platform as pf
import time as tm
from datetime import datetime
from threading import Thread as th
from pathlib import Path
from natsort import *

# capture module
from InitialSetup.Code.capture_cv2 import CaptureImage
CaptureImg = CaptureImage()
# Compare answer module
from InitialSetup.Code.compareAns import CompareAns
CompareAns = CompareAns()
# Border setting camera
from InitialSetup.Code.load_cameraUsed_Border import CameraUsedandBorderCamera
CameraUsed_Border = CameraUsedandBorderCamera()
# lode output control
from InitialSetup.Code.controlOutput import ControlOutputSystem
ControlOutputSystem = ControlOutputSystem()
# Load input system
from InitialSetup.Code.ControlInput import ControlInputSystem
ControlInputSystem = ControlInputSystem()
# Inititial Setting before run system
from InitialSetup.Code.loadInitialSettingInGUI import LoadInitialConf, UpdateConfig, SetupIOControl
# Load initial config
LoadInitialConf = LoadInitialConf()
# Update config when change some setting
UpdateConfig = UpdateConfig()
# Config IP of I/O Control
SetupIOControl = SetupIOControl()
# Update model config file
from InitialSetup.UpdateWrite_configModel import SetupPathModel
SetupPathModel = SetupPathModel()
# Module load modelname for used detection
from InitialSetup.Code.load_ModelUsed import LoadModelName
LoadModelName = LoadModelName()

# Load detection system
from InitialSetup.customdetect import detect
detect = detect()

# Base GUI
app = QtWidgets.QApplication(sys.argv)
MainWindow = QtWidgets.QMainWindow()

# ROOT Path
FILE = Path(__file__).resolve()
ROOT = FILE.parents[0]

# Import Camera SDK
from Camera import mvsdk as sdk

# Class Main GUI
class GUISystem(Ui_MainWindow):
    def __init__(self):
        super().setupUi(MainWindow)
        MainWindow.setWindowTitle('System')
        # Set windows icons
        MainWindow.setWindowIcon(QIcon(os.path.join(ROOT, r'imagefile\detection.png')))
        # Set Main UI window it's Full screen
        MainWindow.showMaximized()
        # Base path for load model in config file
        self.Base_Path_config = os.path.join(ROOT, r'InitialSetup\config.json')
        # Status of stream it's True for start camera
        self.check_stream_0 = True
        self.check_stream_1 = True
        # Variable for assign status of thread for input value
        self.StopThreadInput = False
        # Capture status
        self.StatusFromIO = False
        # load initial config from json file
        self.LoadInitialConfigStartProgram()
        # Search camera heve connected with PC
        self.SearchCamera()
        # setup dropdown for selected model
        self.SetUpModelSelected()
        # Setup every BUTTON or acion in GUI
        self.SetupAction()
        # Initial model
        self.SelectedModelName()
         
        # Timer thread for start show image and waited for open-close solenoid valve when OK confirm detection
        self.tmtx1 = QtCore.QTimer()
        self.tmtx1.timeout.connect(self.ControlOutputOK)
        self.tmtx1.setInterval(1)

        # Timer thread for start show image and waited for open-close when NG confirm detection
        self.tmtx2 = QtCore.QTimer()
        self.tmtx2.timeout.connect(self.ControlOutputNG)
        self.tmtx2.setInterval(1)

        # timer thred for capture auto
        self.tmtx3 = QtCore.QTimer()
        self.tmtx3.timeout.connect(self.CaptureImageAuto)
        self.tmtx3.setInterval(1)
    
########## Setup action in GUI ##########

    def SetupAction(self):
        # Close Program
        self.actionClose_program.triggered.connect(self.CloseProgram)
        # Change model in combobox
        self.modelName.currentIndexChanged.connect(self.SelectedModelName)
        # Capture button on GUI
        self.capture.clicked.connect(self.CaptureImage)
        # Open folder detection
        self.actionOpen_Folder.triggered.connect(self.OpenFolderImageDetection)
        # Delete all file in detection folder
        self.actionDelete_image.triggered.connect(self.DeleteFolderImageDetection)
        # Check I/O status connection
        self.actionCheck_connected.triggered.connect(self.CheckConnected_IO_Control)
        # open exposure windows
        self.actionExposure.triggered.connect(self.OpenExposureWindow)
        # open model management windows
        self.actionManage_model.triggered.connect(self.OpenModelManagementWindow)
        # open threshold windows
        self.actionthreshold.triggered.connect(self.OpenThresholdWindow)
        # Open delay windows
        self.actionDelay_solenoid.triggered.connect(self.OpenDelayWindow)
        # Open IP Config windows
        self.actionIP_Address.triggered.connect(self.OpenIPConfigWindow)
        # Open setup output channel
        self.actionOutput_Channel_Control.triggered.connect(self.SetUpOutputChannel)
        # Open selected I/O module
        self.actionProduct_I_O.triggered.connect(self.OpenSelectedIOModuleWindow)
        # check box for use auto capture
        self.AutoCapture.clicked.connect(self.LoadStateFromCheckBox)

########## Setup action in GUI ##########

########## initial setup ##########

# Load initial config
    def LoadInitialConfigStartProgram(self):
        # Load initial from json file, LoadInitialConfFromJSON class it's should return [Exposure, Threshold, Delay]
        return_conf = LoadInitialConf.LoadInitialConfFromJSON(path = self.Base_Path_config)
        # Set up camera
        self.ExposureValueCamere1 = return_conf[0][0]
        self.ExposureValueCamere2 = return_conf[0][1]
        # accuracy base for detection
        self._threshold = return_conf[1]
        # Initial timer for start on solenoid
        self.delayOn = return_conf[2]
        # Load IP address
        self.LoadIPAddress()
        # Setup channel for I/O control
        productUsage = SetupIOControl.LoadProductUsed(path = self.Base_Path_config)
        self.ChanneloutputOK = SetupIOControl.LoadOutputUsage(path = self.Base_Path_config, productname = productUsage)
        self.ChanneloutputOK = self.ChanneloutputOK['OK']
        self.ChanneloutputNG = SetupIOControl.LoadOutputUsage(path = self.Base_Path_config, productname = productUsage)
        self.ChanneloutputNG = self.ChanneloutputNG['NG']
        
########## initial setup ##########

########## Capture Image ##########

    ##### Auto or Manual mode for capture #####

    # Load state from check box for selected auto capture ********** [ Auto Mode or Manual Mode ]
    def LoadStateFromCheckBox(self):
        # AutoCapture its' method in Main GUI, when method clicked, it's change status from some value to some value
        if self.AutoCapture.isChecked():
            # Call AleartBoxConfirm for confirm use auto capture mode
            confirm = self.AleartBoxConfirm(description = 'Are you sure to use auto capture mode ?')
            # confirm it's boolean return from AleartBoxConfirm, its' True it's mean confirm to use auto capture mode
            if confirm:
                # Initial input value
                ControlInputSystem.SetValueInitialRising()
                # Disable Capture button in GUI
                self.capture.setEnabled(False)
                # start timer for wait value from I/O Module
                self.tmtx3.start()
            elif not(confirm):
                # Enable capture button in GUI
                self.capture.setEnabled(True)
                # set check box to False
                self.AutoCapture.setChecked(False)
        # not(self.AutoCapture.isChecked()) it's convert from False to True
        elif not(self.AutoCapture.isChecked()):
            # Call AleartBoxConfirm for confirm use auto capture mode
            confirm = self.AleartBoxConfirm(description = 'Are you sure to use manual capture mode ?')
            # confirm it's boolean return from AleartBoxConfirm, its' True it's mean confirm to use auto capture mode
            if confirm:
                # Enable capture button in GUI
                self.capture.setEnabled(True)
                # set check box to False
                self.AutoCapture.setChecked(False)
                # Stop timer for wait value from input channel in I/O Module
                self.tmtx3.stop()
            elif not(confirm):
                # Disable capture buttonn
                self.capture.setEnabled(False)
                # set check box to True
                self.AutoCapture.setChecked(True)

    ##### Auto or Manual mode for capture #####

    # Capture image auto mode with signal, Recieve by I/O Module
    def CaptureImageAuto(self):
         # initial var for save image name
        if self.StatusFromIO:
            try:
                self.FilenameOnceCamera = []
                self.FilenameBothCamera = []
                # Base path model: self.BasePathForModel
                # load model name for set imagename
                try:
                    self.namePart = str(self.modelName.currentText()).split('Address')[0].strip()
                except:
                    print('Error')
                # Base path image predict
                self.BasePathImage = os.path.join(self.BasePathForModel, r'_imagedetection')
                # Capture
                if CameraUsed_Border.CameraUsed == 1:
                    CaptureImg.Capture(path_img = self.BasePathImage, camera = CameraUsed_Border.CameraUsed, partName = self.namePart, sliced_img = [CameraUsed_Border.y_left_top, CameraUsed_Border.y_right_top, CameraUsed_Border.x_left_top, CameraUsed_Border.x_right_top], frame = self.frame_capture0)
                    # print('Filename: {}' .format(CaptureImg.FileNameIMG[0]))
                    self.FilenameOnceCamera = [CaptureImg.FileNameIMG[0]]
                    # Load image before send to detection process
                    self.LoadImageForDetection(imageName = self.FilenameOnceCamera)
                elif CameraUsed_Border.CameraUsed == 2:
                    CaptureImg.Capture(path_img = self.BasePathImage, camera = CameraUsed_Border.CameraUsed, partName = self.namePart, sliced_img = [[CameraUsed_Border.y_left_top, CameraUsed_Border.y_right_top, CameraUsed_Border.x_left_top, CameraUsed_Border.x_right_top], [CameraUsed_Border.y_left_bottom, CameraUsed_Border.y_right_bottom, CameraUsed_Border.x_left_bottom, CameraUsed_Border.x_right_bottom]], frame = [self.frame_capture0, self.frame_capture1])
                    # FileNameIMG[0] is image from top camera, FileNameIMG[1] is image from bottom camera
                    self.FilenameBothCamera = [CaptureImg.FileNameIMG[0], CaptureImg.FileNameIMG[1]]
                    # Load image before send to detection process
                    self.LoadImageForDetection(imageName = self.FilenameBothCamera)
            except:
                self.AleartBoxERROR(description = 'Can\'t capture image, Not found _imagedetection folder')
            # Change value of StatusFromIO from 'True' back to 'False'
            self.StatusFromIO = False
        else:
            pass

    # Capture image in manual mode
    def CaptureImage(self) -> None:
        # initial var for save image name
        try:
            self.FilenameOnceCamera = []
            self.FilenameBothCamera = []
            # Base path model: self.BasePathForModel
            # load model name for set imagename
            try:
                self.namePart = str(self.modelName.currentText()).split('Address')[0].strip()
            except:
                print('Error')
            # Base path image predict
            self.BasePathImage = os.path.join(self.BasePathForModel, r'_imagedetection')
            # Capture
            if CameraUsed_Border.CameraUsed == 1:
                CaptureImg.Capture(path_img = self.BasePathImage, camera = CameraUsed_Border.CameraUsed, partName = self.namePart, sliced_img = [CameraUsed_Border.y_left_top, CameraUsed_Border.y_right_top, CameraUsed_Border.x_left_top, CameraUsed_Border.x_right_top], frame = self.frame_capture0)
                # print('Filename: {}' .format(CaptureImg.FileNameIMG[0]))
                self.FilenameOnceCamera = [CaptureImg.FileNameIMG[0]]
                # Load image before send to detection process
                self.LoadImageForDetection(imageName = self.FilenameOnceCamera)
            elif CameraUsed_Border.CameraUsed == 2:
                CaptureImg.Capture(path_img = self.BasePathImage, camera = CameraUsed_Border.CameraUsed, partName = self.namePart, sliced_img = [[CameraUsed_Border.y_left_top, CameraUsed_Border.y_right_top, CameraUsed_Border.x_left_top, CameraUsed_Border.x_right_top], [CameraUsed_Border.y_left_bottom, CameraUsed_Border.y_right_bottom, CameraUsed_Border.x_left_bottom, CameraUsed_Border.x_right_bottom]], frame = [self.frame_capture0, self.frame_capture1])
                # FileNameIMG[0] is image from top camera, FileNameIMG[1] is image from bottom camera
                self.FilenameBothCamera = [CaptureImg.FileNameIMG[0], CaptureImg.FileNameIMG[1]]
                # Load image before send to detection process
                self.LoadImageForDetection(imageName = self.FilenameBothCamera)
        except:
            self.AleartBoxERROR(description = 'Can\'t capture image, Not found _imagedetection folder')

########## Capture Image ##########

########## Load model name for set model used for detection ##########

    def LoadModelNameForDetection(self):
        try:
            _path = os.path.join(self.BasePathForModel, r'LoadModel\config.json')
            LoadModelName.LoadModelJ_File(path = _path, mainPath = os.path.join(self.BasePathForModel, r'LoadModel'))
            if LoadModelName.ERROR:
                self.AleartBoxERROR(description = 'Not found config.json in folder')
            else:
                # Model used
                self.ModelUsed = LoadModelName.modelUsed
                self.ModelNotFound = LoadModelName.modelNotFound
                if self.ModelNotFound == []:
                    print('LOAD MODEL COMPLETE!')
                else:
                    for i in self.ModelNotFound:
                        self.AleartBoxERROR(description = 'Not found model: {}' .format(i))
        except:
            pass

########## Reset Result ##########
    
    # Reset result image
    def ResetResultDefinition(self):
        # Reset result image
        self.result_cam0.clear()
        self.result_cam1.clear()
        # Reset result text
        self.SumResult0.clear()
        self.SumResult1.clear()

########## Reset Result ##########

########## Camera and Stream ##########

    # Config camera border, Camera used
    def LoadConfigCamera(self):
        try:
            path = os.path.join(self.BasePathForModel, r'LoadModel\config.json')
            CameraUsed_Border.LoadConfig(path_config = path)
        except:
            self.AleartBoxERROR(description = 'Not found config.json in folder')

    # Search camera
    def SearchCamera(self):
        self.DevListCamera = sdk.CameraEnumerateDevice()
        self.DevListFound = len(self.DevListCamera)
        if self.DevListFound == 0:
            print('No camera connected')
        else:
            print('Found %d camera(s)' % self.DevListFound)
        if self.DevListFound != 2:
            self.AleartBox_FoundCamera()
        # Call Initialize camera
        self.InitCamera0()
        self.InitCamera1()

    # Initialize camera
    def InitCamera0(self):
        self.h_camera0 = 0
        while True:
            try:
                self.h_camera0 = sdk.CameraInit(self.DevListCamera[0])
                print('Camera 0 initialized')
                break
            except:
                print(f'Camera 0 Init failed ({sdk.CameraException.error_code}) : {sdk.CameraException.message}')
                continue
        self.SetUpCamera0()

    # Setup camera0
    def SetUpCamera0(self):
        self.cap_0 = sdk.CameraGetCapability(self.h_camera0)
        self.monoCamera_0 = self.cap_0.sIspCapacity.bMonoSensor != 0
        if self.monoCamera_0:
            sdk.CameraSetIspOutFormat(self.h_camera0, sdk.CAMERA_MEDIA_TYPE_MONO8)
        else:
            sdk.CameraSetIspOutFormat(self.h_camera0, sdk.CAMERA_MEDIA_TYPE_BGR8)
        sdk.CameraSetTriggerMode(self.h_camera0, 0)
        sdk.CameraSetAeState(self.h_camera0, 0)
        sdk.CameraSetExposureTime(self.h_camera0, self.ExposureValueCamere1 * 1000)
        sdk.CameraPlay(self.h_camera0)
        self.FrameBufferSize0 = self.cap_0.sResolutionRange.iWidthMax * self.cap_0.sResolutionRange.iHeightMax * (1 if self.monoCamera_0 else 3)
        self.pFrame0 = sdk.CameraAlignMalloc(self.FrameBufferSize0, 16)
        print('Camera 0: Setup Complete!')

    # Load Stream camera 0
    def LoadStream_Camera0(self) -> None:
        while True:
            try:
                # if self.check_stream it's True, then start stream with Threading
                if self.check_stream_0:
                    self.pRawData0, self.FrameHead0 = sdk.CameraGetImageBuffer(self.h_camera0, 200)
                    sdk.CameraImageProcess(self.h_camera0, self.pRawData0, self.pFrame0, self.FrameHead0)
                    sdk.CameraReleaseImageBuffer(self.h_camera0, self.pRawData0)
                    
                    if pf.system() == 'windows':
                        sdk.CameraFlipFrameBuffer(self.pFrame0, self.FrameHead0, 1)
                        
                    self.frame_data0 = (sdk.c_ubyte * self.FrameHead0.uBytes).from_address(self.pFrame0)
                    self.frame0 = np.frombuffer(self.frame_data0, dtype = np.uint8)
                    self.frame0 = self.frame0.reshape((self.FrameHead0.iHeight, self.FrameHead0.iWidth, 1 if self.FrameHead0.uiMediaType == sdk.CAMERA_MEDIA_TYPE_MONO8 else 3))
                    self.height0, self.width0, self.channel0 = self.frame0.shape
                    # # define frame_capture0 for use in detection process
                    # -1 is flip vertical and horizontal
                    self.frame_capture0 = cv2.flip(self.frame0, 1)
                    # Flip vertical
                    # self.frame0 = cv2.flip(self.frame0, 0)
                    # Flip horizontal
                    self.frame0 = cv2.flip(self.frame0, 1)
                    try:
                        self.frame0 = cv2.rectangle(self.frame0, (CameraUsed_Border.x_left_top, CameraUsed_Border.y_left_top), (CameraUsed_Border.x_right_top, CameraUsed_Border.y_right_top), (0, 0, 255), 3)
                    except:
                        pass
                    # Resized video stream
                    self.frame0 = cv2.resize(self.frame0, (int(self.width0 * 22 / 100), int(self.height0 * 22 / 100)), interpolation = cv2.INTER_AREA)
                    # change color of image
                    self.frame0 = cv2.cvtColor(self.frame0, cv2.COLOR_BGR2RGB)
                    self.height0, self.width0, self.channel0 = self.frame0.shape
                    self.step0 = self.channel0 * self.width0
                    self.qImg0 = QImage(self.frame0.data, self.width0, self.height0, self.step0, QImage.Format_RGB888)
                    # show image in img_label
                    pixmap = QPixmap.fromImage(self.qImg0)
                    self.camera_0.setPixmap(pixmap)
                else:
                    break
            except sdk.CameraException as e:
                if sdk.CameraException != sdk.CAMERA_STATUS_TIME_OUT:
                    if e.error_code == -5:
                        break
                    else:
                        continue
            tm.sleep(0.05)

    # Initialize camera
    def InitCamera1(self):
        self.h_camera1 = 0
        while True:
            try:
                self.h_camera1 = sdk.CameraInit(self.DevListCamera[1])
                print('Camera 1 initialized')
                break
            except:
                print(f'Camera 1 Init failed ({sdk.CameraException.error_code}) : {sdk.CameraException.message}')
                continue
        self.SetUpCamera1()
        
    # Setup camera1
    def SetUpCamera1(self):
        self.cap_1 = sdk.CameraGetCapability(self.h_camera1)
        self.monoCamera_1 = self.cap_1.sIspCapacity.bMonoSensor != 0
        if self.monoCamera_1:
            sdk.CameraSetIspOutFormat(self.h_camera1, sdk.CAMERA_MEDIA_TYPE_MONO8)
        else:
            sdk.CameraSetIspOutFormat(self.h_camera1, sdk.CAMERA_MEDIA_TYPE_BGR8)
        sdk.CameraSetTriggerMode(self.h_camera1, 0)
        sdk.CameraSetAeState(self.h_camera1, 0)
        sdk.CameraSetExposureTime(self.h_camera1, self.ExposureValueCamere2 * 1000)
        sdk.CameraPlay(self.h_camera1)
        self.FrameBufferSize1 = self.cap_1.sResolutionRange.iWidthMax * self.cap_1.sResolutionRange.iHeightMax * (1 if self.monoCamera_1 else 3)
        self.pFrame1 = sdk.CameraAlignMalloc(self.FrameBufferSize1, 16)
        print('Camera 1: Setup Complete!')

    # Load Stream camera 1
    def LoadStream_Camera1(self) -> None:
        while True:
            try:
                # if self.check_stream it's True, then start stream with Threading
                if self.check_stream_1 and CameraUsed_Border.CameraUsed == 2:
                    self.pRawData1, self.FrameHead1 = sdk.CameraGetImageBuffer(self.h_camera1, 200)
                    sdk.CameraImageProcess(self.h_camera1, self.pRawData1, self.pFrame1, self.FrameHead1)
                    sdk.CameraReleaseImageBuffer(self.h_camera1, self.pRawData1)
                    
                    if pf.system() == 'windows':
                        sdk.CameraFlipFrameBuffer(self.pFrame1, self.FrameHead1, 1)
                        
                    self.frame_data1 = (sdk.c_ubyte * self.FrameHead1.uBytes).from_address(self.pFrame1)
                    self.frame1 = np.frombuffer(self.frame_data1, dtype = np.uint8)
                    self.frame1 = self.frame1.reshape((self.FrameHead1.iHeight, self.FrameHead1.iWidth, 1 if self.FrameHead1.uiMediaType == sdk.CAMERA_MEDIA_TYPE_MONO8 else 3))
                    # # define frame_capture1 for use in detection process
                    self.frame_capture1 = cv2.flip(self.frame1, 1)
                    # self.frame_capture1 = self.frame1
                    self.height1, self.width1, self.channel1 = self.frame1.shape
                    # Flip vertical
                    self.frame1 = cv2.flip(self.frame1, 1)
                    # Bounding square box for capture
                    try:
                        self.frame1 = cv2.rectangle(self.frame1, (CameraUsed_Border.x_left_bottom, CameraUsed_Border.y_left_bottom), (CameraUsed_Border.x_right_bottom, CameraUsed_Border.y_right_bottom), (0, 0, 255), 3)
                    except:
                        pass
                    # Resized video stream
                    self.frame1 = cv2.resize(self.frame1, (int(self.width1 * 22 / 100), int(self.height1 * 22 / 100)), interpolation = cv2.INTER_AREA)
                    # change color of image
                    self.frame1 = cv2.cvtColor(self.frame1, cv2.COLOR_BGR2RGB)
                    self.height1, self.width1, self.channel1 = self.frame1.shape
                    self.step1 = self.channel1 * self.width1
                    self.qImg1 = QImage(self.frame1.data, self.width1, self.height1, self.step1, QImage.Format_RGB888)
                    # show image in img_label
                    self.camera_1.setPixmap(QPixmap.fromImage(self.qImg1))
                # if camera used in model less than 2 and check stream it's True 
                elif CameraUsed_Border.CameraUsed < 2 and self.check_stream_1:
                    # Clear old image 
                    self.camera_1.clear()
                    # Set background color to black
                    self.camera_1.setStyleSheet('background-color : rgb(0, 0, 0);')
                # if change self.check_stream from True to False, Break thread
                elif not(self.check_stream_1):
                    break
            except sdk.CameraException as e:
                if sdk.CameraException != sdk.CAMERA_STATUS_TIME_OUT:
                    if e.error_code == -5:
                        break
                    else:
                        continue
            tm.sleep(0.05)

    # Uninit camera
    def UnInitCamera(self) -> None:
        try:
            sdk.CameraUnInit(self.h_camera0)
            sdk.CameraUnInit(self.h_camera1)
            sdk.CameraAlignFree(self.pFrame0)
            sdk.CameraAlignFree(self.pFrame1)
        except:
            pass

########## Camera and Stream ##########

########## Combobox selected model with combobox in GUI ##########

    # Set up model selected
    def SetUpModelSelected(self) -> None:
        # Model variable
        self.Model_In_Path = []
        datas = SetupPathModel.LoadModel()
        # for i in range(len(datas['PartNumber'])):
        for i in datas['PartNumber']:
            # print(datas['PartNumber'][str(i)]['Name'])
            self.Model_In_Path.append('{}\t Address: {}' .format(datas['PartNumber'][str(i)]['Name'], datas['PartNumber'][str(i)]['Address']))
        # add model to combobox
        self.AddModelComboBox()

    # Add model to combobox
    def AddModelComboBox(self) -> None:
        # Clear combobox
        self.modelName.clear()
        # Add model in combobox
        self.modelName.addItems(self.Model_In_Path)

    # selected model named
    def SelectedModelName(self):
        # Reset old definition
        self.ResetResultDefinition()
        try:
            self.ModelSelected = str(self.modelName.currentText()).split('Address')[0].strip()
            with open(self.Base_Path_config) as f:
                datas = json.load(f)
            self.BasePathForModel = datas['PartNumber'][str(self.ModelSelected)]['Path']
            # Load config camera
            self.LoadConfigCamera()
            # # Load model name that selected
            self.LoadModelNameForDetection()
        except KeyError:
            pass

########## Combobox ##########

########## Load image for detection ##########

    # Load image for detection
    def LoadImageForDetection(self, imageName):
        filenameImage = []
        if len(imageName) == 1:
            filenameImage.append(imageName[0])

        elif len(imageName) == 2:
            filenameImage.append(imageName[0])
            filenameImage.append(imageName[1])
        self.DetectionProcess(sourceFile = filenameImage)

########## Load image for detection ##########

########## Detection process ##########

    # Detection process
    def DetectionProcess(self, sourceFile):
        try:
            # Path for load file list
            path = os.path.join(self.BasePathForModel, r'LoadModel')
            # if used once camera
            if CameraUsed_Border.CameraUsed == 1:
                if len(self.ModelUsed) == CameraUsed_Border.CameraUsed:
                    # Reset result image
                    self.result_cam1.clear()
                    # Reset text result
                    self.SumResult0.clear()
                    # modelName[0] is first model and used for single camera
                    # source file and path_save it's same
                    detect.run(weights = os.path.join(path, self.ModelUsed[0]), source = sourceFile[0], device = 'cpu', conf_thres = self._threshold, iou_thres = 0.6, path_save = sourceFile[0], path_compare = os.path.join(self.BasePathForModel, r'LoadModel\match_detection.json'), camera = 'CAMERA1')
                    # path for load image after detection
                    # index_result = o it's result for camera 0 ( Top camera )
                    _return = self.LoadResult(pathLoad = [sourceFile[0]], index_result = 0)
                    # Compare answer and definition
                    answer_return = CompareAns.LoadCompareAns(path_compare = os.path.join(self.BasePathForModel, r'LoadModel\match_detection.json'), ansfromDetection = detect.Answer, Key = 'CAMERA1', camera = CameraUsed_Border.CameraUsed)
                    # Okcode from compare
                    _OKCode = answer_return[1]
                    # NGCode from compare
                    _NGCode = answer_return[2]
                    # OK Confirm status
                    OKConFirm = answer_return[3]
                    # Add result to gui
                    if answer_return[0] != []:
                        # Append definition to line edit
                        self.AppendResult(definition = answer_return[0], OKCode = _OKCode, NGCode = _NGCode, Index_Result = 0)
                    else:
                        # Append definition to line edit
                        self.AppendResult(definition = answer_return[0], OKCode = _OKCode, NGCode = _NGCode, Index_Result = 0)
                    # If not found object, delete image
                    if answer_return[0] == []:
                        # call function for control output with NG setting
                        self.tmtx2.start()
                        self.JustmentResult(status = False)
                        self.DeleteImageIfNotFoundObj(FileName = sourceFile[0])
                    elif answer_return[0] != []: 
                        # If OKConfirm = True, it's mean OK Code
                        if OKConFirm:
                            self.JustmentResult(status = True)
                            # _return it's state return when setpixmap success   
                            if _return and OKConFirm:
                                self.tmtx1.start()                                
                        elif not(OKConFirm) and _return:
                            # call function for control output with NG setting
                            self.tmtx2.start()
                            self.JustmentResult(status = False)                    
                else:
                    # Model isn't match with setting
                    self.AleartBoxERROR(description = 'Model is not match')
                    # remove image file
                    for i in sourceFile:
                        os.remove(i)

            # if used both camera
            elif CameraUsed_Border.CameraUsed == 2:
                # Defind for add answer from compare
                Ans_Compare = {}
                # Detection with image from top camera
                # self.ModelUsed[0] for Top camera, self.ModelUsed[1] for Bottom camera
                if len(self.ModelUsed) == CameraUsed_Border.CameraUsed:
                    ##### Top camera Process
                    detect.run(weights = os.path.join(path, self.ModelUsed[0]), source = sourceFile[0], device = 'cpu', conf_thres = self._threshold, iou_thres = 0.6, path_save = sourceFile[0], path_compare = os.path.join(self.BasePathForModel, r'LoadModel\match_detection.json'), camera = 'CAMERA1')
                    _return = self.LoadResult(pathLoad = [sourceFile[0], sourceFile[1]], index_result = 0)
                    # Compare answer and definition
                    # Compare camera 1
                    answer_return = CompareAns.LoadCompareAns(path_compare = os.path.join(self.BasePathForModel, r'LoadModel\match_detection.json'), ansfromDetection = detect.Answer, Key = 'CAMERA1', camera = CameraUsed_Border.CameraUsed)
                    # if OK and NGCode = 0, answer_return[3] it's 'True'
                    Ans_Compare['Camera1'] = answer_return
                    # Okcode from compare
                    _OKCode = answer_return[1]
                    # NGCode from compare
                    _NGCode = answer_return[2]
                    # OK CheConfirm status
                    OKConFirm = answer_return[3]
                    # Reset text result
                    self.SumResult0.clear()
                    if answer_return[0] != []:
                        self.AppendResult(definition = answer_return[0], OKCode = _OKCode, NGCode =  _NGCode, Index_Result = 0)
                    else:
                        # Append definition to line edit
                        self.AppendResult(definition = answer_return[0], OKCode = _OKCode, NGCode =  _NGCode, Index_Result = 0)
                    # Delete file, if's not found object, and alarm NG code
                    if answer_return[0] == [] and not(OKConFirm):
                        self.tmtx2.start()  
                        self.JustmentResult(status = False)
                        self.DeleteImageIfNotFoundObj(FileName = sourceFile[0])
                    
                    ##### Bottom camera Process
                    # Detection with image from bottom camera
                    detect.run(weights = os.path.join(path, self.ModelUsed[1]), source = sourceFile[1], device = 'cpu', conf_thres = self._threshold, iou_thres = 0.6, path_save = sourceFile[1], path_compare = os.path.join(self.BasePathForModel, r'LoadModel\match_detection.json'), camera = 'CAMERA2')
                    # Load result after detection
                    _return = self.LoadResult(pathLoad = [sourceFile[0], sourceFile[1]], index_result = 1)
                    # Compare camera 2
                    answer_return = CompareAns.LoadCompareAns(path_compare = os.path.join(self.BasePathForModel, r'LoadModel\match_detection.json'), ansfromDetection = detect.Answer, Key = 'CAMERA2', camera = CameraUsed_Border.CameraUsed)
                    Ans_Compare['Camera2'] = answer_return
                    # Okcode from compare
                    _OKCode = answer_return[1]
                    # NGCode from compare
                    _NGCode = answer_return[2]
                    # OK Confirm status
                    OKConFirm = answer_return[3]
                    # Reset text result
                    self.SumResult1.clear()
                    if answer_return[0] != []:
                        self.AppendResult(definition = answer_return[0], OKCode = _OKCode, NGCode = _NGCode, Index_Result = 1)
                    else:
                        # Append definition to line edit
                        self.AppendResult(definition = answer_return[0], OKCode = _OKCode, NGCode = _NGCode, Index_Result = 1)
                    # Delete file, if's not found object
                    if answer_return[0] == [] and not(OKConFirm):
                        self.JustmentResult(status = False)
                        self.DeleteImageIfNotFoundObj(FileName = sourceFile[1])
                    # Call method for check answer with both camera from detection
                    _OkConmfirmReturn = self.CompareBothAnswerFromDetection(ans = Ans_Compare)   
                    # _return it's state return when setpixmap success    
                    if _return and _OkConmfirmReturn:
                        self.tmtx1.start() 
                    elif not(_OkConmfirmReturn) and _return:
                        self.tmtx2.start()             
                else:
                    # Model isn't match with setting
                    self.AleartBoxERROR(description = 'Model is not match')
                    # remove image file
                    for i in sourceFile:
                        os.remove(i)
        except:
            pass
            
    # Compare both answer
    def CompareBothAnswerFromDetection(self, ans):
        count_OKState = 0
        for keys in ans:
            if ans[keys][3] == True:
                count_OKState += 1
        # if count_OKState = quality of camera, it's mean OK
        if count_OKState == CameraUsed_Border.CameraUsed:
            self.JustmentResult(status = True)
            return True
        else:
            self.JustmentResult(status = False)
            return False
        
########## Detection process ##########

########## Load Result ##########

    # Load result
    def LoadResult(self, pathLoad, index_result):
        # result top cam
        if index_result == 0:
            try:
                Img = cv2.imread(str(pathLoad[0]))
                Img = cv2.cvtColor(Img, cv2.COLOR_BGR2RGB)
                height, width, channel = Img.shape
                Img = cv2.resize(Img, (int(width * 34 / 100), int(height * 34 / 100)), interpolation = cv2.INTER_AREA)
                height, width, channel = Img.shape
                step = width * channel
                qImg = QImage(Img.data, width, height, step, QImage.Format_RGB888)
                self.result_cam0.setPixmap(QPixmap.fromImage(qImg))
            except:
                print('Error load result')
        # result bottom cam
        elif index_result == 1:
            try:
                Img = cv2.imread(str(pathLoad[1]))
                Img = cv2.cvtColor(Img, cv2.COLOR_BGR2RGB)
                height, width, channel = Img.shape
                Img = cv2.resize(Img, (int(width * 50 / 100), int(height * 50 / 100)), interpolation = cv2.INTER_AREA)
                height, width, channel = Img.shape
                step = width * channel
                qImg = QImage(Img.data, width, height, step, QImage.Format_RGB888)
                self.result_cam1.setPixmap(QPixmap.fromImage(qImg))
            except:
                print('Error load result')
        return True

    # Append definition to GUI
    def AppendResult(self, definition, OKCode, NGCode, Index_Result):
        # Append result to GUI
        msg = ''
        # print('Definition: {}' .format(definition))
        if definition != []:
            for i in definition:
                if not(str(i).startswith('OK')) and NGCode != 0:
                    msg += i + '\n'
                if str(i).startswith('OK') and NGCode == 0:
                    msg += i + '\n'
            if (Index_Result == 0 or Index_Result == 1) and NGCode != 0:
                self.ResultSet(msg = msg, color_text = 'rgb(255, 0, 0);', index_result = Index_Result)
            elif (Index_Result == 0 or Index_Result == 1) and NGCode == 0:
                self.ResultSet(msg = msg, color_text = 'rgb(4, 119, 17);', index_result = Index_Result)
        else:
            msg += 'Wrong part or not found object'
            self.ResultSet(msg = msg, color_text = 'rgb(255, 0, 0);', index_result = Index_Result)

    # Set result description
    def ResultSet(self, msg, color_text, index_result):
        if index_result == 0:
            self.SumResult0.setStyleSheet(f'color: {color_text}')
            self.SumResult0.append(msg)
        elif index_result == 1:
            self.SumResult1.setStyleSheet(f'color: {color_text}')
            self.SumResult1.append(msg)
        
########## Load Result ##########

########## Justment result ###########

    # Justment result
    def JustmentResult(self, status):
        self.andcheck_result.clear()
        if status:
            self.andcheck_result.setStyleSheet('background-color : rgb(4, 119, 17);'
                                                'color: rgb(255, 255, 255);')
            self.andcheck_result.setText('OK')
        else:
            self.andcheck_result.setStyleSheet('background-color : rgb(255, 0, 0);'
                                                'color: rgb(255, 255, 255);')
            self.andcheck_result.setText('NG')

########## Justment result ###########

########### Aleart box ##########

    # Alert box found camera
    def AleartBox_FoundCamera(self) -> None:
        msg = QMessageBox()
        msg.setIcon(QMessageBox.Warning)
        msg.setText('Found %d camera(s)' % self.DevListFound)
        msg.setInformativeText('Please connect 2 camera')
        msg.setWindowTitle('Warning')
        exit = msg.exec_()
        if exit:
            sys.exit()

    # Aleartbox Success 
    def AleartBoxSuccess(self, description):
        msg = QMessageBox()
        msg.setWindowTitle('Success')
        msg.setWindowIcon(QtGui.QIcon(str(os.path.join(ROOT, r'imagefile\information.png'))))
        msg.setText(description)
        msg.setIcon(QMessageBox.Information)
        msg.setStandardButtons(QMessageBox.Ok)
        msg.exec_()

    # Aleartbox Error
    def AleartBoxERROR(self, description):
        msg = QMessageBox()
        msg.setWindowTitle('Error')
        msg.setText(description)
        msg.setIcon(QMessageBox.Warning)
        msg.setStandardButtons(QMessageBox.Ok)
        msg.exec_()

    # Aleartbox for confirm
    def AleartBoxConfirm(self, description):
        msg = QMessageBox()
        msg.setWindowTitle('Confirm')
        msg.setWindowIcon(QtGui.QIcon(str(os.path.join(ROOT, r'imagefile\question.png'))))
        msg.setText(description)
        msg.setIcon(QMessageBox.Question)
        msg.setStandardButtons(QMessageBox.Yes | QMessageBox.No)
        # ret is signal from clicked button
        ret = msg.exec_()
        if ret == QMessageBox.Yes:
            return True
        elif ret == QMessageBox.No:
            return False

########### Aleart box ##########

########## Folder process ##########

    # Open folder image detection
    def OpenFolderImageDetection(self) -> None:
        try:
            os.startfile(os.path.join(self.BasePathForModel, r'_imagedetection'))
        except:
            print('Error open folder')

    # Delete image deteciton
    def DeleteFolderImageDetection(self):
        try:
            for i in os.listdir(os.path.join(self.BasePathForModel, r'_imagedetection')):
                file = os.path.join((os.path.join(self.BasePathForModel, r'_imagedetection')), i)
                os.remove(file)
            # when delete successs
            self.AleartBoxSuccess(description = 'Delete Image success')
        except: 
            self.AleartBoxERROR(description = 'Error delete image')
        
    # Delete image if not found object
    def DeleteImageIfNotFoundObj(self, FileName = None):
        try:
            os.remove(FileName)
        except:
            print('Error delete image')

########## Folder process ##########

########## Exposure windows ##########

    def OpenExposureWindow(self):
        self.widget_exposure = QtWidgets.QMainWindow()
        # self.ui is exposure windows
        self.ui = UI.gui_exposure.Ui_MainWindow()
        self.ui.setupUi(self.widget_exposure)
        self.widget_exposure.setWindowTitle('Exposure setting')
        self.widget_exposure.setWindowIcon(QtGui.QIcon(str(os.path.join(ROOT, r'imagefile\exposure.png'))))
        self.widget_exposure.setFixedWidth(359)
        self.widget_exposure.setFixedHeight(166)
        # button exposure windows setup
        self.buttonExposureWindows()
        # Initial setup exposure value
        self.ui.exposure_cam1.setValue(self.ExposureValueCamere1)
        self.ui.exposure_cam2.setValue(self.ExposureValueCamere2)
        # Show exposure windows
        self.widget_exposure.show()

    # button in exposure windows
    def buttonExposureWindows(self):
        self.ui.exposure_cam1.sliderReleased.connect(self.loadExposureCamera1)
        self.ui.exposure_cam2.sliderReleased.connect(self.loadExposureCamera2)
        self.ui.okbutton.clicked.connect(self.SetUpExposureValue)
    
    # load exposure camera 1, and set new exposure value
    def loadExposureCamera1(self):
        self.ExposureValueCamere1 = int(self.ui.exposure_cam1.value())
        try:
            sdk.CameraSetExposureTime(self.h_camera0, int(self.ui.exposure_cam1.value()) * 1000)
        except:
            self.AleartBoxERROR(description = 'Can\'t set up camera 1 exposure value !')
            
    # load exposure camera 2, and set new exposure value
    def loadExposureCamera2(self):
        self.ExposureValueCamere2 = int(self.ui.exposure_cam2.value())
        try:
            sdk.CameraSetExposureTime(self.h_camera1, int(self.ui.exposure_cam2.value()) * 1000)
        except:
            self.AleartBoxERROR(description = 'Can\'t set up camera 2 exposure value !')

    # Set up exposure value
    def SetUpExposureValue(self):
        try:            
            self.AleartBoxSuccess(description = 'Exposure value\nCamera 1: {}\nCamera 2: {}' .format(self.ExposureValueCamere1, self.ExposureValueCamere2))
            # Update exposure value in configExposure_thre_delay.json
            UpdateConfig.UpdateConfigExposure(path = self.Base_Path_config, exposure = [self.ExposureValueCamere1, self.ExposureValueCamere2])
            self.widget_exposure.close()
        except:
            self.AleartBoxERROR(description = 'Can\'t set up exposure value !')

########## Exposure windows ##########

########## Model management windows ##########

    # Open model management windows
    def OpenModelManagementWindow(self):
        self.widget_model = QtWidgets.QMainWindow()
        self.ui_model = UI.gui_modelmanage.Ui_MainWindow()
        self.ui_model.setupUi(self.widget_model)
        self.widget_model.setWindowTitle('Model management')
        # Set Windows icon
        try:
            self.widget_model.setWindowIcon(QtGui.QIcon(str(os.path.join(ROOT, r'imagefile\listmodel.png'))))
        except:
            pass
        self.widget_model.setFixedWidth(322)
        self.widget_model.setFixedHeight(384)
        # Load model name
        self.LoadModelInConfigFile()
        # Setup button
        self.buttonAddModelDeleteModel()
        self.widget_model.show()

    # Button in model management windows
    def buttonAddModelDeleteModel(self):
        # Add model button
        self.ui_model.AddModel_Button.clicked.connect(self.AddModelPath)
        # when click model items in listview, it's connected to SelectedModelInListView for change model selected
        self.ui_model.ListModel.itemSelectionChanged.connect(self.SelectedModelInListView)
        # Delete model button
        self.ui_model.DeleteModel_Buttom.clicked.connect(self.DeleteModelPath)
        # Change bit address per model
        self.ui_model.SetAddress.clicked.connect(self.SettingBITAddress)
    
    # Load model name in config file
    def LoadModelInConfigFile(self):
        # Clear list model
        self.ui_model.ListModel.clear()
        # datas in file config.json
        datas = SetupPathModel.LoadModel()
        # Used loop for initial model name
        for i in datas['PartNumber']:
            # Add model items in config.json in InitialSetup
            i += '\tAddress: {}' .format(datas['PartNumber'][i]['Address'])
            self.ui_model.ListModel.addItem(i)

    # View model
    def SelectedModelInListView(self):
        try:
            # selected model in list view
            self._modelSelectedInListView = str(self.ui_model.ListModel.currentItem().text()).split('Address')[0].strip()
        except:
            print('Error')

    # Add model path
    def AddModelPath(self) -> None:
        # Get path model
        path = QFileDialog.getExistingDirectory(caption = 'Select path for model')
        if path != '':
            keys = str(path).split(r'/')[-1]
            paths = str(path).replace('/', '\\')
            SetupPathModel.AddPathModel(keys = keys, path = paths)
            # Load model after add model to config
            self.SetUpModelSelected()
            # Reload model in list view
            self.LoadModelInConfigFile()
            self.AleartBoxSuccess(description = 'Update Model Success !')

    # Delete model path
    def DeleteModelPath(self):
        # show aleartbox for confirm to delete model
        ok = self.AleartBoxConfirm(description = 'Are you sure to delete model: {} ?' .format(self._modelSelectedInListView))
        if ok:
            # response is values that's return from write.DeleteModelPathInConfig if response is True, it's mean can't delete model in config
            response = SetupPathModel.DeleteModelPathInConfig(keys = self._modelSelectedInListView)
            if response:
                self.AleartBoxERROR(description = 'UPDATE Model ERROR !')
            else:
                self.AleartBoxSuccess(description = 'DELETE Model Success !')
                # Reload model in config file
                self.SetUpModelSelected()
                # Reload model in list model
                self.LoadModelInConfigFile()

    # Setting Bit Address of model
    def SettingBITAddress(self):
        # Load bit address
        bit_address = SetupPathModel.LoadBitAddress(modelname = self._modelSelectedInListView)
        try:
            text, ok = QInputDialog.getText(MainWindow, 'Address', f'Address: {bit_address}\nInput new Address')
            if ok:
                if len(text) == 5:
                    # Update bit address of model
                    confirm = SetupPathModel.UpdateBitAddress(modelname = self._modelSelectedInListView, bitaddress = text)
                    if confirm:
                        self.AleartBoxSuccess(description = 'Update BIT Address Success !')
                        self.SetUpModelSelected()
                    else:
                        self.AleartBoxERROR(description = 'Bit address is already !')
                        self.SettingBITAddress()
                else:
                    self.AleartBoxERROR(description = 'Address must be 5 bit !')
                    self.SettingBITAddress()
        except:
            self.AleartBoxERROR(description = 'Can\'t set up bit address !')
                
########## Model management windows ##########

########## Threshold windows ##########

    # Open threshold windows
    def OpenThresholdWindow(self):
        self.widget_threshold = QtWidgets.QMainWindow()
        self.ui_threshold = UI.gui_threshold.Ui_MainWindow()
        self.ui_threshold.setupUi(self.widget_threshold)
        self.widget_threshold.setWindowTitle('Threshold')
        try:
            self.widget_threshold.setWindowIcon(QtGui.QIcon(os.path.join(ROOT, r'imagefile\threshold.png')))
        except:
            pass
        self.widget_threshold.setFixedHeight(140)
        self.widget_threshold.setFixedWidth(252)
        # Load initial setup
        self.LoadInitialThresholdValue()
        # Setup other button on GUI
        self.widget_threshold_setupButton()
        self.widget_threshold.show()

    # load initial threshold value
    def LoadInitialThresholdValue(self):
        # Set old value to text label
        self.ui_threshold.OldThreshold.setText(str(self._threshold))

    # Setup button on GUI
    def widget_threshold_setupButton(self):
        # OK for confirm setting
        self.ui_threshold.OKBT.clicked.connect(self.confirmThresholdValue)

    def confirmThresholdValue(self):
        self._threshold = self.ui_threshold.ThresholdValue.text()
        # threshold should be numeric
        if self._threshold.isnumeric() and self._threshold != '':
            self._threshold = int(self._threshold)
            if self._threshold >= 0 and self._threshold <= 100:
                # Update threshold value in initial setting
                UpdateConfig.UpdateConfigThreshold(path = self.Base_Path_config, threshold = self._threshold / 100)
                self.AleartBoxSuccess(description = 'Update Threshold Success !')
                self.widget_threshold.close()
            else:
                self.AleartBoxERROR(description = 'Threshold must be between 0 and 100 !')
                # Close windows, then open again
                self.widget_threshold.close()
                self.OpenThresholdWindow()
        else:
            self.AleartBoxERROR(description = 'Threshold must be numeric !')

########## Threshold windows ##########

########## delay windows ##########

    # Open delay windows
    def OpenDelayWindow(self):
        self.widget_delay = QtWidgets.QMainWindow()
        self.ui_delay = UI.gui_delay.Ui_MainWindow()
        self.ui_delay.setupUi(self.widget_delay)
        self.widget_delay.setWindowTitle('Delay')
        try:
            self.widget_delay.setWindowIcon(QtGui.QIcon(os.path.join(ROOT, r'imagefile\delay.png')))
        except:
            pass
        self.widget_delay.setFixedHeight(140)
        self.widget_delay.setFixedWidth(252)
        # Load initial setup
        self.LoadInitialDelayValue()
        # Setup other button on GUI
        self.widget_delay_setupButton()
        self.widget_delay.show()

    # load initial delay value
    def LoadInitialDelayValue(self):
        # Set old value to text label
        self.ui_delay.OldDelay.setText(str(self.delayOn))

    # Setup button on GUI
    def widget_delay_setupButton(self):
        # OK for confirm setting
        self.ui_delay.OKBT.clicked.connect(self.confirmDelayValue)

    # Confirm delay value
    def confirmDelayValue(self):
        self.delayOn = self.ui_delay.DelayValue.text()
        # delay should be numeric
        if self.delayOn.isnumeric() and self.delayOn != '':
            self.delayOn = int(self.delayOn)
            if self.delayOn >= 1 and self.delayOn <= 10:
                # Update delay value in initial setting
                UpdateConfig.UpdateConfigDelay(path = self.Base_Path_config, delay = self.delayOn)
                self.AleartBoxSuccess(description = 'Update Delay Success !')
                self.widget_delay.close()
            else:
                self.AleartBoxERROR(description = 'Delay must be between 1 and 10 !')
                # Close windows, then open again
                self.widget_delay.close()
                self.OpenDelayWindow()

########## delay windows ##########

########## IP I/O Config ##########

    # Open IP I/O Config windows
    def OpenIPConfigWindow(self):
        self.widget_ipio = QtWidgets.QMainWindow()
        self.ui_ipio = UI.gui_ipconfig.Ui_MainWindow()
        self.ui_ipio.setupUi(self.widget_ipio)
        self.widget_ipio.setWindowTitle('IP I/O Config')
        try:
            self.widget_ipio.setWindowIcon(QtGui.QIcon(os.path.join(ROOT, r'imagefile\ipaddress.png')))
        except:
            pass
        self.widget_ipio.setFixedHeight(136)
        self.widget_ipio.setFixedWidth(253)
        # Load old control in config file
        self.LoadInitialIOCOntrol()
        # set up OK button on IP config windows
        self.SetupButtonOnIPConfigWindow()
        self.widget_ipio.show()

    # Load initial control in config file
    def LoadInitialIOCOntrol(self):
        try:
            # Set old value to text label
            self.LoadIPAddress()
            # Load current module
            Module = SetupIOControl.LoadProductUsed(path = self.Base_Path_config)
            self.ui_ipio.CurrentModule.setText(str(Module))
            self.ui_ipio.IP.setText(self._ip)
        except:
            self.AleartBoxERROR(description = 'Can\'t load IP I/O Control !')
    # Setup button on GUI
    def SetupButtonOnIPConfigWindow(self):
        self.ui_ipio.OKBT.clicked.connect(self.UpdateIP_I_O_Control)
    # When click OK Button on IP config windows, Confirm to update IP
    def UpdateIP_I_O_Control(self):
        # Load IP address of module in config file
        newIP = self.ui_ipio.NEWIP.text()
    # Update IP I/O Control
        splitIP = newIP.split('.')
        # when split IP with '.' , it should be 4 number
        if len(splitIP) == 4:
            # if update IP success, it's return True
            self._ip = newIP
            productUsed = SetupIOControl.LoadProductUsed(path = self.Base_Path_config)
            confirm = SetupIOControl.UpdateIP_IO_Control(path = self.Base_Path_config, ip = self._ip, moduleName = productUsed)
            if confirm:
                self.AleartBoxSuccess(description = 'Update IP Successs !')
                # set up IP 
                self.LoadIPAddress()
                # Close windows
                self.widget_ipio.close()
            else:
                self.AleartBoxERROR(description = 'Update IP Failed !')
                # Close windows, then open again
                self.widget_ipio.close()
        else:
            self.AleartBoxERROR(description = 'IP must be 4 number and 3 point !')
            # Clear text box
            self.ui_ipio.NEWIP.clear()

########## IP I/O Config ##########

########## Selected I/O Module ##########

    # Open Selected I/O Module windows
    def OpenSelectedIOModuleWindow(self):
        self.widget_selected_IOMODULE = QtWidgets.QMainWindow()
        self.ui_selected_io_module = UI.gui_selectedIOModule.Ui_MainWindow()
        self.ui_selected_io_module.setupUi(self.widget_selected_IOMODULE)
        self.widget_selected_IOMODULE.setWindowTitle('Selected I/O Module')
        try:
            self.widget_selected_IOMODULE.setWindowIcon(QtGui.QIcon(os.path.join(ROOT, r'imagefile\product.png')))
        except:
            pass
        self.widget_selected_IOMODULE.setFixedHeight(334)
        self.widget_selected_IOMODULE.setFixedWidth(296)
        # Load I/O module in config file
        self.LoadIOModule()
        # Setup button
        self.SetUpButtonOnSelectedIOModuleWindow()
        self.widget_selected_IOMODULE.show()

    # Load I/O module in config file
    def LoadIOModule(self):
        try:
            # Load Product list in config file
            ModuleName = SetupIOControl.LoadProductList(path = self.Base_Path_config)
            # Add current module used
            self.currentModuleUsed = SetupIOControl.LoadProductUsed(path = self.Base_Path_config)
            self.ui_selected_io_module.Currentmodule.setText(str(self.currentModuleUsed))
            # Add Items to product list
            self.ui_selected_io_module.ProductIOList.addItems([str(keys) for keys in ModuleName])
        except:
            self.AleartBoxERROR(description = 'Can\'t load I/O Module !')

    # Setup button in Selected I/O module
    def SetUpButtonOnSelectedIOModuleWindow(self):
        self.ui_selected_io_module.SelectModule.setEnabled(False)
        self.ui_selected_io_module.SelectModule.clicked.connect(self.WriteModuleSelectedInConfig)
        self.ui_selected_io_module.ProductIOList.itemClicked.connect(self.ChangeModuleName)

    # Write module selected in config file
    def ChangeModuleName(self):
        # Module that selected for use in system
        self.Module_IO_Selected = self.ui_selected_io_module.ProductIOList.currentItem().text()
        # change state of button
        self.ui_selected_io_module.SelectModule.setEnabled(True)

    # Write config in config file
    def WriteModuleSelectedInConfig(self):
        try:
            state = SetupIOControl.OverWriteSetupModuleSelected(path = self.Base_Path_config, moduleIO = self.Module_IO_Selected)
            if state:
                # Aleart box success if update success
                self.AleartBoxSuccess(description = 'Update I/O Module Success !')
                # Close windows
                self.widget_selected_IOMODULE.close()
            else:
                self.AleartBoxERROR(description = 'Update I/O Module Failed !')
                # Close windows, then open again
                self.widget_selected_IOMODULE.close()
                self.OpenSelectedIOModuleWindow()
            self.ui_selected_io_module.SelectModule.setEnabled(False)
        except:
            self.AleartBoxERROR(description = 'Update I/O Module Failed !')
            # Close windows, then open again
            self.widget_selected_IOMODULE.close()
            self.OpenSelectedIOModuleWindow()
            self.ui_selected_io_module.SelectModule.setEnabled(False)

########## Selected I/O Module ##########

########## Setup output channel usage ##########

    # Open output channel usage windows
    def SetUpOutputChannel(self):
        self.widget_outputchannel = QtWidgets.QMainWindow()
        self.ui_outputchannel = UI.gui_setupOutputChannel.Ui_MainWindow()
        self.ui_outputchannel.setupUi(self.widget_outputchannel)
        self.widget_outputchannel.setWindowTitle('Output Channel Setting')
        try:
            self.widget_outputchannel.setWindowIcon(QtGui.QIcon(os.path.join(ROOT, r'imagefile\control.png')))
        except:
            pass
        self.widget_outputchannel.setFixedHeight(460)
        self.widget_outputchannel.setFixedWidth(399)
        # Setup output channel
        self.LoadProduct()
        # Disable all button
        self.DisableAllButton()
        # Setup button
        self.SetupButtonOrSignal_On_ChannelSelectedWindow()
        # Load initial product
        self.LoadOutputChannelToChannelList()
        self.widget_outputchannel.show()

    # Load Output all channel
    def LoadProduct(self):
        product = SetupIOControl.LoadProductUsed(path = self.Base_Path_config)
        # Add Product list to combobox
        self.ui_outputchannel.Currentmodule.setText(product)

    # setup button
    def SetupButtonOrSignal_On_ChannelSelectedWindow(self):
        # When items in Channel list clicked
        self.ui_outputchannel.ChannelList.itemClicked.connect(self.ChannelListSelected)
        # When items in OK list clicked
        self.ui_outputchannel.OKList.itemClicked.connect(self.OKListSelected)
        # When items in NG list clicked
        self.ui_outputchannel.NGList.itemClicked.connect(self.NGListSelected)
        # Add to OK list button
        self.ui_outputchannel.AddtoOK.clicked.connect(self.AddItemToOKList)
        # Add to NG list button
        self.ui_outputchannel.AddtoNG.clicked.connect(self.AddItemToNGList)
        # Remove from OK list button
        self.ui_outputchannel.RemovefromOK.clicked.connect(self.RemoveItemFromOKList)
        # Remove from NG list button
        self.ui_outputchannel.RemovefromNG.clicked.connect(self.RemoveItemFromNGList)
        # Confirm Setup button
        self.ui_outputchannel.Confirm.clicked.connect(self.ConfirmSetupWriteInConfigFile)
    
    # load output channel to channel list
    def LoadOutputChannelToChannelList(self):
        # Product selected in combobox product list
        currentModuleUsed = SetupIOControl.LoadProductUsed(path = self.Base_Path_config)
        # All product in config
        product = SetupIOControl.LoadProductList(path = self.Base_Path_config)
        # Quality output, it's output channel quality of product that you selected
        Quality_Output_ChannelList = [i for i in range(product[currentModuleUsed]['Output'])]
        # Product usage in currentModuleUsed from config file return with key 'OK', 'NG'
        productUsage = SetupIOControl.LoadOutputUsage(path = self.Base_Path_config, productname = currentModuleUsed)
        # Channel for OK signal
        self.ChanneloutputOK = productUsage['OK']
        # remove output usage for OK Channel out of Quality_Output_ChannelList
        for outputch in self.ChanneloutputOK:
            if outputch in Quality_Output_ChannelList:
                Quality_Output_ChannelList.remove(outputch)
        # Channel for NG signal
        self.ChanneloutputNG = productUsage['NG']
        # remove output usage for NG Channel out of Quality_Output_ChannelList
        for outputch in self.ChanneloutputNG:
            if outputch in Quality_Output_ChannelList:
                Quality_Output_ChannelList.remove(outputch)
        # Clear Channel list, OK List and NG List
        self.ui_outputchannel.ChannelList.clear()
        self.ui_outputchannel.OKList.clear()
        self.ui_outputchannel.NGList.clear()
        # Add items to channel list
        try:
            # Add items after remove channel is already in product usage
            self.ui_outputchannel.ChannelList.addItems(['Channel ' + str(i) for i in Quality_Output_ChannelList])
            # Add items in OK List
            self.ui_outputchannel.OKList.addItems(['Channel ' + str(i) for i in self.ChanneloutputOK])
            # Add items in NG List
            self.ui_outputchannel.NGList.addItems(['Channel ' + str(i) for i in self.ChanneloutputNG])
        except:
            pass
        
    # Disable all Button
    def DisableAllButton(self):
        self.ui_outputchannel.AddtoNG.setEnabled(False)
        self.ui_outputchannel.AddtoOK.setEnabled(False)
        self.ui_outputchannel.RemovefromNG.setEnabled(False)
        self.ui_outputchannel.RemovefromOK.setEnabled(False)

    # When items in Channel list clicked
    def ChannelListSelected(self):
        # Disable button
        self.DisableAllButton()
        # Enable some button
        self.ui_outputchannel.AddtoOK.setEnabled(True)
        self.ui_outputchannel.AddtoNG.setEnabled(True)
        # [0] is channel name, [1] is channel row number, [2] is name of dict for remove item in ui_outputchannel
        self.ItemSelected = [self.ui_outputchannel.ChannelList.currentItem().text(), self.ui_outputchannel.ChannelList.currentIndex().row(), 'ChannelList']

    # When items in OK list clicked
    def OKListSelected(self):
        # Disable button
        self.DisableAllButton()
        # Enable some button
        self.ui_outputchannel.RemovefromOK.setEnabled(True)
        self.ui_outputchannel.AddtoNG.setEnabled(True)
        # [0] is channel name, [1] is channel row number, [2] is name of dict for remove item in ui_outputchannel
        self.ItemSelected = [self.ui_outputchannel.OKList.currentItem().text(), self.ui_outputchannel.OKList.currentIndex().row(), 'OKList']

    # When items in NG list clicked
    def NGListSelected(self):
        # Disable button
        self.DisableAllButton()
        # Enable some button
        self.ui_outputchannel.RemovefromNG.setEnabled(True)
        self.ui_outputchannel.AddtoOK.setEnabled(True)
        # [0] is channel name, [1] is channel row number, [2] is name of dict for remove item in ui_outputchannel
        self.ItemSelected = [self.ui_outputchannel.NGList.currentItem().text(), self.ui_outputchannel.NGList.currentIndex().row(), 'NGList']

    # When click Add to OK list button
    def AddItemToOKList(self):
        try:
            # Add item to OK list
            self.ui_outputchannel.OKList.addItem(self.ItemSelected[0])
            # remove item from channel list to OK list
            self.ui_outputchannel.__dict__[self.ItemSelected[2]].takeItem(self.ItemSelected[1])
        except:
            self.AleartBoxERROR(description = 'Add to OK list failed !')
        # Sorted Item
        self.ui_outputchannel.OKList.sortItems()
        # Disable button
        self.DisableAllButton()
    
    # When click Add to NG list button
    def AddItemToNGList(self):
        try:
            # Add item to NG list
            self.ui_outputchannel.NGList.addItem(self.ItemSelected[0])
            # remove item from channel list to NG list
            self.ui_outputchannel.__dict__[self.ItemSelected[2]].takeItem(self.ItemSelected[1])
        except:
            self.AleartBoxERROR(description = 'Add to NG list failed !')
        # Sorted Item
        self.ui_outputchannel.NGList.sortItems()
        # Disable button
        self.DisableAllButton()

    # When click Remove from OK list button
    def RemoveItemFromOKList(self):
        try:
            # Add item to channel list
            self.ui_outputchannel.ChannelList.addItem(self.ItemSelected[0])
            # remove item from OK list
            self.ui_outputchannel.OKList.takeItem(self.ItemSelected[1])
        except:
            self.AleartBoxERROR(description = 'Remove from OK list failed !')
        # Sorted Item
        self.ui_outputchannel.ChannelList.sortItems()
        # Disable button
        self.DisableAllButton()

    # When click Remove from NG list button
    def RemoveItemFromNGList(self):
        try:
            # Add item to channel list
            self.ui_outputchannel.ChannelList.addItem(self.ItemSelected[0])
            # remove item from NG list
            self.ui_outputchannel.NGList.takeItem(self.ItemSelected[1])
        except:
            self.AleartBoxERROR(description = 'Remove from NG list failed !')
        # Sorted Item
        self.ui_outputchannel.ChannelList.sortItems()
        # Disable button
        self.DisableAllButton()

    # Confirm Setup button
    def ConfirmSetupWriteInConfigFile(self):
        try:
            # load items in OK List
            OKList = [int((self.ui_outputchannel.OKList.item(i).text()).split('l')[-1].strip()) for i in range(self.ui_outputchannel.OKList.count())]
            # load items in NG List
            NGList = [int((self.ui_outputchannel.NGList.item(i).text()).split('l')[-1].strip()) for i in range(self.ui_outputchannel.NGList.count())]
            # Product usaged
            productUsed = SetupIOControl.LoadProductUsed(path = self.Base_Path_config)
            # Over write output channel in config file
            state = SetupIOControl.OverWriteSetupOutputUsage(path = self.Base_Path_config, productused = productUsed, ok_channel = OKList, ng_channel = NGList)
            # state it's status return True if write file with new value success
            if state:
                self.AleartBoxSuccess(description = 'Update Output success !')
            else:
                self.AleartBoxERROR(description = 'Update Output failed !')
            self.widget_outputchannel.close()
        except:
            self.AleartBoxERROR(description = 'Update Output failed !')
            self.widget_outputchannel.close()

########## Setup output channel usage ##########


########## Control output system ##########

    # Load IP address from config file in InitialSetup Folder
    def LoadIPAddress(self):
        try:
            currentModuleUsed = SetupIOControl.LoadProductUsed(path = self.Base_Path_config)
            self._ip = ControlOutputSystem.LoadIPAddress(path = self.Base_Path_config, moduleIO = currentModuleUsed)
        except:
            self.AleartBoxERROR(description = 'Can\'t load IP address !')

    # Check  I/O connect status
    def CheckConnected_IO_Control(self):
        status = ControlOutputSystem.CheckConnection(base_address = 'http://' + self._ip + '/do_value/slot_0/ch_0')
        if status:
            self.AleartBoxSuccess(description = 'Connected I/O Control success !')
        else:
            self.AleartBoxERROR(description = 'Connected I/O Control failed !') 

    # Control output when status is OK
    def ControlOutputOK(self):
        # Load module used
        currentModuleUsed = SetupIOControl.LoadProductUsed(path = self.Base_Path_config)
        # open channel use for OK
        channelOuptut = ControlOutputSystem.LoadChannelForOK(path = self.Base_Path_config, moduleIO = currentModuleUsed)
        try:
            # Load output setup in config file for control with setting in config
            for i in channelOuptut:
                # On device
                ControlOutputSystem.OnDevice(address = 'http://' + self._ip + '/do_value/slot_0/ch_' + str(i), channel = i)
            # delay before send to OffDevice
            tm.sleep(self.delayOn)
            # Off Device
            for i in channelOuptut:
                ControlOutputSystem.OffDevice(address = 'http://' + self._ip + '/do_value/slot_0/ch_' + str(i), channel = i)
        except:
            self.AleartBoxERROR(description = 'Can\'t control output !')
        # stop timer
        self.tmtx1.stop()

    # Control Output NG
    def ControlOutputNG(self):
        # Load module used
        currentModuleUsed = SetupIOControl.LoadProductUsed(path = self.Base_Path_config)
        # open channel use for OK
        channelOuptut = ControlOutputSystem.LoadChannelForNG(path = self.Base_Path_config, moduleIO = currentModuleUsed)
        try:
            # Channel of NG List
            for i in channelOuptut:
                ControlOutputSystem.OnDevice(address = 'http://' + self._ip + '/do_value/slot_0/ch_' + str(i), channel = i)
            # delay before send to OffDevice
            tm.sleep(self.delayOn)
            # Off Device
            for i in channelOuptut:
                ControlOutputSystem.OffDevice(address = 'http://' + self._ip + '/do_value/slot_0/ch_' + str(i), channel = i)
        except:
            self.AleartBoxERROR(description = 'Can\'t control output !')
        # stop timer
        self.tmtx2.stop()

########## Control output system ##########

########## Input system ##########

    # Setup input channel for auto capture with signal rising edge
    def InputRisingEdgeCaptureAuto(self):
        try:
            # Load name of product used
            currentModuleUsed = SetupIOControl.LoadProductUsed(path = self.Base_Path_config)
            # Load channel that's setup for rising edge
            Channel = ControlInputSystem.LoadInputRisingEdge(path = self.Base_Path_config, moduleIO = currentModuleUsed)
            # Load state of channel
            status = ControlInputSystem.LoadValueInputRisingEdge_From_IO(ip = self._ip, channel = Channel)
            # if status it's True can change status for capture auto mode
            if status:
                # if StatusFromIO it's True, it's mean recieve status Rising Edge from Input I/O 
                self.StatusFromIO = True
        except:
            pass

    # laod input signal for view bit address from machine
    def LoadInputChannelInProduct(self):
        try:
            # Load name of product used
            currentModuleUsed = SetupIOControl.LoadProductUsed(path = self.Base_Path_config)
            # Load channel in config file
            Channel = ControlInputSystem.LoadInputChannel(path = self.Base_Path_config, moduleIO = currentModuleUsed)
            # Channel of input that can use it
            channel_able = Channel['ch'] - len(Channel['ch_rising'])
            # Value return from module read value from input channel
            BitAddressReturn = ControlInputSystem.LoadValueInputBitAddress(ip = self._ip, channel = channel_able)
        except:
            pass

########## Input system ##########

########## Close program ##########

    def CloseProgram(self):
        # uninit camera
        self.UnInitCamera()
        # Close other windows
        try:
            self.widget_exposure.close()
        except:
            pass
        try:
            self.widget_model.close()
        except:
            pass
        try:
            self.widget_threshold.close()
        except:
            pass
        try:
            self.widget_delay.close()
        except:
            pass
        try:
            self.widget_ipio.close()
        except:
            pass
        try:
            self.widget_outputchannel.close()
        except:
            pass
        try:
            self.widget_selected_IOMODULE.close()
        except:
            pass
        # change status of stream to false to stop thread
        self.check_stream_0 = False
        self.check_stream_1 = False
        # Change status from False to True for stop thread read input value from I/O control
        self.StopThreadInput = True
        # Close main UI
        MainWindow.close()

########## Close program ##########

# Defind object for start system
object = GUISystem()

# Thread 1 Camera0
class ThreadCamera0(th):
    def __init__(self):
        th.__init__(self)
    def run(self):
        object.LoadStream_Camera0()

# Thread 2 Camera1
class ThreadCamera1(th):
    def __init__(self):
        th.__init__(self)
    def run(self):
        object.LoadStream_Camera1()

# Thread for load input signal from I/O module
class ThreadInputChannelIn_IO_Module(th):
    def __init__(self):
        th.__init__(self)
    def run(self):
        while True:
            # it's argument for stop thread for capture auto
            if object.StopThreadInput:
                break
            else:
                # Input rising edge mode for capture image with signal
                object.InputRisingEdgeCaptureAuto()
                # read value from all input cahnnel excepted rising edge channel
                object.LoadInputChannelInProduct()

if __name__ == '__main__':
    # found camera not equal 2
    if object.DevListFound != 2:
        object.AleartBox_FoundCamera()
    else:
        # start thread, and mainwindows start
        th0 = ThreadCamera0()
        th0.start()
        th1 = ThreadCamera1()
        th1.start()
        th2 = ThreadInputChannelIn_IO_Module()
        th2.start()
        try:
            MainWindow.show()
        except:
            MainWindow.show()
        sys.exit(app.exec_())