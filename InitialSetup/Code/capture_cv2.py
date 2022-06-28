import cv2
import os
import sys
from datetime import datetime
from natsort import *
import numpy as np

class CaptureImage:
    def __init__(self) -> None:
        pass
    def Capture(self, path_img, camera, partName, sliced_img, frame):
        # print('Path img: {}, camera: {}' .format(path_img, camera))
        self.FileNameIMG = []
        # Used once camera
        if camera == 1:
            # count file in image predict
            index = len(os.listdir(path_img))
            # Top cam for capture image
            filename_topcam = os.path.join(path_img, (str(index + 1) + '_' + partName + '-TOP_' + str(datetime.now().strftime('%d-%m-%Y_%H-%M-%S')) + '.jpg'))
            cropImg = self.CroppedProcess(image = frame, parameter_sliced = sliced_img)
            cv2.imwrite(filename_topcam, cropImg)
            self.FileNameIMG.append(filename_topcam)
        # Used both camera
        elif camera == 2:
            # count file in image predict
            index = len(os.listdir(path_img))
            if index == 0:
                # File name top camera
                filename_topcam = os.path.join(path_img, (str(index + 1) + '_' + partName + '-TOP_' + str(datetime.now().strftime('%d-%m-%Y_%H-%M-%S')) + '.jpg'))
                cropImg = self.CroppedProcess(image = frame[0], parameter_sliced = sliced_img[0])
                cv2.imwrite(filename_topcam, cropImg)
                filename_bottomcam = os.path.join(path_img, (str(index + 1) + '_' + partName + '-BOTTOM_' + str(datetime.now().strftime('%d-%m-%Y_%H-%M-%S')) + '.jpg'))
                cropImg = self.CroppedProcess(image = frame[1], parameter_sliced = sliced_img[1])
                cv2.imwrite(filename_bottomcam, cropImg)
            elif index != 0:
                index = int(natsorted(os.listdir(path_img))[-1].split('_')[0])
                # File name top camera
                filename_topcam = os.path.join(path_img, (str(index + 1) + '_' + partName + '-TOP_' + str(datetime.now().strftime('%d-%m-%Y_%H-%M-%S')) + '.jpg'))
                cropImg = self.CroppedProcess(image = frame[0], parameter_sliced = sliced_img[0])
                cv2.imwrite(filename_topcam, cropImg)
                # File name bottom camera
                filename_bottomcam = os.path.join(path_img, (str(index + 1) + '_' + partName + '-BOTTOM_' + str(datetime.now().strftime('%d-%m-%Y_%H-%M-%S')) + '.jpg'))
                cropImg = self.CroppedProcess(image = frame[1], parameter_sliced = sliced_img[1])
                cv2.imwrite(filename_bottomcam, cropImg)
            self.FileNameIMG.append(filename_topcam)
            self.FileNameIMG.append(filename_bottomcam)

    # Cropped image
    def CroppedProcess(self, image, parameter_sliced):
        self.image = image
        self.parameter_sliced = parameter_sliced
        # Cropped image
        cropImg = self.image[self.parameter_sliced[0]:self.parameter_sliced[1], self.parameter_sliced[2]:self.parameter_sliced[3]]
        filter = np.array([[-1, -1, -1], [-1, 9, -1], [-1, -1, -1]])
        cropImg = cv2.GaussianBlur(cropImg, (3, 3), 0)
        cropImg = cv2.filter2D(cropImg, -1, filter)
        cropImg = cv2.blur(cropImg, (1, 1))
        return cropImg
