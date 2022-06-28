import os 
import sys
import json

class CameraUsedandBorderCamera:
    def __init__(self):
        self.CameraConfig = ''
        
    def LoadConfig(self, path_config = ''):
        with open(path_config, 'r') as f:
            self.CameraConfig = json.load(f)
        for i in self.CameraConfig['Camera']:
            if i.startswith('camera'):
                self.CameraUsed = self.CameraConfig['Camera'][i]
                print('Used camera: {}' .format(self.CameraUsed))
            elif i == 'border_top' and self.CameraUsed >= 1:
                border = self.CameraConfig['Camera'][i]
                if border != []:
                    self.x_left_top = int(border[0])
                    self.y_left_top = int(border[1])
                    self.x_right_top = int(border[2])
                    self.y_right_top = int(border[3])
                    # print(f'X left: {self.x_left_top}, Y left: {self.y_left_top}, X right: {self.x_right_top}, Y right: {self.y_right_top}')
                else:
                    self.x_left_top = None
                    self.y_left_top = None
                    self.x_right_top = None
                    self.y_right_top = None
            elif i == 'border_bottom' and self.CameraUsed >= 1:
                border = self.CameraConfig['Camera'][i]
                if border != []:
                    self.x_left_bottom = int(border[0])
                    self.y_left_bottom = int(border[1])
                    self.x_right_bottom = int(border[2])
                    self.y_right_bottom = int(border[3])
                    # print(f'X left: {self.x_left_bottom}, Y left: {self.y_left_bottom}, X right: {self.x_right_bottom}, Y right: {self.y_right_bottom}')
                else:
                    self.x_left_bottom = None
                    self.y_left_bottom = None
                    self.x_right_bottom = None
                    self.y_right_bottom = None

if __name__ == '__main__':
    cam = CameraUsedandBorderCamera()
    cam.LoadConfig(path_config = r'D:\Product\Thai Marujun Product\Version3\Detect\TSAR\LoadModel\config.json')