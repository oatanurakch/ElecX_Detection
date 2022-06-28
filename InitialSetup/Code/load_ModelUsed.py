import os
import sys
import json

class LoadModelName:
    def __init__(self):
        self.datas = ''
    def LoadModelJ_File(self, path, mainPath):
        self.ERROR = False
        try:
            with open(path, 'r') as f:
                self.datas = json.load(f)
            self.datas = self.datas['ModelData']
            # Add model used to list
            self.modelUsed = []
            self.modelNotFound = []
            # loop for add model to list
            for i in self.datas:
                # Find model name in mainPath
                if self.datas[i] in os.listdir(mainPath):
                    self.modelUsed.append(self.datas[i])
                elif self.datas[i] not in os.listdir(mainPath) and self.datas[i] != "":
                    self.modelNotFound.append(self.datas[i])
        except:
            self.ERROR = True