import os
import sys
from pathlib import Path
import json

FILE = Path(__file__).resolve()
ROOT = FILE.parents[0]

class SetupPathModel:
    def __init__(self) -> None:
        self.PathModel = {"PartNumber" : {}}

    # Load model config in InitialSetup/config.json
    def LoadModel(self):
        try:
            with open(os.path.join(ROOT, r'config.json'), 'r') as f:
                datas = json.load(f)
            return datas
        except:
            pass
    
    # Add path for model
    def AddPathModel(self, keys, path):
        # Read JSON file before editing
        try:
            with open(os.path.join(ROOT, r'config.json'), 'r') as f:
                datas = json.load(f)
            datas['PartNumber'].update({keys : {"Name" : keys, "Path" : path, "Address" : ""}})
            # Remove JSON before write again
            os.remove(os.path.join(ROOT, r'config.json'))
            # Create JSON File with all path
            with open(os.path.join(ROOT, r'config.json'), 'a+') as f:
                json.dump(datas, f)
            print('UPDATE JSON SUCCESS!')
        except:
            self.ERROR = True

    # Delete model with selected in GUI
    def DeleteModelPathInConfig(self, keys):
        Error = False
        try:
            with open(os.path.join(ROOT, r'config.json'), 'r') as f:
                datas = json.load(f)
            # Delete config with key in datas from config.json
            del datas['PartNumber'][keys]
            # Remove JSON before write again
            os.remove(os.path.join(ROOT, r'config.json'))
            # Create JSON File with all path
            with open(os.path.join(ROOT, r'config.json'), 'a+') as f:
                json.dump(datas, f)
            print('UPDATE JSON SUCCESS!')
        except:
            Error = True
            return Error

    # Load and update bit address
    def LoadBitAddress(self, modelname):
        Error = False
        try:
            with open(os.path.join(ROOT, r'config.json'), 'r') as f:
                datas = json.load(f)
            datas = datas['PartNumber'][modelname]['Address']
            return datas
        except:
            Error = True
            return Error

    # Update bit address
    def UpdateBitAddress(self, modelname, bitaddress):
        try:
            with open(os.path.join(ROOT, r'config.json'), 'r') as f:
                datas = json.load(f)
            # Check address in config, it's duplicated with new bit it's can't update bit address
            pack_address = []
            for i in datas['PartNumber']:
                pack_address.append(datas['PartNumber'][i]['Address'])
            if bitaddress in pack_address:
                return False
            else:
                datas['PartNumber'][modelname]['Address'] = bitaddress
                # Remove JSON before write again
                os.remove(os.path.join(ROOT, r'config.json'))
                # Create JSON File with all path
                with open(os.path.join(ROOT, r'config.json'), 'a+') as f:
                    json.dump(datas, f)
                return True
        except:
            pass

    def CheckModelInMainFolder(self, modelname):
        try:
            with open(os.path.join(ROOT, r'config.json'), 'r') as f:
                datas = json.load(f)
            if modelname in datas['PartNumber']:
                mainPath = str(datas['PartNumber'][modelname]['Path']).split('\\')[-2]
                subPath = str(datas['PartNumber'][modelname]['Path']).split('\\')[-1]
                return True if subPath in os.listdir(mainPath) else False
        except:
            pass

