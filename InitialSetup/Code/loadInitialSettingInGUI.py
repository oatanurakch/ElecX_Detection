import json
import os
import sys

class LoadInitialConf:
    def __init__(self):
        pass
    # load config from Json 
    def LoadInitialConfFromJSON(self, path):
        # Open file json
        with open(path, 'r') as f:
            datas = json.load(f)
        Exposure_initial = datas['InitialConfig']['Exposure']
        Threshold = datas['InitialConfig']['Threshold']
        Delay = datas['InitialConfig']['Delay']
        return [Exposure_initial, Threshold, Delay]
    # write new config from GUI

class UpdateConfig:
    def __init__(self):
        pass

    # update threshold value
    def UpdateConfigThreshold(self, path, threshold):
        try:
            # Open file json
            with open(path, 'r') as f:
                datas = json.load(f)
        except:
            print('ERROR: Cannot open file config_expo_thre_delay.json')
        try:
            datas['InitialConfig']['Threshold'] = threshold
            # Remove JSON before write again
            try:
                os.remove(path)
                # Create JSON File with all path
                with open(path, 'a+') as f:
                    json.dump(datas, f)
            except:
                print('Error: Cannot remove file config_expo_thre_delay.json')
        except:
            print('ERROR: Cannot update config_expo_thre_delay.json')

    # update delay value
    def UpdateConfigDelay(self, path, delay):
        try:
            # Open file json
            with open(path, 'r') as f:
                datas = json.load(f)
        except:
            print('ERROR: Cannot open file config_expo_thre_delay.json')
        try:
            datas['InitialConfig']['Delay'] = delay
            # Remove JSON before write again
            try:
                os.remove(path)
                # Create JSON File with all path
                with open(path, 'a+') as f:
                    json.dump(datas, f)
            except:
                print('Error: Cannot remove file config_expo_thre_delay.json')
        except:
            print('ERROR: Cannot update config_expo_thre_delay.json')

    # update exposure value
    def UpdateConfigExposure(self, path, exposure):
        try:
            # Open file json
            with open(path, 'r') as f:
                datas = json.load(f)
        except:
            print('ERROR: Cannot open file config_expo_thre_delay.json')
        try:
            datas['InitialConfig']['Exposure'] = exposure
            # Remove JSON before write again
            try:
                os.remove(path)
                # Create JSON File with all path
                with open(path, 'a+') as f:
                    json.dump(datas, f)
            except:
                print('Error: Cannot remove file config_expo_thre_delay.json')
        except:
            print('ERROR: Cannot update config_expo_thre_delay.json')

# I/O Module
class SetupIOControl:
    def __init__(self):
        pass
    # Load Product lits in file JSON
    def LoadProductList(self, path):
        try:
            with open(path, 'r') as f:
                datas = json.load(f)
            return datas['ControlExtend']['Product']
        except:
            return False
    # update IP Control
    def UpdateIP_IO_Control(self, path, ip, moduleName):
        try:
            with open(path, 'r') as f:
                datas = json.load(f)
            # update data
            datas['ControlExtend']['Product'][moduleName]['IP'] = ip
            os.remove(path)
            # Create JSON File with all path
            with open(path, 'a+') as f:
                json.dump(datas, f)
            return True
        except:
            pass
    # Load current product used in config
    def LoadProductUsed(self, path):
        try:
            with open(path, 'r') as f:
                datas = json.load(f)
            return datas['ControlExtend']['ProductUsed']
        except:
            return False

    # Load Output usage per product
    def LoadOutputUsage(self, path, productname):
        try:
            with open(path, 'r') as f:
                datas = json.load(f)
            return datas['ControlExtend']['Product'][productname]['OutputUsage']
        except:
            return False
    # Write Product setting
    def OverWriteSetupModuleSelected(self, path, moduleIO):
        try:
            with open(path, 'r') as f:
                datas = json.load(f)
            datas['ControlExtend']['ProductUsed'] = moduleIO
            # remove file , then write again
            os.remove(path)
            # Create file again
            with open(path, 'a+') as f:
                json.dump(datas, f)
            return True
        except:
            return False
    # Write Output usage per product
    def OverWriteSetupOutputUsage(self, path, productused, ok_channel, ng_channel):
        try:
            # Open config file
            with open(path, 'r') as f:
                datas = json.load(f)
            # update value in config file
            datas['ControlExtend']['Product'][productused]['OutputUsage']['OK'] = ok_channel
            datas['ControlExtend']['Product'][productused]['OutputUsage']['NG'] = ng_channel
            # remove file and then write again
            os.remove(path)
            # Write file again with new value
            with open(path, 'a+') as f:
                json.dump(datas, f)
            return True
        except:
            return False
    