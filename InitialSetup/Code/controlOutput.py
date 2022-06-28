import json
import requests as rq
from requests.auth import *
from time import sleep
import os

# base_address = 'http://192.168.1.1/do_value/slot_0/ch_0'
address = 'http://192.168.1.1/ai_value/slot_0/ch_0'
auth = HTTPBasicAuth('root', '00000000')
resolution = (2 ** 16) - 1
msg_control = {"Ch" : 0, "Md" : 0, "Val" : 0, "Stat" : 0, "PsCtn" : 0, "PsStop" : 0, "PsIV" : 0}

class ControlOutputSystem:
    def __init__(self):
        pass
    # Load IP Address from JSON
    def LoadIPAddress(self, path, moduleIO):
        # Address of output channel 0
        with open(path, 'r') as f:
            datas = json.load(f)
        return datas['ControlExtend']['Product'][moduleIO]['IP']
    # Check connection status
    def CheckConnection(self, base_address):
        try:
            rq.get(base_address, auth = auth, timeout = 3)
            return True
        except:
            return False
    # On device
    def OnDevice(self, address, channel):
        print('Channel : {} ON' .format(channel))
        msg_control['Ch'] = channel
        msg_control['Val'] = 1
        msg_control['Stat'] = 1
        rq.put(address, data = json.dumps(msg_control), auth = auth, timeout = 3)
    
    # Off device
    def OffDevice(self, address, channel):
        print('Channel : {} OFF' .format(channel))
        msg_control['Ch'] = channel
        msg_control['Val'] = 0
        msg_control['Stat'] = 0
        rq.put(address, data = json.dumps(msg_control), auth = auth, timeout = 3)

if __name__ == '__main__':
    obj = ControlOutputSystem()
    data = obj.LoadIPAddress(r'D:\Product\Thai Marujun Product\Version3\InitialSetup\config.json')
    obj.PumpOn(address_output_ch0 = data['CH0'])
    sleep(3)
    obj.PumpOff(address_output_ch0 = data['CH0'])