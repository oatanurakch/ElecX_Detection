import json
import requests as rq
import numpy as np
from requests.auth import *
from threading import Thread as th
import time as tm

# Authroization with HTTP Basic auth
auth = HTTPBasicAuth('root', '00000000')

class ControlInputSystem:
    def __init__(self):
        pass
    # Load IP Address from JSON
    def LoadIPAddress(self, path, moduleIO):
        # Address of output channel 0
        with open(path, 'r') as f:
            datas = json.load(f)
        return datas['ControlExtend']['Product'][moduleIO]['IP']
        
    # Load channel input type rising edge
    def LoadInputRisingEdge(self, path, moduleIO):
        # Address of output channel 0
        with open(path, 'r') as f:
            datas = json.load(f)
        return datas['ControlExtend']['Product'][moduleIO]['InputUsage']['Rising']

    # Load value 
    def SetValueInitialRising(self):
        self.DataInput = np.zeros(5).astype('int')

    # Load value input from I/O
    def LoadValueInputRisingEdge_From_IO(self, ip, channel):
        address = 'http://' + ip + '/di_value/slot_0/ch_' + str(channel)
        # Load value from I/O Control, then loads for attrach value in json form
        DataReturn = json.loads((rq.get(address, auth = auth, timeout = 3)).text)
        status = DataReturn['Val']
        # Append value for check state of input 
        self.DataInput = np.append(self.DataInput[1:], status)
        if self.DataInput[-1] == 1 and self.DataInput[-2] == 0:
            return True
     
class Thread1(th):
    def __init__(self):
        th.__init__(self)
    def run(self):
        while True:
            obj.LoadValueFrom_IO(ip = '192.168.1.1', channel = 0)
            
if __name__ == '__main__':
    obj = ControlInputSystem()
    obj.SetValueInitialRising()
    th1 = Thread1()
    th1.start()