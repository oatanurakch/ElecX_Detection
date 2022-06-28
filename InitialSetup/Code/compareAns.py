import os
import sys
import json

class CompareAns():
    def __init__(self):
        self.answer = []
    def LoadCompareAns(self, path_compare, ansfromDetection, Key, camera):
        # print('Path compare: {}\nansfromDetection: {}' .format(path_compare, ansfromDetection))
        with open(path_compare, 'r') as f:
            datas = json.load(f)

        # Check answer with file
        if camera ==  1:
            # return var to blank list
            self.answer = []
            # Loop for check answer
            for ans in ansfromDetection:
                if ans in datas[Key]['SETUP']:
                    if datas[Key]['SETUP'][ans] != '':
                        # print('Answer: {}' .format(datas[Key]['SETUP'][ans]))
                        self.answer.append(datas[Key]['SETUP'][ans])
                    elif datas[Key]['SETUP'][ans] == '':
                        print('You have\'t set answer for detection: {}' .format(ans))
            justment = datas[Key]['SUMMARY']['OK']
            _justment = len(justment)
            count_OKCODE = 0
            count_NGCODE = 0
            for i in ansfromDetection:
                if i in justment:
                    count_OKCODE += 1
                else:
                    count_NGCODE += 1
            if count_OKCODE == _justment and count_NGCODE == 0:
                OK_State = True
            elif count_NGCODE > 0 and count_OKCODE != _justment:
                OK_State = False
            else:
                OK_State = False
            return [self.answer, count_OKCODE, count_NGCODE, OK_State]

        elif camera == 2:
            self.answer = []
            # Loop for check answer
            for ans in ansfromDetection:
                if ans in datas[Key]['SETUP']:
                    if datas[Key]['SETUP'][ans] != '':
                        # print('Answer: {}' .format(datas[Key]['SETUP'][ans]))
                        self.answer.append(datas[Key]['SETUP'][ans])
                    elif datas[Key]['SETUP'][ans] == '':
                        print('You have\'t set answer for detection: {}' .format(ans))
            justment = datas[Key]['SUMMARY']['OK']
            _justment = len(justment)
            count_OKCODE = 0
            count_NGCODE = 0
            for i in ansfromDetection:
                if i in justment:
                    count_OKCODE += 1
                else:
                    count_NGCODE += 1
            if count_OKCODE == _justment and count_NGCODE == 0:
                OK_State = True
            elif count_NGCODE > 0 and count_OKCODE != _justment:
                OK_State = False
            else:
                OK_State = False
            return [self.answer, count_OKCODE, count_NGCODE, OK_State]