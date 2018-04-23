#!/usr/bin/env python
import json
import os
print('Content-type: text/html; charset=UTF-8\r\n')
file_dict = {0:[], 1:[],2:[],3:[],4:[],5:[],6:[],7:[]}
for i in range(0, 8):
    file_dir = "datas/kansou_husen/" + str(i) + "/"
    files = os.listdir(file_dir)
    for file_name in files:
        file_dict[i].append(file_name)
print json.dumps(file_dict)
