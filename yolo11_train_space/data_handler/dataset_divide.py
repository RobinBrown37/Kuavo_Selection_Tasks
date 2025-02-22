#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import random
import argparse
 
rootpath = "/home/robinbrown/yolov8/data_handler"
parser = argparse.ArgumentParser()
# 标注文件的地址，根据自己的数据进行修改
parser.add_argument('--label_path', default='./train/labels', type=str, help='input label path')
# 数据集的划分，地址选择自己数据dataSet下
parser.add_argument('--txt_path', default='./dataSet', type=str, help='output dataset path')
opt = parser.parse_args()
 
# 训练与测试数据集比例
trainval_percent = 1.0
train_percent = 0.9
labelfilepath = opt.label_path
txtsavepath = opt.txt_path
# 读取所有已经标注文件的名称
total_label = os.listdir(labelfilepath)
if not os.path.exists(txtsavepath):
    os.makedirs(txtsavepath)
 
num = len(total_label)
list_index = range(num)
tv = int(num * trainval_percent)
tr = int(tv * train_percent)
trainval = random.sample(list_index, tv)
train = random.sample(trainval, tr)

file_trainval = open(txtsavepath + '/trainval.txt', 'w')
file_test = open(txtsavepath + '/test.txt', 'w')
file_train = open(txtsavepath + '/train.txt', 'w')
file_val = open(txtsavepath + '/val.txt', 'w')
 
for i in list_index:
    name = total_label[i][:-4] + '\n'
    # 排除掉生成的classes.txt文件
    if name=='classes' + '\n':
        continue
    if i in trainval:
        file_trainval.write(rootpath + '/train/images/'+ name.replace('\n','') +'.jpg\n')
        if i in train:
            file_train.write(rootpath + '/train/images/'+name.replace('\n','') +'.jpg\n')
        else:
            file_val.write(rootpath + '/train/images/'+ name.replace('\n','') +'.jpg\n')
    else:
        file_test.write(rootpath + '/train/images/'+ name .replace('\n','') +'.jpg\n')
        
file_trainval.close()
file_train.close()
file_val.close()
file_test.close()