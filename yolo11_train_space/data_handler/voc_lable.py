#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os,shutil
#根目录路径
rootpath="/home/robinbrown/yolov8/data_handler/"#待修改路径
# 输出路径
imgtrain=rootpath+"train/images/"
imgval=rootpath+"val/images/"
labeltrain=rootpath+"train/labels/"
labelval=rootpath+"val/labels/"
if not os.path.exists(imgtrain):
    os.makedirs(imgtrain)
if not os.path.exists(imgval):
    os.makedirs(imgval)
if not os.path.exists(labeltrain):
    os.makedirs(labeltrain)
if not os.path.exists(labelval):
    os.makedirs(labelval)

f = open(rootpath+"dataSet/train.txt","r")
lines = f.readlines()
for i in lines:
    shutil.copy(rootpath+"images/"+str(i).replace('\n','')+".jpg",imgtrain+str(i).replace('\n','')+".jpg")
    shutil.copy(rootpath + "labels/" + str(i).replace('\n', '') + ".txt", labeltrain + str(i).replace('\n', '') + ".txt")
 
f = open(rootpath+"dataSet/val.txt","r")
lines = f.readlines()
for i in lines:
    shutil.copy(rootpath+"images/"+str(i).replace('\n','')+".jpg",imgval+str(i).replace('\n','')+".jpg")
    shutil.copy(rootpath + "labels/" + str(i).replace('\n', '') + ".txt", labelval + str(i).replace('\n', '') + ".txt")
shutil.copy(rootpath+"dataSet/train.txt",rootpath+"train.txt")
shutil.copy(rootpath+"dataSet/trainval.txt",rootpath+"trainval.txt")
shutil.copy(rootpath+"dataSet/test.txt",rootpath+"test.txt")
shutil.copy(rootpath+"dataSet/val.txt",rootpath+"val.txt")
