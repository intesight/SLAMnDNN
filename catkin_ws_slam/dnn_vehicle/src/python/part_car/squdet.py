#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Jun 29 15:59:33 2018

@author: nikang
"""

import os
dirpath = os.path.dirname(os.path.realpath(__file__))
#os.chdir(dirpath)

import tensorflow as tf
from squeezeDet import  SqueezeDet
import numpy as np
from create_config import load_dict
from evaluation import filter_batch
from visualization import bbox_transform_single_box
import pandas as pd
import cv2
import datetime
#置信度采用百分制

mirrorthreshold =0.6 #score
carthreshold =0.7
wheelthreshold =0.7


class inference:
    
    def __init__(self):
        self.cfg = load_dict(dirpath + "/squeeze.config")
        self.squeeze = SqueezeDet(self.cfg)
        self.model = self.squeeze.model
        self.model.load_weights(dirpath + '/model.999-0.69.hdf5')
    
    def predict(self, img):
        t = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S') #get the time of picture
        rig_h, orig_w, _ = [float(v) for v in img.shape]
        img = cv2.resize( img, (self.cfg.IMAGE_WIDTH, self.cfg.IMAGE_HEIGHT))
        img = (img - np.mean(img))/ np.std(img)
        img = img.reshape((1,1080,1280,3))
        pred = self.model.predict(img)
        all_filtered_boxes, all_filtered_classes, all_filtered_scores = filter_batch(pred, self.cfg)
        #all_filtered_scores = np.array(np.array(all_filtered_scores)*100 ,dtype=int)#整数
        
        bbox=np.array([])
        for j, det_box in enumerate(all_filtered_boxes[0]):
            #transform into xmin, ymin, xmax, ymax
            line=np.array([])
            det_box = bbox_transform_single_box(det_box)
            if all_filtered_classes[0][j] == 0 and all_filtered_scores[0][j] < carthreshold:
                continue
            elif all_filtered_classes[0][j] == 1 and all_filtered_scores[0][j] < mirrorthreshold:
                continue
            elif all_filtered_classes[0][j] == 2 and all_filtered_scores[0][j] < wheelthreshold:
                continue
            else:
                line = np.append(line,all_filtered_classes[0][j])
                line = np.append(line,all_filtered_scores[0][j])
                line = np.append(line,det_box)
                bbox = np.append(bbox,line)
        bbox = np.reshape(bbox,(-1,6))
        #print bbox
#        write txt        
#        with open('%s.txt'%t,'w') as f:
#            file_data = ""
#            print(bbox.shape[0])
#            for i in range(bbox.shape[0]):
#                file_data += str(bbox[i]).replace('[','').replace(']','')+'\n'
#            f.write(file_data)
#        
#        write csv
#        df = pd.DataFrame(bbox,columns=['car/mirror/wheel','score','xmin','ymin','xmax','ymax'])
#        df.to_csv('%s.csv'%t,header=True,index=False)
        return bbox.astype(np.float32)
#        return img
    
if __name__ == '__main__':
    sqd = inference()
    imgtest = cv2.imread('./car/2018-06-20 11-51-154.png')
    a = sqd.predict(imgtest)
    print a
    
    
