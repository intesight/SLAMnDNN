from __future__ import print_function

import argparse
import os
import sys
import time
from PIL import Image
import tensorflow as tf
import numpy as np
from scipy import misc
import cv2

from seg_model import FCN8s, PSPNet50, ENet, ICNet

dirpath = os.path.dirname(os.path.realpath(__file__))

save_dir = dirpath + '/output/'
model_path = {'pspnet': dirpath + '/model/pspnet50.npy',
              'fcn': dirpath + '/model/fcn.npy',
              'enet': dirpath + '/model/cityscapes/enet.ckpt',
              'icnet': dirpath + '/model/cityscapes/icnet.npy'}

class segmentation:
    def __init__(self):
        model = "icnet"

        if model == "enet":
            self.model = ENet()
        elif model == "icnet":
            self.model = ICNet()

        # Init tf Session
        config = tf.ConfigProto()
        #config.gpu_options.allow_growth = True
        config.gpu_options.per_process_gpu_memory_fraction = 0.1
        self.sess = tf.Session(config=config)
        init = tf.global_variables_initializer()
        
        self.sess.run(init)
        
        self.model.load(model_path[model], self.sess)
    
    def segment(self, image):
        #print (self)
        #print (self.model)
        self.model.set_input(image, "outdoor_1.png")
    
        preds = self.model.forward(self.sess)
        #if not os.path.exists(save_dir):
        #    os.makedirs(save_dir)
        #misc.imsave(save_dir + "icnet" + '_' + self.model.img_name + '_1.png', preds[0])
        
        image_seg = preds[0].astype(np.uint8)
        img = cv2.resize(image_seg, (image.shape[1], image.shape[0]), interpolation=cv2.INTER_CUBIC)

        return img

if __name__ == '__main__':
    seg = segmentation()
    img = misc.imread("input/outdoor_1.png", mode='RGB')
    seg.segment(img)
