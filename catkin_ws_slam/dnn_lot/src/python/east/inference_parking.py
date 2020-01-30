# -- coding: utf-8 --

import cv2
import time
import math
import os
import numpy as np
import tensorflow as tf

import lanms

import parking_model
from icdar import restore_rectangle


dirpath = os.path.dirname(os.path.realpath(__file__))


output_dir      = dirpath + '/out'
model_path      = dirpath + '/model/' 
model_path_tracking      = dirpath + '/model_tracking/' 
test_data_path  = dirpath + '/data/'

gpu_list        = '0'
no_write_images = False


class parking_detection:

    def __init__(self):
        global model_path, model_path_tracking

        self.imgWidth = 800
        self.imgHeigh = 960
        import os
        os.environ['CUDA_VISIBLE_DEVICES'] = gpu_list

        try:
            os.makedirs(output_dir)
        except OSError as e:
            if e.errno != 17:
                raise

        self.frameIdx = 0

        self.graph_detection = tf.Graph()
        self.graph_tracking = tf.Graph()

        with self.graph_detection.as_default():
            config = tf.ConfigProto()
            #config.gpu_options.allow_growth = True
            config.gpu_options.per_process_gpu_memory_fraction = 0.1
            self.sess_detect = tf.Session(config=config)
            init = tf.global_variables_initializer()

            self.sess_detect.run(init)

            self.input_images_detect = tf.placeholder(tf.float32, shape=[None, None, None, 3], name='input_images')
            self.global_step_detect = tf.get_variable('global_step', [], initializer=tf.constant_initializer(0), trainable=False)
            self.f_score_detect, self.f_geometry_detect = parking_model.model(self.input_images_detect, is_training=False)

            variable_averages = tf.train.ExponentialMovingAverage(0.997, self.global_step_detect)    
            saver = tf.train.Saver(variable_averages.variables_to_restore())
            print('Restore from {}'.format(model_path))

            ckpt_state = tf.train.get_checkpoint_state(model_path)
            model_path = os.path.join(model_path, os.path.basename(ckpt_state.model_checkpoint_path))
            saver.restore(self.sess_detect, model_path)

        with self.graph_tracking.as_default():
            config = tf.ConfigProto()
            #config.gpu_options.allow_growth = True
            config.gpu_options.per_process_gpu_memory_fraction = 0.1
            self.sess_track = tf.Session(config=config)
            init = tf.global_variables_initializer()

            self.sess_track.run(init)

            self.input_images_track = tf.placeholder(tf.float32, shape=[None, None, None, 3], name='input_images')
            self.global_step_track = tf.get_variable('global_step', [], initializer=tf.constant_initializer(0), trainable=False)
            self.f_score_track, self.f_geometry_track = parking_model.model(self.input_images_track, is_training=False)

            variable_averages = tf.train.ExponentialMovingAverage(0.997, self.global_step_track)    
            saver = tf.train.Saver(variable_averages.variables_to_restore())
            print('Restore from {}'.format(model_path_tracking))

            ckpt_state = tf.train.get_checkpoint_state(model_path_tracking)
            model_path_tracking = os.path.join(model_path_tracking, os.path.basename(ckpt_state.model_checkpoint_path))
            saver.restore(self.sess_track, model_path_tracking)

    def get_images(self):
        '''
        find image files in test data path
        :return: list of files found
        '''
        files = []
        exts = ['jpg', 'png', 'jpeg', 'JPG']
        for parent, dirnames, filenames in os.walk(test_data_path):
            for filename in filenames:
                for ext in exts:
                    if filename.endswith(ext):
                        files.append(os.path.join(parent, filename))
                        break
        print('Find {} images'.format(len(files)))
        return files


    def resize_image(self, im, max_side_len=2400):
        '''
        resize image to a size multiple of 32 which is required by the network
        :param im: the resized image
        :param max_side_len: limit of max image size to avoid out of memory in gpu
        :return: the resized image and the resize ratio
        '''
        h, w, _ = im.shape

        resize_w = w
        resize_h = h

        # limit the max side
        if max(resize_h, resize_w) > max_side_len:
            ratio = float(max_side_len) / resize_h if resize_h > resize_w else float(max_side_len) / resize_w
        else:
            ratio = 1.
        resize_h = int(resize_h * ratio)
        resize_w = int(resize_w * ratio)

        resize_h = resize_h if resize_h % 32 == 0 else (resize_h // 32 - 1) * 32
        resize_w = resize_w if resize_w % 32 == 0 else (resize_w // 32 - 1) * 32
        im = cv2.resize(im, (int(resize_w), int(resize_h)))

        ratio_h = resize_h / float(h)
        ratio_w = resize_w / float(w)

        return im, (ratio_h, ratio_w)

    #def detect(self, score_map, geo_map, timer, score_map_thresh=0.85, box_thresh=0.1, nms_thres=0.2):
    def detect(self, score_map, geo_map, timer, score_map_thresh=0.8, box_thresh=0.2, nms_thres=0.2):
        '''
        restore text boxes from score map and geo map
        :param score_map:
        :param geo_map:
        :param timer:
        :param score_map_thresh: threshhold for score map
        :param box_thresh: threshhold for boxes
        :param nms_thres: threshold for nms
        :return:
        '''
        if len(score_map.shape) == 4:
            score_map = score_map[0, :, :, 0] 
            geo_map = geo_map[0, :, :, ]      

        # filter the score map
        xy_text = np.argwhere(score_map > score_map_thresh) # (560, 2)
        # sort the text boxes via the y axis
        xy_text = xy_text[np.argsort(xy_text[:, 0])]        # (560, 2) 
        # print('{} text boxes after thresh'.format(xy_text.shape[0]))
        # restore
        start = time.time()
        # (1035, 4, 2)                       # （n,2)*4   （n,5）
        text_box_restored = restore_rectangle(xy_text[:, ::-1]*4, # x,y互换
                                            geo_map[xy_text[:, 0], xy_text[:, 1], :])
        boxes = np.zeros((text_box_restored.shape[0], 9), dtype=np.float32) # (N,9)
        boxes[:, :8] = text_box_restored.reshape((-1, 8))
        boxes[:, 8]  = score_map[xy_text[:, 0], xy_text[:, 1]]
        timer['restore'] = time.time() - start  
        # nms part
        start = time.time()
        # boxes = nms_locality.nms_locality(boxes.astype(np.float64), nms_thres)
        boxes = lanms.merge_quadrangle_n9(boxes.astype('float32'), nms_thres) #(2, 9)
        # print('{} text boxes after nms'.format(boxes.shape[0]))
        timer['nms'] = time.time() - start

        if boxes.shape[0] == 0:
            return None, timer

        # here we filter some low score boxes by the average score map, this is different from the orginal paper
        for i, box in enumerate(boxes):
            mask = np.zeros_like(score_map, dtype=np.uint8)
            cv2.fillPoly(mask, box[:8].reshape((-1, 4, 2)).astype(np.int32) // 4, 1)
            boxes[i, 8] = cv2.mean(score_map, mask)[0]
        boxes = boxes[boxes[:, 8] > box_thresh]

        return boxes, timer


    def sort_poly(self, p):
        min_axis = np.argmin(np.sum(p, axis=1))
        p = p[[min_axis, (min_axis+1)%4, (min_axis+2)%4, (min_axis+3)%4]]
        if abs(p[0, 0] - p[1, 0]) > abs(p[0, 1] - p[1, 1]):
            return p
        else:
            return p[[0, 3, 2, 1]]

    def histogramEqualization(self, img):
        img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        s, v, h = cv2.split(img)
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        h = clahe.apply(h)
        hsv_img = cv2.merge([s, v, h])
        out = cv2.cvtColor(hsv_img, cv2.COLOR_HSV2BGR)

        return out

    def all(self, image, is_detect=True):

        self.frameIdx += 1
        # im = image[:, :, ::-1]#bgr->rgb
        start_time = time.time()

        im_resized, (ratio_h, ratio_w) = self.resize_image(image)

        # histogram equalization
        im_resized = self.histogramEqualization(im_resized)

        timer = {'net': 0, 'restore': 0, 'nms': 0}
        start = time.time()

        if is_detect:
            with self.graph_detection.as_default():
                self.score_detect, self.geometry_detect = self.sess_detect.run([self.f_score_detect, self.f_geometry_detect], feed_dict={self.input_images_detect: [im_resized]})
            boxes, timer = self.detect(score_map=self.score_detect, geo_map=self.geometry_detect, timer=timer)
        else:
            with self.graph_tracking.as_default():
                self.score_track, self.geometry_track = self.sess_track.run([self.f_score_track, self.f_geometry_track], feed_dict={self.input_images_track: [im_resized]})
            boxes, timer = self.detect(score_map=self.score_track, geo_map=self.geometry_track, timer=timer)

        timer['net'] = time.time() - start

        
        #print(self.frameIdx, '  net {:.0f}ms, restore {:.0f}ms, nms {:.0f}ms'.format(
        #        timer['net']*1000, 
        #        timer['restore']*1000, 
        #        timer['nms']*1000))

        if boxes is not None:
            boxes = boxes[:, :8].reshape((-1, 4, 2))
            boxes[:, :, 0] /= ratio_w
            boxes[:, :, 1] /= ratio_h

        # scale filter
        boxesTmp = []
        if boxes is not None:
            for box in boxes:
                acLen = pow((box[0, 0] - box[2, 0]), 2) + pow((box[0, 1] - box[2, 1]), 2)
                if acLen > pow(130, 2):
                    boxesTmp.append(box)
        boxes = np.array(boxesTmp, dtype=int)
    
        # save to file
        #if boxes is not None:
        #    res_file = os.path.join(
        #        output_dir,
        #        '{}.txt'.format(
        #            os.path.basename(str(self.frameIdx)).split('.')[0]))
#
        #    with open(res_file, 'w') as f:
        #        for box in boxes:
        #            # to avoid submitting errors
        #            box = self.sort_poly(box.astype(np.int32))
        #            if np.linalg.norm(box[0] - box[1]) < 5 or np.linalg.norm(box[3]-box[0]) < 5:
        #                continue
        #          #  f.write('{},{},{},{},{},{},{},{}\r\n'.format(
        #           #     box[0, 0], box[0, 1], box[1, 0], box[1, 1], box[2, 0], box[2, 1], box[3, 0], box[3, 1],
        #           # ))
        #            cv2.polylines(image, [box.astype(np.int32).reshape((-1, 1, 2))], True, color=(0, 255, 0), thickness=2)
       # if not no_write_images:
       #     img_path = os.path.join(output_dir, os.path.basename(str(self.frameIdx) + '.jpg'))
       #   #  cv2.imwrite(img_path, image)

        outBoxes = boxes
        if boxes is not None:
            outBoxes.reshape((-1, 8))
            outBoxes = outBoxes.astype(np.float32)

        return outBoxes

    def detect_one(self, image):
        return self.all(image, True)

    def track_one(self, image):
        return self.all(image, False)

if __name__ == '__main__':
    dec = parking_detection()

    im_fn_list = dec.get_images()
    for im_fn in im_fn_list:
        image = cv2.imread(im_fn)
        print("Detect: {}".format(im_fn))
        print(dec.detect_one(image))

        print("Track: {}".format(im_fn))
        print(dec.track_one(image))

