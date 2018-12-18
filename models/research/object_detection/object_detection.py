from __future__ import division
import numpy as np
import os
import six.moves.urllib as urllib
import sys
import tarfile
import tensorflow as tf
import zipfile
import json
import time
import glob
import cv2
import math
from numpy import array
from io import StringIO
from PIL import Image
import math
import os.path
import time
from utils import visualization_utils as vis_util
from utils import label_map_util
from multiprocessing.dummy import Pool as ThreadPool
from matplotlib import pyplot as plt
import time
import os

MAX_NUMBER_OF_BOXES = 10
MINIMUM_CONFIDENCE =0.9

#sess=tf.Session()
PATH_TO_LABELS = 'mscoco_label_map.pbtxt'
#image_path= image_directory+"test_img_10.png"
PATH_TO_TEST_IMAGES_DIR = "images/"


label_map = label_map_util.load_labelmap(PATH_TO_LABELS)
categories = label_map_util.convert_label_map_to_categories(label_map, max_num_classes=sys.maxsize, use_display_name=True)
CATEGORY_INDEX = label_map_util.create_category_index(categories)

# Path to frozen detection graph. This is the actual model that is used for the object detection.
MODEL_NAME = 'model'
PATH_TO_CKPT = MODEL_NAME + '/frozen_inference_graph.pb'
image_directory = os.getenv("HOME") + "/accio/"

def read_cv_image():
    cv_image = [1,1]
    flag=False
    while not flag:
        file_name=image_directory+"test_img_10.png"
        flag=os.path.isfile(file_name)
    if flag:
        cv_image = cv2.imread(file_name)
        cv2.imwrite("images/test_img_10.png",cv_image)
        time.sleep(1)

def load_image_into_numpy_array(image):
    (im_width, im_height) = image.size
    return np.array(image.getdata()).reshape(
        (im_height, im_width, 3)).astype(np.uint8)

def save_image(image_path,total):
     image=cv2.imread(image_path)
     for i in total:
        ymin,ymax,xmin,xmax=i[1]
        cv2.rectangle(image,(xmin,ymax),(xmax,ymin),(0,255,0),2)
     cv2.imwrite(image_path,image)

def detect_objects(image_path,task_num):
    image = Image.open(image_path)
    (im_width,im_height)=image.size
    image2 = cv2.imread(image_path)
    image_np = load_image_into_numpy_array(image)
    image_np_expanded = np.expand_dims(image_np, axis=0)

    (boxes, scores, classes, num) = sess.run([detection_boxes, detection_scores, detection_classes, num_detections], feed_dict={image_tensor: image_np_expanded})
    total=[]
    for i in range(int(num[0])):
        tar=classes[0][i]
        print(tar)
        if tar==task_num:
            ymin=int(boxes[0][i][0]*im_height)
            ymax=int(boxes[0][i][2]*im_height)
            xmin=int(boxes[0][i][1]*im_width)
            xmax=int(boxes[0][i][3]*im_width)
            total.append((tar,[ymin,ymax,xmin,xmax],((xmax+xmin)/2,(ymax+ymin)/2)))
 
    vis_util.visualize_boxes_and_labels_on_image_array(
        	image_np,
        	np.squeeze(boxes),
        	np.squeeze(classes).astype(np.int32),
        	np.squeeze(scores),
        	CATEGORY_INDEX,
        	min_score_thresh=MINIMUM_CONFIDENCE,
        	use_normalized_coordinates=True,
        	line_thickness=8)
    fig = plt.figure()
    fig.set_size_inches(16, 9)
    ax = plt.Axes(fig, [0., 0., 1., 1.])
    ax.set_axis_off()
    fig.add_axes(ax)
    save_image(image_path,total)
    plt.imshow(image_np, aspect = 'auto')
    plt.savefig('output/{}'.format(image_path), dpi = 62)
    plt.close(fig)
    print(total)
    for i in total:
        x,y=i[2]
        save_cv_position(x,y)
    return total

def save_cv_position(x,y):
    # open setup file
    #f = open(self.image_dir + "cv_position.dat", "w")
    f = open(image_directory+"cv_position.dat", "w")
    s = 'x = %d\n' % x
    f.write(s)
    s = 'y = %d\n' % y
    f.write(s)           
    f.close()

def task():
    task = 0
    file_name=image_directory+"task_num.dat"
    flag=False
    while not flag:
        flag=os.path.isfile(file_name)
    f = open(file_name, "r") 
    s = (f.readline()).split()
    if len(s) >= 3:
        task = int(s[2])
    f.close()
    return task

def detection():
    global sess, detection_boxes, detection_scores, detection_classes, num_detections
    global image_tensor
    print('Loading model...')
    detection_graph = tf.Graph()
    with detection_graph.as_default():
        od_graph_def = tf.GraphDef()
        with tf.gfile.GFile(PATH_TO_CKPT, 'rb') as fid:
            serialized_graph = fid.read()
            od_graph_def.ParseFromString(serialized_graph)
            tf.import_graph_def(od_graph_def, name='')
 
    print('detecting...')
    with detection_graph.as_default():
        with tf.Session(graph=detection_graph) as sess:
            image_tensor = detection_graph.get_tensor_by_name('image_tensor:0')
            detection_boxes = detection_graph.get_tensor_by_name('detection_boxes:0')
            detection_scores = detection_graph.get_tensor_by_name('detection_scores:0')
            detection_classes = detection_graph.get_tensor_by_name('detection_classes:0')
            num_detections = detection_graph.get_tensor_by_name('num_detections:0')
            total=[]
            TEST_IMAGE_PATHS = glob.glob(os.path.join(PATH_TO_TEST_IMAGES_DIR, '*.png'))
            for image_path in TEST_IMAGE_PATHS:
                print(image_path)
                total.append(detect_objects("images/test_img_10.png",task_num))

#task_num = 44
task_num = task()
total=[]
# Load model into memory
read_cv_image()
#time.sleep(10)
detection()

