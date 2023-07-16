# ---------------------------------------------------------------------
# Exercises from lesson 2 (object detection)
# Copyright (C) 2020, Dr. Antje Muntzinger / Dr. Andreas Haja.  
#
# Purpose of this file : Starter Code
#
# You should have received a copy of the Udacity license together with this program.
#
# https://www.udacity.com/course/self-driving-car-engineer-nanodegree--nd013
# ----------------------------------------------------------------------
#

from PIL import Image
import io
import sys
import os
import cv2
import open3d as o3d
import math
import numpy as np
import zlib

import matplotlib
matplotlib.use('wxagg') # change backend so that figure maximizing works on Mac as well     
import matplotlib.pyplot as plt

# Exercise C2-4-6 : Plotting the precision-recall curve
def plot_precision_recall(): 

    # Please note: this function assumes that you have pre-computed the precions/recall value pairs from the test sequence
    #              by subsequently setting the variable configs.conf_thresh to the values 0.1 ... 0.9 and noted down the results.
    
    # Please create a 2d scatter plot of all precision/recall pairs 

    pass

# Exercise C2-3-4 : Compute precision and recall
def compute_precision_recall(det_performance_all, conf_thresh=0.5):

    if len(det_performance_all)==0 :
        print("no detections for conf_thresh = " + str(conf_thresh))
        return
    
    # extract the total number of positives, true positives, false negatives and false positives
    # format of det_performance_all is [ious, center_devs, pos_negs]
    pos_negs = []
    for item in det_performance_all:
        pos_negs.append(item[2])
    pos_negs_arr = np.asarray(pos_negs)        

    positives = sum(pos_negs_arr[:,0])
    true_positives = sum(pos_negs_arr[:,1])
    false_negatives = sum(pos_negs_arr[:,2])
    false_positives = sum(pos_negs_arr[:,3])
    print("TP = " + str(true_positives) + ", FP = " + str(false_positives) + ", FN = " + str(false_negatives))
    
    # compute precision
    precision = true_positives / (true_positives + false_positives) # When an object is detected, what are the chances of it being real?   
    
    # compute recall 
    recall = true_positives / (true_positives + false_negatives) # What are the chances of a real object being detected?

    print("precision = " + str(precision) + ", recall = " + str(recall) + ", conf_thres = " + str(conf_thresh) + "\n")    
    



# Exercise C2-3-2 : Transform metric point coordinates to BEV space
def pcl_to_bev(lidar_pcl, configs, vis=True):

    # compute bev-map discretization by dividing x-range by the bev-image height

    bev_discret_x = (configs.lim_x[1] - configs.lim_x[0]) / configs.bev_height
    bev_discret_y = (configs.lim_y[1] - configs.lim_y[0]) / configs.bev_width
    # create a copy of the lidar pcl and transform all metrix x-coordinates into bev-image coordinates    
    lidar_pcl_copy = np.copy(lidar_pcl)
    x = lidar_pcl_copy[:,0]

    x = np.int_(x / bev_discret_x)



    # transform all metrix y-coordinates as well but center the foward-facing x-axis on the middle of the image

    y = lidar_pcl_copy[:,1]
    y = np.int_(y / bev_discret_y + (configs.bev_width + 1) / 2)



    lidar_pcl_copy[:, 0] = x
    lidar_pcl_copy[:, 1] = y
    # shift level of ground plane to avoid flipping from 0 to 255 for neighboring pixels
    lidar_pcl_copy[:, 2] = lidar_pcl_copy[:, 2] - configs.lim_z[0]  
    # re-arrange elements in lidar_pcl_cpy by sorting first by x, then y, then by decreasing height

    sorted_indices = np.lexsort((lidar_pcl_copy[:, 0], lidar_pcl_copy[:, 1], -lidar_pcl_copy[:, 2]))

    lidar_pcl_copy = lidar_pcl_copy[sorted_indices]
    
    # extract all points with identical x and y such that only the top-most z-coordinate is kept (use numpy.unique)

    _, indices = np.unique(lidar_pcl_copy[:, :2], axis=0, return_index=True)
    lidar_pcl_copy = lidar_pcl_copy[indices]
    # assign the height value of each unique entry in lidar_top_pcl to the height map and 
    # make sure that each entry is normalized on the difference between the upper and lower height defined in the config file
    
    height = np.zeros((configs.bev_width  + 1, configs.bev_height + 1))
    for x, y , z, intensity in lidar_pcl_copy:
        height[int(x),int(y)] = z
    height = height * 255 / (np.amax(height) - np.amin(height))
    height = height.astype(np.uint8)


    # sort points such that in case of identical BEV grid coordinates, the points in each grid cell are arranged based on their intensity

    sorted_indices = np.lexsort((lidar_pcl_copy[:, 0], lidar_pcl_copy[:, 1], -lidar_pcl_copy[:, 3]))

    lidar_pcl_copy = lidar_pcl_copy[sorted_indices]
        

    # only keep one point per grid cell

    _, indices = np.unique(lidar_pcl_copy[:, :2], axis=0, return_index=True)
    lidar_pcl_copy = lidar_pcl_copy[indices]

    # create the intensity map
    # b = np.array([0, 1e-5, 1e-4, 1e-3, 1e-2, 1e-1, 1, 1e+1, 1e+3, 1e+3, 1e+4, 1e+5, 1e+6, 1e+7])
    # hist,bins = np.histogram(lidar_pcl_copy[:,3], bins=b)
    # print(hist)
    idx_limit = lidar_pcl_copy[:, 3] > 1.0
    lidar_pcl_copy[idx_limit, 3] = 1.0 
    intensity_map = np.zeros((configs.bev_width  + 1, configs.bev_height + 1))
    for x, y , z, intensity in lidar_pcl_copy:
        intensity_map[int(x),int(y)] = intensity
    

    # visualize intensity map
    if vis: 
       img_intensity = intensity_map * 256
       img_intensity = img_intensity.astype(np.uint8)
       while (1):
           cv2.imshow('height', height)
           cv2.imshow('img_intensity', img_intensity)
           if cv2.waitKey(10) & 0xFF == 27:
               break
       cv2.destroyAllWindows()
