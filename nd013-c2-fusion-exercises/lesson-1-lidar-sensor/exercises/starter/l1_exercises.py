# ---------------------------------------------------------------------
# Exercises from lesson 1 (lidar)
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
import numpy as np
import zlib

## Add current working directory to path
sys.path.append(os.getcwd())

## Waymo open dataset reader
from tools.waymo_reader.simple_waymo_open_dataset_reader import dataset_pb2


# Exercise C1-5-5 : Visualize intensity channel
def vis_intensity_channel(frame, lidar_name):

    print("Exercise C1-5-5")
    # extract range image from frame
    lidar = [obj for obj in frame.lasers if obj.name == lidar_name][0] # get laser data structure from frame
    ri = []
    if len(lidar.ri_return1.range_image_compressed) > 0: # use first response
        ri = dataset_pb2.MatrixFloat()
        ri.ParseFromString(zlib.decompress(lidar.ri_return1.range_image_compressed))
        ri = np.array(ri.data).reshape(ri.shape.dims)
    ri[ri<0]=0.0
    # map value range to 8bit
    ri_range = ri[:,:,1]
    ri_range = np.amax(ri_range) / 2 *  ri_range * 255 / (np.amax(ri_range) - np.amin(ri_range))
    img_range = ri_range.astype(np.uint8)
    # focus on +/- 45° around the image center

    deg45 = int(img_range.shape[1] / 8)
    ri_center = int(img_range.shape[1]/2)
    img_range = img_range[:,ri_center-deg45:ri_center+deg45]

    print('max. val = ' + str(round(np.amax(img_range[:,:]),2)))
    print('min. val = ' + str(round(np.amin(img_range[:,:]),2)))

    cv2.imshow('range_image', img_range)
    cv2.waitKey(0)

# Exercise C1-5-2 : Compute pitch angle resolution
def print_pitch_resolution(frame, lidar_name):

    print("Exercise C1-5-2")
    # load range image
    lidar = [obj for obj in frame.lasers if obj.name == lidar_name][0] # get laser data structure from frame
    ri = []
    if len(lidar.ri_return1.range_image_compressed) > 0: # use first response
        ri = dataset_pb2.MatrixFloat()
        ri.ParseFromString(zlib.decompress(lidar.ri_return1.range_image_compressed))
        ri = np.array(ri.data).reshape(ri.shape.dims)
    # compute vertical field-of-view from lidar calibration 
    calib_lidar = [obj for obj in frame.context.laser_calibrations if obj.name == lidar_name][0]
    beam_inclination_min = calib_lidar.beam_inclination_min
    beam_inclination_max = calib_lidar.beam_inclination_max

    # compute pitch resolution and convert it to angular minutes
    vfov = beam_inclination_max - beam_inclination_min
    pitch_res = vfov / ri.shape[0] * 180 / np.pi
    print("pitch angle resolution = " + '{0:.2f}'.format(pitch_res) + "°")

    

# Exercise C1-3-1 : print no. of vehicles
def print_no_of_vehicles(frame):

    print("Exercise C1-3-1")    

    # find out the number of labeled vehicles in the given frame
    # Hint: inspect the data structure frame.laser_labels
    lidar_name = dataset_pb2.LaserName.TOP
    num_vehicles = 0
    for i in frame.laser_labels:
        if i.type == i.TYPE_VEHICLE:
            num_vehicles+= 1
            
    print("number of labeled vehicles in current frame = " + str(num_vehicles))