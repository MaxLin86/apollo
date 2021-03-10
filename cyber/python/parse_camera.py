"""
function to parse camera images from *.record files, created using Apollo-Auto

parsed data is saved to *.jpeg file, for each capture

"""

import sys, os
from cyber_py import cyber
from cyber_py import record
from modules.drivers.proto.sensor_image_pb2 import CompressedImage, Image

import base64
import cv2
import numpy as np

###########################################################
def parse_data(channelname, msg, out_folder, index, frame_index):
    """
    parser images from Apollo record file
    """
    
    # msg_camera = CompressedImage()
    msg_camera = Image()

    # print(str(msg))
    msg_camera.ParseFromString(str(msg))

    tstamp = msg_camera.measurement_time

    temp_time = str(tstamp).split('.')

    # print(temp_time)
    # if len(temp_time[1])==1:
    #     temp_time1_adj = temp_time[1] + '0'
    # else:
    #     temp_time1_adj = temp_time[1]
    # image_time = temp_time[0] + '_' + temp_time1_adj

    image_time = str(index) + '_' + str(frame_index)

    image_filename = "image_" + image_time + ".jpeg"

    image_size = len(msg_camera.data)
    image_width =  msg_camera.width
    image_height = msg_camera.height;
    image_channel =  image_size / (image_width * image_height);

    print('image name: {}  size: {}  width: {}  height: {}  channel: {}'.format(image_filename, image_size, image_width, image_height, image_channel))

    img_array = np.array((map(ord, msg_camera.data)), dtype = np.uint8 )
    img = img_array.reshape(image_height, image_width, image_channel )
    rgb_img=cv2.cvtColor(img,cv2.COLOR_BGR2RGB)

    cv2.imwrite(out_folder + image_filename, rgb_img)
    # f = open(out_folder + image_filename, 'w+b')
    # f.write(rgb_img)
    # f.close()

    return tstamp

###########################################################
