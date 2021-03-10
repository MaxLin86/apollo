#!/usr/bin/env python

###############################################################################
# Copyright 2019 The Apollo Authors. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
###############################################################################

"""
print received perception message
"""
import argparse
from cyber_py import cyber
import time
from modules.perception.proto.perception_obstacle_pb2 import PerceptionObstacles
from modules.drivers.proto.conti_radar_pb2 import ContiRadar
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.lines import Line2D
from matplotlib.animation import FFMpegWriter
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg,NavigationToolbar2TkAgg
import threading
import Tkinter as tk
from Tkinter import *

left_corner_obj_list = []
front_left_obj_list = []
front_obj_list = []
front_right_obj_list = []
right_corner_obj_list = []

def receiver(data):
    """receiver"""
    
    mutex.acquire()
    global cnt
    global radar_id
    global double_obj_list
    global left_corner_obj_list
    global front_left_obj_list
    global front_obj_list
    global front_right_obj_list
    global right_corner_obj_list
    cnt = 0
    radar_id = data.radar_id
    #print("data.radar_id:")
    #print(str(data.radar_id))
    del double_obj_list[:]
    for index in range(len(data.contiobs)):
      double_obj_list.append([data.contiobs[index].pos_y,data.contiobs[index].pos_x])
      #print(data.contiobs[index].pos_y)
    #print(double_obj_list)
    if radar_id == 2:
       del right_corner_obj_list[:]
       right_corner_obj_list = list(double_obj_list)
    elif radar_id == 1:
       del front_right_obj_list[:]
       front_right_obj_list = list(double_obj_list)
    elif radar_id == 0:
       del front_obj_list[:]
       front_obj_list = list(double_obj_list)
    elif radar_id == -1:
       del front_left_obj_list[:]
       front_left_obj_list = list(double_obj_list)
    elif radar_id == -2:
       del left_corner_obj_list[:]
       left_corner_obj_list = list(double_obj_list)
    else:
       time.sleep(0.001)
    mutex.release()
    #time.sleep(10)
def perception_receiver(perception_channel):
    """publisher"""
    cyber.init()
    node = cyber.Node("perception")
    node.create_reader(perception_channel, ContiRadar, receiver)
    node.spin()
def left_corner_select():
    time.sleep(0.001)
def draw(id, times):
    global double_obj_list
    global cnt
    global radar_id
    global left_corner_obj_list
    global front_left_obj_list
    global front_obj_list
    global front_right_obj_list
    global right_corner_obj_list
    left_corner_is_select = 0
    front_left_is_select = 0
    front_is_select = 0
    front_right_is_select = 0
    right_corner_is_select = 0
    frame = tk.Tk()
    #canvas = tk.Canvas()
    fig = plt.figure(figsize=(16,12),dpi=80)
    ax = fig.add_subplot(1,1,1)
    fig.canvas.set_window_title("rada data")
    canvas = FigureCanvasTkAgg(fig, frame)
    canvas.draw()
    canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH,expand=1)
    toolbar=NavigationToolbar2TkAgg(canvas,frame)
    toolbar.update()
    canvas._tkcanvas.pack(side=tk.TOP, fill=tk.BOTH,expand=1)
    
    CheckVar1 = tk.IntVar(value=1)
    C1 = tk.Checkbutton(frame,text="left_corner",variable=CheckVar1,command=left_corner_select,onvalue=1,offvalue=0,height=1,width=10)
    C1.place(x=230,y=900,anchor=NW)
    CheckVar2 = tk.IntVar(value=1)
    C2 = tk.Checkbutton(frame,text="front_left",variable=CheckVar2,command=left_corner_select,onvalue=1,offvalue=0,height=1,width=10)
    C2.place(x=420,y=900)
    CheckVar3 = tk.IntVar(value=1)
    C3 = tk.Checkbutton(frame,text="front",variable=CheckVar3,command=left_corner_select,onvalue=1,offvalue=0,height=1,width=10)
    C3.place(x=580,y=900)
    CheckVar4 = tk.IntVar(value=1)
    C4 = tk.Checkbutton(frame,text="front_right",variable=CheckVar4,command=left_corner_select,onvalue=1,offvalue=0,height=1,width=10)
    C4.place(x=730,y=900)
    CheckVar5 = tk.IntVar(value=1)
    C5 = tk.Checkbutton(frame,text="right_corner",variable=CheckVar5,command=left_corner_select,onvalue=1,offvalue=0,height=1,width=10)
    C5.place(x=930,y=900)
    while cnt < 10:
        cnt+=1
        if(CheckVar1.get() == 1):
            left_corner_is_select = 1
        else:
            left_corner_is_select = 0
        if(CheckVar2.get() == 1):
            front_left_is_select = 1
        else:
            front_left_is_select = 0
        if(CheckVar3.get() == 1):
            front_is_select = 1
        else:
            front_is_select = 0
        if(CheckVar4.get() == 1):
            front_right_is_select = 1
        else:
            front_right_is_select = 0
        if(CheckVar5.get() == 1):
            right_corner_is_select = 1
        else:
            right_corner_is_select = 0
        ax.cla()
        #ax = plt.subplot(1,1,1,figsize=(6.4,4.8))
        #fig = ax.figure
        #plt.figure(figsize=(6.4,4.8))
        ax.set_title("rada data")
        ax.spines['bottom'].set_position(('data', 0))
        car_pos = (1.3, -1.5, -2.6, 6.0)
        plt.xlim((50, -50))
        plt.ylim((-30, 100))
        plt.xticks(np.linspace(50, -50, 11))
        plt.yticks(np.linspace(-30, 120, 16))
        plt.xlabel("Y axis")
        plt.ylabel("X axis")
        plt.grid(linewidth=0.2, alpha=0.5)
        ax.add_patch(plt.Rectangle(xy=(car_pos[0], car_pos[1]),
                                  width=car_pos[2], 
                                  height=car_pos[3],
                                  color='r',
                                  fill=True, linewidth=2))
        left_corner_xy = (car_pos[0]+40, car_pos[1]-25)
        front_left_xy = (car_pos[0]+20, car_pos[1]-25)
        front_xy = (car_pos[0]+2, car_pos[1]-25)
        front_right_xy = (car_pos[0]-10, car_pos[1]-25)
        right_corner_xy = (car_pos[0]-30, car_pos[1]-25)
        ax.add_patch(plt.Circle(left_corner_xy,
                                  radius=0.5, 
                                  color='k'))
        ax.add_patch(plt.Circle(front_left_xy,
                                  radius=0.5,
                                  color='r'))
        ax.add_patch(plt.Circle(front_xy,
                                  radius=0.5,
                                  color='g'))
        ax.add_patch(plt.Circle(front_right_xy,
                                  radius=0.5,
                                  color='c'))
        ax.add_patch(plt.Circle(right_corner_xy,
                                  radius=0.5,
                                  color='b'))
        ax.text(left_corner_xy[0]-2, left_corner_xy[1], "left_corner",size = 10)
        ax.text(front_left_xy[0]-2, front_left_xy[1], "front_left",size = 10)
        ax.text(front_xy[0]-2, front_xy[1], "front",size = 10)
        ax.text(front_right_xy[0]-2, front_right_xy[1], "front_right",size = 10)
        ax.text(right_corner_xy[0]-2, right_corner_xy[1], "right_corner",size = 10)
        #print(radar_id)
        if radar_id == 2:
             color = 'k'
        elif radar_id == 1:
             color = 'r'
        elif radar_id == 0:
             color = 'g'
        elif radar_id == -1:
             color = 'c'
        elif radar_id == -2:
             color = 'b'
        else:
             color = 'w'
        mutex.acquire()
        if right_corner_is_select == 1:
            for index in range(len(left_corner_obj_list)):
                ax.scatter(left_corner_obj_list[index][0],left_corner_obj_list[index][1],
                           c='',marker='o',edgecolors='b')
        if front_right_is_select == 1:
            for index in range(len(front_left_obj_list)):
                ax.scatter(front_left_obj_list[index][0],front_left_obj_list[index][1],
                  c='',marker='o',edgecolors='c')
        if front_is_select == 1:
            for index in range(len(front_obj_list)):
                ax.scatter(front_obj_list[index][0],front_obj_list[index][1],
                  c='',marker='o',edgecolors='g')
        if front_left_is_select == 1:
            for index in range(len(front_right_obj_list)):
                ax.scatter(front_right_obj_list[index][0],front_right_obj_list[index][1],
                  c='',marker='o',edgecolors='r')
        if left_corner_is_select == 1:
            for index in range(len(right_corner_obj_list)):
                ax.scatter(right_corner_obj_list[index][0],right_corner_obj_list[index][1],
                  color='',marker='o',edgecolors='k')
        mutex.release()
        global num
        num += 1 
        canvas.draw()
        frame.update_idletasks()
        frame.update()
        time.sleep(0.01)
        #frame.mainloop()
        #plt.savefig(str(num)+".jpg")
        #plt.pause(0.000001)
    
if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="create fake perception obstacles",
                                     prog="show_perception.py")
    parser.add_argument("-c", "--channel", action="store", type=str,
                        default="/apollo/sensor/radar/front",
                        #default="/apollo/perception/obstacles",
                        help="set the perception channel")

    args = parser.parse_args()
    num = 0
    cnt = 0
    radar_id = 0
    double_obj_list = []
    mutex = threading.Lock()
    t = threading.Thread(target=draw, args=("hawk", 5))
    t.start()
    perception_receiver(args.channel)
    
