"""
A Kuka youBot reaches for 4 randomly places targets.
This script contains examples of:
    - Linear mobile paths with an omnidirectional robot to reach a target.
"""
import matplotlib
import matplotlib.pyplot as plt
print(matplotlib.get_backend())
from os.path import dirname, join, abspath
import sys
sys.path.append(dirname(abspath(__file__)))
sys.path.append(join(dirname(abspath(__file__)), 'yidong'))
from sence import *
from robot_api import *
from pos_obj_map import *
print(matplotlib.get_backend())
import numpy as np
import time
import ast

import threading
import queue

# 定义一个全局队列，用于线程间通信
input_queue = queue.Queue()
SCENE_FILE = join(dirname(abspath(__file__)), 'warehouse5.ttt')
sence=Sence(SCENE_FILE)
# from rpc_api import servestart
# rpc_thread = threading.Thread(target=servestart,args=[sence])
# rpc_thread.start()

# print("here")
# servestart(sence)
# print("here")
for i in range(22):
    sence.pr.step()

# commend=input("input:")
#拿到场景中的物品
scissor=Shape("scissor_black")
scissor_pos=Dummy("Dummy_scissor").get_position()
drive_to_position(sence,scissor_pos)

# print("here")
# image=sence.vision[1].capture_rgb()
# print(image)
# plt.imshow(image)
# plt.axis('off')
# plt.show()

graspload(sence,scissor)
drive_to_position(sence,Dummy("huojia1").get_position())
target_pos=Dummy("objtarget_rack1").get_position()
graspput(sence,scissor,target_pos)
target_pos=Dummy("youBot_target").get_position()
move_arm(sence,target_pos,euler=[0,0,0])


def input_thread():
    print("输入语义地点")
    while True:
        user_input = input()  # 等待用户输入
        input_queue.put(user_input)   # 将用户输入放入队列
# 创建并启动线程
input_thread = threading.Thread(target=input_thread)
input_thread.start()

while(True):
    sence.pr.step()
    if not input_queue.empty():
        user_input = input_queue.get()  # 从队列中获取输入Step3
        # if user_input=="stage3":
        #     stage3=True
        try:
            res=user_input.split(',') 
        except (Exception):
            print("不是合法的字符串格式")            
        # 将获取的字符串转换为列表
        path2=[]
        objname=''
        try:
            for item in res:
                if(':' in item):
                    objname=item.split(':')[0]
                    objname=objmap[objname]
                else:
                    pos=positionmap[item]
                    print(pos)                                      
                    path2.append(pos)
        except (KeyError):
            print("输入地点无效，请重新输入！！")
        for pos in path2:
            car_pos=sence.car.get_2d_pose()
            path=getpath(int(round(car_pos[0])*2),int(round(car_pos[1])*2),int(pos[0]),int(pos[1]))
            sequence_move(sence,pathtrans(path))
        if objname!='':
            obj=Shape(objname)
            obj.set_model(True)
            graspload(sence,obj)

sence.pr.stop()
sence.pr.shutdown()