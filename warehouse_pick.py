"""
A Kuka youBot reaches for 4 randomly places targets.
This script contains examples of:
    - Linear mobile paths with an omnidirectional robot to reach a target.
"""
import matplotlib
import matplotlib.pyplot as plt
print(matplotlib.get_backend())
from os.path import dirname, join, abspath
from pyrep import PyRep
from pyrep.robots.mobiles.youbot import YouBot_car
from pyrep.robots.arms.youBot import youBot_arm
from pyrep.objects.shape import Shape
from pyrep.objects.dummy import Dummy
from pyrep.objects.vision_sensor import VisionSensor
from pyrep.const import PrimitiveShape
from pyrep.robots.end_effectors.baxter_gripper import BaxterGripper
from pyrep.robots.end_effectors.youbot_gripper import YouBotGripper
from pyrep.robots.end_effectors.baxter_suction_cup import BaxterSuctionCup
print(matplotlib.get_backend())
import sys
sys.path.append('C:\CoppeliaSim_PyRep\PyRep\warehouse\yidong')
from run import getpath
import numpy as np
import time
import ast

import threading
import queue

positionmap = {
    'baoan_xia': (15, 26),
    '保安区': (15, 26),
    '当前位置':None,
    'sanhuo_you': (10, 12),
    '散货区1': (10, 12),
    'sanhuo_zuo': (5, 12),
    '散货区2': (5, 12),
    'sanhuo_xia': (8, 9),
    '散货区3': (8, 9),
    'lingjian_shang': (17, 17),
    '零件区1': (17, 17),
    'lingjian_xia': (17, 14),
    '零件区2': (17, 14),
    'lingjian_zuo': (15, 16),
    '零件区3': (15, 16),

    'gongwei1': (15, 7),
    '工位1': (15, 7),

    'gongwei2': (17, 7),
    '工位2': (17, 7),

    'huojia1': (2, 10),
    '货架1': (2, 10),

    'huojia2': (2, 11),
    '货架2': (2, 11),

    'xiuxi1': (14, 20),
    '休息区1': (14, 20),

    'xiuxi2': (12, 20),
    '休息区2': (12, 20),
    "men":(19,29),
    "门":(19,29)
    }

objmap={"白色方块":"cube_white"}

# 定义一个全局队列，用于线程间通信
input_queue = queue.Queue()

SCENE_FILE = join(dirname(abspath(__file__)), 'warehouse5.ttt')
pr = PyRep()
pr.launch(SCENE_FILE, headless=False,responsive_ui=True)
pr.start()
#拿到场景中的物品
scissor=Shape("scissor_black")
car = YouBot_car()
car.set_motor_locked_at_zero_velocity(True)
car.set_base_angular_velocites([0.,0.,0.])
arm= youBot_arm()
gripper=BaxterSuctionCup()

vision_down=VisionSensor("vision_down")


# def positionmap(content1, content2):
#     if content1=='当前位置':
#         pos1 = np.array(car.get_2d_pose()[0:2])
#         pos2 = np.array(positionmap[content2])
#     else:
#         pos1 = np.array(positionmap[content1])
#         pos2 = np.array(positionmap[content2])
#     return pos1,pos2



def readpath(file_path):
    with open(file_path, 'r', encoding='utf-8') as file:
        content = file.read()
    import re
    pattern = r'\[[\(\d+, \d+\),? ?]+\]'
    matches = re.findall(pattern, content)
    # if(len[matches]==0):
    #     print("search path fail")
    #     return -1
    pattern2 = r'\(\d+, \d+\)'
    matches2 = re.findall(pattern2, matches[0])
    positions=[]
    for match in matches2:
        formatted_match = match.replace('(', '').replace(')', '')
        coordinates = tuple(map(int, formatted_match.split(',')))
        positions.append(coordinates)
    return positions
def pointtrans(point):
    return (point[0]/2,point[1]/2)
    
def pathtrans(positions:list):
    nppath=np.array(positions)
    mask=np.zeros(nppath.shape[0])
    mask[0]=1
    direction_old=nppath[1]-nppath[0]
    for i in range(1,nppath.shape[0]-1):
        direction_now=nppath[i+1]-nppath[i]
        if(not((direction_now==direction_old)[0] and (direction_now==direction_old)[1])):
            mask[i]=1
            direction_old=direction_now
    mask[nppath.shape[0]-1]=1
    pnew=[]
    for i in range(nppath.shape[0]):
        if(mask[i]):
            pnew.append(list(nppath[i]))
    return pnew

def drive_to_position(position):
    base_path = car.get_linear_path(position)
    base_path.visualize()
    done = False
    while not done:
        done = base_path.step()
        pr.step()
    base_path.clear_visualization()
    car.set_base_angular_velocites([0.,0.,0.])
def sequence_move(positions,realsacle=False):
    for pos in positions:
        print([pos[0]/2,pos[1]/2])
        drive_to_position([pos[0]/2,pos[1]/2])

def move_arm(position, quaternion=None,euler=None, ignore_collisions=True):

    car.set_base_angular_velocites([0.,0.,0.])
    arm_path = arm.get_path(position,
                            quaternion=quaternion,
                            euler=euler,
                            ignore_collisions=ignore_collisions)
    arm_path.visualize()
    #pr.stop()
    done = False
    while not done:
        done = arm_path.step()
        pr.step()
    arm_path.clear_visualization()

def graspload(obj):
    eepos=obj.get_pose()
    hightchu2=obj.get_model_bounding_box()[5]
    eepos[2]=eepos[2]+0.005
    #todo:自动求合适的欧拉角或四元数
    move_arm(eepos[0:3],euler=[0,0,0])
    obj.set_dynamic(False)
    obj.set_respondable(False)
    gripper.grasp(obj)
    print("grasp")
    target=Dummy("youBot_target").get_position()
    move_arm(target,euler=[0,0,0])
    obj.set_dynamic(True)
    obj.set_respondable(True)
    #car.set_model_dynamic(False)
    gripper.release()
    print('release')
    
def graspput(obj,position):   
    eepos=obj.get_pose()
    hightchu2=obj.get_model_bounding_box()[5]
    eepos[2]=eepos[2]+0.005
    #eepos[2]=eepos[2]-hightchu2
    move_arm(eepos[0:3],euler=[0,0,0])
    
    obj.set_dynamic(False)
    obj.set_respondable(False)
    gripper.grasp(obj)
    print("grasp")
    move_arm(position,euler=[0,0,0])
    obj.set_dynamic(True)
    obj.set_respondable(True)
    # car.set_model_dynamic(False)
    gripper.release()
    print('release')

for i in range(22):
    pr.step()

# commend=input("input:")
# scissor_pos=Dummy("Dummy_scissor").get_position()
# drive_to_position(scissor_pos)

# print("here")
# image=vision_down.capture_rgb()
# print(image)
# plt.imshow(image)
# plt.axis('off')
# plt.show()

# graspload(scissor)
# drive_to_position(Dummy("huojia1").get_position())
# target_pos=Dummy("objtarget_rack1").get_position()
# graspput(scissor,target_pos)
# target_pos=Dummy("youBot_target").get_position()
# move_arm(target_pos,euler=[0,0,0])


def input_thread():
    print("输入语义地点")
    while True:
        user_input = input()  # 等待用户输入
        input_queue.put(user_input)   # 将用户输入放入队列


# 创建并启动线程
input_thread = threading.Thread(target=input_thread)


input_thread.start()
# stage3=False
# for i in range(5000):
#     pr.step()
#     if not input_queue.empty():
#         user_input = input_queue.get()  # 从队列中获取输入Step3
#         if user_input=="stage3":
#             stage3=True
#         try:
#             res=user_input.split(',') 
#         except (Exception):
#             print("不是合法的字符串格式")            
#         # 将获取的字符串转换为列表
#         path2=[]
#         try:
#             for item in res:
#                 pos=positionmap[item]
#                 print(pos)
#                 path2.append(pos)
#         except (KeyError):
#             print("无效的语义地点")
#         for pos in path2:
#             car_pos=car.get_2d_pose()
#             path=getpath(int(round(car_pos[0])*2),int(round(car_pos[1])*2),int(pos[0]),int(pos[1]))
#             sequence_move(pathtrans(path))
#         # drive_to_position(pointtrans(pos))

while(True):
    pr.step()
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
            car_pos=car.get_2d_pose()
            path=getpath(int(round(car_pos[0])*2),int(round(car_pos[1])*2),int(pos[0]),int(pos[1]))
            sequence_move(pathtrans(path))
        if objname!='':
            obj=Shape(objname)
            obj.set_model(True)
            graspload(obj)
        # drive_to_position(pointtrans(pos))

# while(input_queue.empty()):
#     pass
# user_input = input_queue.get()
# res=user_input.split(',') 
# objt_pos=Dummy("lingjian_shang").get_position()
# drive_to_position(objt_pos)
# obj=Shape("cube_white")
# graspload(obj)

pr.stop()
pr.shutdown()