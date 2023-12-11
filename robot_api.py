import math
import numpy as np
from sence import Sence
from pyrep.objects.dummy import Dummy
from yidong.run import getpath

#从文本读取坐标序列，暂时不用
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

#将密集坐标序列提取为精简坐标序列
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

#移动，采用实际坐标(浮点数，和coppelia环境对应)
def drive_to_position(sence:Sence,position,angle=0):
    base=sence.car.get_2d_pose()[0:2]
    target=np.array(position[0:2])
    angle = math.atan2(target[1]-base[1], target[0] - base[0])-math.pi/2
    base_path = sence.car.get_linear_path(position,angle)
    base_path.visualize()
    done = False
    while not done:
        done = base_path.step()
        sence.pr.step()
    base_path.clear_visualization()
    sence.car.set_base_angular_velocites([0.,0.,0.])
    
#移动，采用抽象坐标(整数，和俯视图对应，实际坐标*2)
def sequence_move(sence:Sence,positions,realsacle=False):
    for pos in positions:
        p=[pos[0]/2,pos[1]/2]
        print(p)
        drive_to_position(sence,p)


def move_arm(sence:Sence,position, quaternion=None,euler=None, ignore_collisions=True):

    sence.car.set_base_angular_velocites([0.,0.,0.])
    arm_path = sence.arm.get_path(position,
                            quaternion=quaternion,
                            euler=euler,
                            ignore_collisions=ignore_collisions)
    arm_path.visualize()
    #pr.stop()
    done = False
    while not done:
        done = arm_path.step()
        sence.pr.step()
    arm_path.clear_visualization()

def graspload(sence:Sence,obj):
    if(not sence.gripper._proximity_sensor.is_detected(obj)):
        eepos=obj.get_pose()
        hightchu2=obj.get_model_bounding_box()[5]
        eepos[2]=eepos[2]+0.005
        #todo:自动求合适的欧拉角或四元数
        move_arm(sence,eepos[0:3],euler=[0,0,0])
    obj.set_dynamic(False)
    obj.set_respondable(False)
    sence.gripper.grasp(obj)
    target=Dummy("youBot_target").get_position()
    move_arm(sence,target,euler=[0,0,0])
    obj.set_dynamic(True)
    obj.set_respondable(True)
    #car.set_model_dynamic(False)
    sence.gripper.release()
    
def graspput(sence:Sence,obj,position):   
    eepos=obj.get_pose()
    hightchu2=obj.get_model_bounding_box()[5]
    eepos[2]=eepos[2]+0.005
    #eepos[2]=eepos[2]-hightchu2
    move_arm(sence,eepos[0:3],euler=[0,0,0])
    
    obj.set_dynamic(False)
    obj.set_respondable(False)
    sence.gripper.grasp(obj)
    print("grasp")
    move_arm(sence,position,euler=[0,0,0])
    obj.set_dynamic(True)
    obj.set_respondable(True)
    # car.set_model_dynamic(False)
    sence.gripper.release()


def car_rotate(sence:Sence,angle):   
    base=sence.car.get_2d_pose()[0:2]
    rotation= sence.car.get_orientation()
    print(rotation)
    angle=rotation[2]+angle
    if(angle<0):
        angle+=2*math.pi
    if(angle>=2*math.pi):
        angle-=2*math.pi
    base_path = sence.car.get_linear_path(base,angle)
    base_path.visualize()
    done = False
    while not done:
        done = base_path.step()
        sence.pr.step()
    base_path.clear_visualization()
    sence.car.set_base_angular_velocites([0.,0.,0.])


# def positionmap(content1, content2):
#     if content1=='当前位置':
#         pos1 = np.array(sence.car.get_2d_pose()[0:2])
#         pos2 = np.array(positionmap[content2])
#     else:
#         pos1 = np.array(positionmap[content1])
#         pos2 = np.array(positionmap[content2])
#     return pos1,pos2