from jsonrpclib.SimpleJSONRPCServer import SimpleJSONRPCServer
from os.path import dirname, join, abspath
import sys
sys.path.append(dirname(abspath(__file__)))
sys.path.append(join(dirname(abspath(__file__)), 'yidong'))
from sence import *
from robot_api import *
from pos_obj_map import *
import numpy as np
import threading
from pos_obj_map import *
import math
import time
lock=threading.Lock()
flag=0
def pointmove(x, y):
    global flag
    flag=1
    lock.acquire()
    pos=(x,y)                                     
    car_pos=sence.car.get_2d_pose()
    path=getpath(int(round(car_pos[0])*2),int(round(car_pos[1])*2),int(pos[0]),int(pos[1]))
    sequence_move(sence,pathtrans(path))
    images=get_images()
    result={"state":1,"info":"pointmove success","images":images}
    lock.release()
    flag=0
    return result


def getpath(x,y):
    pos=(x,y)                                     
    car_pos=sence.car.get_2d_pose()
    path=getpath(int(round(car_pos[0])*2),int(round(car_pos[1])*2),int(pos[0]),int(pos[1]))
    return path

def placemove(x, y):
    global flag
    flag=1
    lock.acquire()
    try:
        print("moving to (",x,',',y,')')
        pos=(x,y)                                     
        car_pos=sence.car.get_2d_pose()
        path=getpath(int(round(car_pos[0])*2),int(round(car_pos[1])*2),int(pos[0]),int(pos[1]))
        sequence_move(sence,pathtrans(path))
        print("已到达(",x,',',y,')')
    except (KeyError):
        lock.release()
        flag=0
        return {"state":-1,"info":"无效的语义地点"}
    lock.release()
    flag=0
    images=get_images()
    result={"state":1,"info":"placemove success","images":images}
    return result

def pathmove(positions:list):
    global flag
    flag=1
    lock.acquire()
    sequence_move(sence,positions)
    lock.release()
    flag=0
    images=get_images()
    result={"state":1,"info":"pathmove success","images":images}
    return result




def grasp(eepos:list):
    global flag
    if(len(eepos)<3):
        return {"state":-1,"info":"fail, need 3D position"}
    flag=1
    lock.acquire()

    #开近
    car_pos=sence.car.get_2d_pose()[0:2]
    obj_pos=np.array(eepos[0:2])
    distance = math.sqrt(np.sum((obj_pos-car_pos)**2))
    limit=0.43
    if distance>limit:
        print("driving to object")
        angle = math.atan2(obj_pos[1]-car_pos[1], obj_pos[0] - car_pos[0])
        new_x = car_pos[0] + (distance-limit) * math.cos(angle)
        new_y = car_pos[1] + (distance-limit) * math.sin(angle)
        drive_to_position(sence,[new_x,new_y])



    #移臂
    try:
        print("moving arm to object")
        move_arm(sence,eepos[0:3],euler=[0,0,0])
    except Exception:
        print("fail, cant create armpath")
        lock.release()
        flag=0
        return {"state":-2,"info":"无法移动机械臂到指定的物体位置"}
    


    state=0
    #检测是否抓到
    for obj in sence.objects.values():
        if sence.gripper._proximity_sensor.is_detected(obj):
            print("成功检测到物体，开始抓取")
            try:
                graspload(sence,obj)
            except Exception:
                print("抓取失败, cant create armpath")
                lock.release()
                flag=0
                return {"state":-4,"info":"成功检测到物体，但抓取失败"}
            print("抓取成功")
            state=1
            break
    if(state==0):
        print("探头未检测到物体")
        target_pos=Dummy("youBot_target").get_position()
        move_arm(sence,target_pos,euler=[0,0,0])
    lock.release()
    flag=0
    if(state==0):
        return {"state":-3,"info":"fail, 探头未检测到物体"}
    else:
        images=get_images()
        result={"state":1,"info":"grasp success","images":images}
        return result



# def get_images():
#     global flag
#     flag=1
#     lock.acquire()
#     images=[]
#     for vision in sence.vision:      
#         image=vision.capture_rgb()
#         dimg=vision.capture_depth()
#         dimg=dimg.reshape(dimg.shape[0],-1,1)
#         image=np.concatenate((image, dimg), axis=2)
#         image=image.tolist()
#         images.append(image)
#     lock.release()
#     flag=0
#     result={"state":1,"info":"success","images":images}
#     return result
def get_images():
    global flag
    flag=1
    lock.acquire()
    images=[]
    for vision in sence.vision:      
        image=vision.capture_rgb().tolist()
        images.append(image)
    lock.release()
    flag=0
    return images

# def get_depth():
#     global flag
#     flag=1
#     lock.acquire()
#     depth=sence.vision[0].capture_depth().tolist()
#     result={"depth":depth}
#     lock.release()
#     flag=0   
#     return result

def get_depth():
    global flag
    flag=1
    lock.acquire()
    depth=[]
    for vision in sence.vision:      
        dep=vision.capture_depth().tolist()
        depth.append(dep)
    lock.release()
    flag=0   
    return depth


def get_image():
    global flag
    flag=1
    lock.acquire()
    image=sence.vision[0].capture_rgb().tolist()
    result={"image":image}
    lock.release()
    flag=0   
    return result

def get_vision_args():
    global flag
    flag=1
    lock.acquire()
    v1=sence.vision[0]
    a=v1.get_position().tolist()
    b=v1.get_orientation().tolist()
    c=v1.get_quaternion().tolist()
    d=v1.get_resolution()
    e=v1.get_perspective_angle()
    vision_args={"position":a,
        "orientation":b,
        "quaternion":c,
        "resolution":d,
        "angle":e}
    lock.release()
    flag=0
    return vision_args

def step(sence1):
    while(True):
        if(flag==0):
            sence1.pr.step()
        else:
            time.sleep(0.5)

if __name__ == '__main__':
    SCENE_FILE = join(dirname(abspath(__file__)), 'warehouse7.ttt')
    global sence
    sence=Sence(SCENE_FILE)   
    objnames=['scissor_black','cup_white','bowl_black','cube_green','cube_red']
    sence.set_objects(objnames)


    # graspload(sence,Shape("cube_white"))
    # grasp([+3.2253e+00,+2.9505e+00,+5.0152e-02])

    step_thread = threading.Thread(target=step,args=[sence])
    step_thread.start()




    json_rpc_server = SimpleJSONRPCServer(("localhost", 8000))
    json_rpc_server.register_function(pointmove,"pointmove")
    json_rpc_server.register_function(placemove,"placemove")
    json_rpc_server.register_function(pathmove,"pathmove")
    json_rpc_server.register_function(grasp,"grasp")
    json_rpc_server.register_function(get_images,"get_images")
    json_rpc_server.register_function(get_vision_args,"get_vision_args")
    json_rpc_server.register_function(get_depth,"get_depth")
    json_rpc_server.register_function(get_image,"get_image")
    json_rpc_server.serve_forever()












