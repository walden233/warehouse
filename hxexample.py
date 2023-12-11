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
import queue
lock=threading.Lock()
input_queue = queue.Queue()
output_queue = []

def pointmove(x, y):
    input_queue.put((1,[x/2,y/2]))
    result=output_queue[0].get()
    return result

    lock.acquire()
    drive_to_position(sence,[x/2,y/2])
    lock.release()
    return "pointmove success"

def pathmove(positions:list):
    input_queue.put((2,positions))
    result=output_queue[1].get()
    return result

    sequence_move(sence,positions)
    return "pathmove success"


def grasp(eepos:list):
    input_queue.put((3,eepos))
    result=output_queue[2].get()
    return result

    if(len(eepos)<3):
        return "need 3D position"
    lock.acquire()
    move_arm(sence,eepos[0:3],euler=[0,0,0])
    obj=sence.gripper._proximity_sensor.get_detected_object()
    if(obj):
        graspload(sence,obj)
    else:
        return "cant detected object"
    lock.release()
    return "grasp success"

def get_images():
    input_queue.put((4,-1))
    result=output_queue[3].get()
    return result

    lock.acquire()
    image=sence.vision[0].capture_rgb()
    lock.release()
    return image


def servestart():
    json_rpc_server = SimpleJSONRPCServer(("localhost", 8000))
    json_rpc_server.register_function(pointmove,"pointmove")
    json_rpc_server.register_function(pathmove,"pathmove")
    json_rpc_server.register_function(grasp,"grasp")
    json_rpc_server.register_function(get_images,"get_images")
    json_rpc_server.serve_forever()

if __name__ == '__main__':
    for i in range(4):
        output_queue=queue.Queue()

    SCENE_FILE = join(dirname(abspath(__file__)), 'warehouse5.ttt')
    sence=Sence(SCENE_FILE)
    rpc_thread = threading.Thread(target=servestart)
    rpc_thread.start()

    while(True):
        sence.pr.step()
        if not input_queue.empty():
            index,arg=input_queue.get()
            if(index==1):
                drive_to_position(sence,arg)
            elif(index==2):
                sequence_move(sence,arg)
            elif(index==3):
                if(len(arg)<3):
                    continue
                move_arm(sence,arg[0:3],euler=[0,0,0])
                obj=sence.gripper._proximity_sensor.get_detected_object()
                if(obj):
                    graspload(sence,obj)
            elif(index==4):
                image=sence.vision[0].capture_rgb()
            
            
