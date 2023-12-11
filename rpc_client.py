import jsonrpclib
import numpy as np
import matplotlib.pyplot as plt
from os.path import dirname, join, abspath
import sys
sys.path.append(dirname(abspath(__file__)))
from pos_obj_map import *
server = jsonrpclib.ServerProxy("http://localhost:8000")
""" 
result格式：
result["state"]：类型int，值为1表示执行成功，-1失败
result["info"]：类型str，执行结果提示
result["images"]：类型list，包含两张h*w*4的ndarray类型的rgbd图像
                  第一张是小车前下方视角，第二张是正前方视角
"""


#x,y:终点二维坐标
def pointmove(x, y):
    result=server.pointmove(x,y)
    print(result["info"])
    images=result["images"]
    imgs_array=[]
    for img in images:
        imgs_array.append(np.array(img))
    result["images"]=imgs_array
    return result


#place：目标地点名称，须在positionmap中有对应的坐标
def placemove(place:str):
    result=server.placemove(place)
    print(result["info"])
    if(result["state"]==1):
        images=result["images"]
        imgs_array=[]
        for img in images:
            imgs_array.append(np.array(img))
        result["images"]=imgs_array
    return result

#positions：移动坐标序列，示例：[(1,2),(2,4),(2,5),(6,6)]
def pathmove(positions:list):
    result=server.pathmove(positions)
    print(result["info"])
    images=result["images"]
    imgs_array=[]
    for img in images:
        imgs_array.append(np.array(img))
    result["images"]=imgs_array
    return result

#eepos：物体三维坐标，示例：[1.2,5.6,7.8]
#注：目前只能移动机械臂，不能进行抓取
def grasp(eepos:list):
    result=server.grasp(eepos)
    print(result["info"])
    if(result["state"]==1):
        images=result["images"]
        imgs_array=[]
        for img in images:
            imgs_array.append(np.array(img))
        result["images"]=imgs_array
    return result

#获取深度摄像头图像
def get_images():
    result=server.get_images()
    print(result["info"])
    images=result["images"]
    imgs_array=[]
    for img in images:
        imgs_array.append(np.array(img))
    result["images"]=imgs_array
    return result

# point=[+8.4753e+00,+8.1255e+00,+11.51e-02]
# server.grasp(point)
# print(server.pointmove(16,13))
# image=server.get_images()
# print(image)
if __name__=="__main__":
    result=placemove("sanhuo_you")
    # result=get_images()
    # print(result["images"][0].shape)
    # point=[+8.4753e+00,+8.1255e+00,+11.51e-02]
    # grasp(point)
    # result=pointmove(14,16)
    # plt.subplot(1, 2, 1)
    # plt.imshow(result["images"][0])
    # plt.subplot(1, 2, 2)
    # plt.imshow(result["images"][1])
    # plt.axis('off')
    # plt.show()