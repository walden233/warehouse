import math
'''
    对象Map，主要有地图数据、起点和终点
'''
class Map(object):
    def __init__(self,mapdata,startx,starty,endx,endy):
        self.data = mapdata
        self.startx = 31-starty
        self.starty = startx
        self.endx = 31-endy
        self.endy = endx