from astar import astar
from Map import Map
import numpy as np
from os.path import dirname, join, abspath

# mymap = [
#     [1,1,1,0,1,0,1,1,1,1,1,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,1,1,1,1],
#     [1,1,1,0,1,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1],
#     [1,1,1,1,1,0,1,1,1,1,1,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,1,1,1,1],
#     [1,1,1,0,1,0,1,1,1,1,1,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,1,1,1,1],
#     [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,1,1,1,1],
#     [1,1,1,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,1,1,1,1],
#     [1,1,1,0,1,0,1,1,1,1,1,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,1,1,1,1],
#     [1,1,1,0,1,0,1,1,1,1,1,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,1,1,1,1],
#     [1,1,1,1,1,0,1,1,1,1,1,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,1,1,1,1]
#     ]

def trans(result):
    out=[]
    for pos in result:
        out.append((pos[1],31-pos[0]))
    return out
def getpath(sx,sy,ex,ey):
    b = np.loadtxt(fname=join(dirname(abspath(__file__)),"data.csv"), dtype=int, delimiter=",")
    mymap = b.tolist()
    map = Map(mymap,sx,sy,ex,ey)
    result = astar(map)
    result = trans(result)
    result.reverse()
    print("path:",result)
    return result
# content = input()
