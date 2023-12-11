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

class Sence():
    objects={}
    vision=[]
    def __init__(self,SCENE_FILE):
        self.pr=PyRep()
        self.pr.launch(SCENE_FILE, headless=False,responsive_ui=True)
        self.pr.start()
        self.car = YouBot_car()
        self.car.set_motor_locked_at_zero_velocity(True)
        self.car.set_base_angular_velocites([0.,0.,0.])
        self.arm= youBot_arm()
        self.gripper=BaxterSuctionCup()
        self.vision.append(VisionSensor("vision_down"))
        self.vision.append(VisionSensor("vision_front"))
        self.vision.append(VisionSensor("vision_left"))
        self.vision.append(VisionSensor("vision_right"))
        self.vision.append(VisionSensor("vision_back"))

    def set_objects(self,objnames:list):
        for name in objnames:
            obj_handle=Shape(name)
            self.objects[name]=obj_handle
