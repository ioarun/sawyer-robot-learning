#!/usr/bin/env python
# peg in hole task script
import rospy
from demonstrations import Demonstration
import random
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)

from gazebo_msgs.srv import (
    SpawnModel,
    DeleteModel,
)
import rospkg
import time
from sawyer_demonstration.srv import StartRecording
from sawyer_demonstration.srv import StopRecording
######################################
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Twist
######################################

class PegInHole(object):
    def __init__(self, controller):
        self.rate = rospy.Rate(10)
        self.demo = Demonstration("pih")
        self.controller = controller
        #######################
        self.set_peg_ft_pose_pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=10)
        #######################


    def control(self):
        if self.controller == "joystick":
            self.demo.joystick_control()
        elif self.controller == "falcon":
            self.demo.falcon_control()

    def run(self):

        while not rospy.is_shutdown():
            self.control()
            if self.demo._limb.ik_request(self.demo._pose) != False:
                self.demo._limb.set_joint_positions(self.demo._limb.ik_request(self.demo._pose))
            else:
                print "IK Request failed."
