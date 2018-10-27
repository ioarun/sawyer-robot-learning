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

class PegInHole(object):
    def __init__(self, controller):
        self.demo = Demonstration()
        self.controller = controller
        self.reset()
        rospy.on_shutdown(self._delete_peg_ft)
        rospy.on_shutdown(self._delete_hole)

    def reset(self):
        
        self.demo.move_to_neutral()

        # use this when spawning objects
        # at random positions
        # x = random.uniform(0.5, 0.7) # 0.5, 0.7
        # y = random.uniform(-0.4, 0.4)

        self.spawn_hole()
        self.spawn_peg_ft()

#-------------------------------------------------------------------------------------------#
    def spawn_peg_ft(self):
        peg_ft_pose=Pose(position=Point(x=0.60, y=-0.05, z=0.630))
        # Get Models' Path
        model_path = rospkg.RosPack().get_path('sawyer_gazebo_env')+"/models/"
        
        # Load Peg URDF
        peg_ft_xml = ''
        with open (model_path + "peg_ft/peg_ft.urdf", "r") as peg_ft_file:
            peg_ft_xml=peg_ft_file.read().replace('\n', '')

        # Spawn Peg URDF
        rospy.wait_for_service('/gazebo/spawn_urdf_model')
        try:
            spawn_urdf = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
            resp_urdf = spawn_urdf("peg_ft", peg_ft_xml, "/",
                                   peg_ft_pose, "world")
        except rospy.ServiceException, e:
            rospy.logerr("Spawn URDF service call failed: {0}".format(e))
#--------------------------------------------------------------------------------------------#
    def spawn_hole(self):
        hole_pose=Pose(position=Point(x=0.60, y=0.05, z=0.630))
        # Get Models' Path
        model_path = rospkg.RosPack().get_path('sawyer_gazebo_env')+"/models/"
        
        # Load Hole URDF
        hole_xml = ''
        with open (model_path + "hole/hole.urdf", "r") as hole_file:
            hole_xml=hole_file.read().replace('\n', '')

        # Spawn Hole URDF
        rospy.wait_for_service('/gazebo/spawn_urdf_model')
        try:
            spawn_urdf = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
            resp_urdf = spawn_urdf("hole", hole_xml, "/", hole_pose, "world")
        except rospy.ServiceException, e:
            rospy.logerr("Spawn URDF service call failed: {0}".format(e))
#---------------------------------------------------------------------------------------------#            
    def _delete_hole(self):
        try:
            delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
            resp_delete = delete_model("hole")
        except rospy.ServiceException, e:
            print("Delete Model service call failed: {0}".format(e))
#---------------------------------------------------------------------------------------------#
    def _delete_peg_ft(self):
        try:
            delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
            resp_delete = delete_model("peg_ft")
        except rospy.ServiceException, e:
            print("Delete Model service call failed: {0}".format(e))
#---------------------------------------------------------------------------------------------#
    def control(self):
        if self.controller == "joystick":
            self.demo.joystick_control()
        elif self.controller == "falcon":
            self.demo.falcon_control()

    def run(self):

        while not rospy.is_shutdown():
            # do stuffs here
            self.control()
