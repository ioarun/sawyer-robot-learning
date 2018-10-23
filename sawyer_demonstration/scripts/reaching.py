#!/usr/bin/env python
# reaching task script
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

class ReachingTask(object):
    def __init__(self, controller):
        self.demo = Demonstration()
        self.controller = controller
        # initial setup for reaching task
        self._initial_setup()

    def _initial_setup(self):
        random.seed(1)
        self.spawn_cube(random.uniform(0.5, 0.7), random.uniform(-0.4, 0.4)) # spawn_cube(x, y)
        # self.spawn_saucer(random.uniform(0.5, 0.7), random.uniform(-0.4, 0.4)) # spawn_saucer(x, y)
        rospy.on_shutdown(self._delete_cube)
        # rospy.on_shutdown(self._delete_saucer)

    def spawn_cube(self, _x, _y, block_reference_frame="world"):
        # block_pose=Pose(position=Point(x=0.4225, y=0.1265, z=0.7725))
        block_pose=Pose(position=Point(x=_x, y=_y, z=0.825))
        # Get Models' Path
        model_path = rospkg.RosPack().get_path('sawyer_sim_examples')+"/models/"
        
        # Load Block URDF
        block_xml = ''
        with open (model_path + "block/model.urdf", "r") as block_file:
            block_xml=block_file.read().replace('\n', '')

        # Spawn Block URDF
        rospy.wait_for_service('/gazebo/spawn_urdf_model')
        try:
            spawn_urdf = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
            resp_urdf = spawn_urdf("block", block_xml, "/",
                                   block_pose, block_reference_frame)
        except rospy.ServiceException, e:
            rospy.logerr("Spawn URDF service call failed: {0}".format(e))

    def spawn_saucer(_x, _y, saucer_reference_frame="world"):
        # block_pose=Pose(position=Point(x=0.4225, y=0.1265, z=0.7725))
        saucer_pose=Pose(position=Point(x=_x, y=_y, z=0.825))
        # Get Models' Path
        model_path = rospkg.RosPack().get_path('sawyer_gazebo_env')+"/models/"
        
        # Load Saucer URDF
        saucer_xml = ''
        with open (model_path + "plate/plate.urdf", "r") as saucer_file:
            saucer_xml=saucer_file.read().replace('\n', '')

        # Spawn Saucer URDF
        rospy.wait_for_service('/gazebo/spawn_urdf_model')
        try:
            spawn_urdf = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
            resp_urdf = spawn_urdf("plate", saucer_xml, "/",
                                   saucer_pose, saucer_reference_frame)
        except rospy.ServiceException, e:
            rospy.logerr("Spawn URDF service call failed: {0}".format(e))


    def _delete_cube(self):
        try:
            delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
            resp_delete = delete_model("block")
        except rospy.ServiceException, e:
            print("Delete Model service call failed: {0}".format(e))

    def _delete_saucer(self):
        try:
            delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
            resp_delete = delete_model("plate")
        except rospy.ServiceException, e:
            print("Delete Model service call failed: {0}".format(e))

    def control(self):
        if self.controller == "joystick":
            self.demo.joystick_control()
        elif self.controller == "falcon":
            self.demo.falcon_control()

    def run(self):

        while not rospy.is_shutdown():

            self.control()

            if self.demo._limb.ik_request(self.demo._pose) != False:
                self.demo._limb.move_to_joint_positions(self.demo._limb.ik_request(self.demo._pose))
            else:
                print "IK Request failed."
