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
import time
from sawyer_demonstration.srv import StartRecording
from sawyer_demonstration.srv import StopRecording

class ReachingTask(object):
    def __init__(self, controller):
        self.demo = Demonstration("reaching")
        self.controller = controller
        self.auto = False
        self.delete_model_cube = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        self.delete_model_plate = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        self.delete_model_sphere = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        self.delete_model_semi = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        random.seed(random.randint(1, 100000))
        rospy.on_shutdown(self.delete_objects)
    

    def reset(self):
        x, y = self.random_spawn()
       
        time.sleep(1)
        print x, y
        self.demo.move_to_neutral()
        self.demo._point.x = x
        self.demo._point.y = y
        self.demo._point.z = 0.0 # by default, z = 0.0
    

    def random_spawn(self):
        x = round(random.uniform(0.5, 0.8), 3) # 0.5, 0.7
        y = round(random.uniform(-0.4, 0.4), 3)
        self.spawn_cube(x, y)

        x1 = round(random.uniform(0.5, 0.8), 3) # 0.5, 0.7
        y1 = round(random.uniform(-0.4, 0.4), 3)
        self.spawn_saucer(x1, y1)

        x1 = round(random.uniform(0.5, 0.8), 3) # 0.5, 0.7
        y1 = round(random.uniform(-0.4, 0.4), 3)
        self.spawn_sphere(x1, y1)

        x1 = round(random.uniform(0.5, 0.8), 3) # 0.5, 0.7
        y1 = round(random.uniform(-0.4, 0.4), 3)
        self.spawn_semicircle(x1, y1)

        return x, y


    def spawn_cube(self, _x, _y, block_reference_frame="base"):
        # block_pose=Pose(position=Point(x=0.4225, y=0.1265, z=0.7725))
        block_pose=Pose(position=Point(x=_x, y=_y, z=-0.250))
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

    def spawn_saucer(self, _x, _y, saucer_reference_frame="base"):
        # block_pose=Pose(position=Point(x=0.4225, y=0.1265, z=0.7725))
        saucer_pose=Pose(position=Point(x=_x, y=_y, z=-0.250))
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

    def spawn_sphere(self, _x, _y, sphere_reference_frame="base"):
        # block_pose=Pose(position=Point(x=0.4225, y=0.1265, z=0.7725))
        sphere_pose=Pose(position=Point(x=_x, y=_y, z=-0.250))
        # Get Models' Path
        model_path = rospkg.RosPack().get_path('sawyer_gazebo_env')+"/models/"
        
        # Load Saucer URDF
        sphere_xml = ''
        with open (model_path + "sphere/sphere.urdf", "r") as sphere_file:
            sphere_xml=sphere_file.read().replace('\n', '')

        # Spawn Saucer URDF
        rospy.wait_for_service('/gazebo/spawn_urdf_model')
        try:
            spawn_urdf = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
            resp_urdf = spawn_urdf("sphere", sphere_xml, "/",
                                   sphere_pose, sphere_reference_frame)
        except rospy.ServiceException, e:
            rospy.logerr("Spawn URDF service call failed: {0}".format(e))


    def spawn_semicircle(self, _x, _y, semi_reference_frame="base"):
        # block_pose=Pose(position=Point(x=0.4225, y=0.1265, z=0.7725))
        semi_pose=Pose(position=Point(x=_x, y=_y, z=-0.250))
        # Get Models' Path
        model_path = rospkg.RosPack().get_path('sawyer_gazebo_env')+"/models/"
        
        # Load Saucer URDF
        semi_xml = ''
        with open (model_path + "semicircle/semicircle.urdf", "r") as semi_file:
            semi_xml=semi_file.read().replace('\n', '')

        # Spawn Saucer URDF
        rospy.wait_for_service('/gazebo/spawn_urdf_model')
        try:
            spawn_urdf = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
            resp_urdf = spawn_urdf("semicircle", semi_xml, "/",
                                   semi_pose, semi_reference_frame)
        except rospy.ServiceException, e:
            rospy.logerr("Spawn URDF service call failed: {0}".format(e))


    def delete_objects(self):
        try:
            rospy.wait_for_service('/gazebo/delete_model')
            self.delete_model_cube("block")
            rospy.wait_for_service('/gazebo/delete_model')
            self.delete_model_plate("plate")
            rospy.wait_for_service('/gazebo/delete_model')
            self.delete_model_sphere("sphere")
            rospy.wait_for_service('/gazebo/delete_model')
            self.delete_model_semi("semicircle")
        except rospy.ServiceProxy, e:
            print("Delete Model service call failed: {0}".format(e))


    def control(self):
        if self.controller == "joystick":
            self.demo.joystick_control()
        elif self.controller == "falcon":
            self.demo.falcon_control()

    def run(self):

        while not rospy.is_shutdown():
            if self.auto:

           
                self.reset()

                if self.demo._limb.ik_request(self.demo._pose) != False:
                    # rospy.wait_for_service('start_recording')
                    # try:
                    #     start_recording = rospy.ServiceProxy('start_recording', StartRecording)
                    #     resp1 = start_recording()
                    #     print "started"
                    # except rospy.ServiceException, e:
                    #     print "Service call failed: %s"%e
                    # time.sleep(1)
                    self.demo._limb.move_to_joint_positions(self.demo._limb.ik_request(self.demo._pose))
                    
                    # rospy.wait_for_service('stop_recording')
                    
                    # try: 
                    #     stop_recording = rospy.ServiceProxy('stop_recording', StopRecording)
                    #     resp2 = stop_recording()
                    #     print "stopped"
                    # except rospy.ServiceException, e:
                    #     print "Service call failed: %s"%e
                else:
                    print "IK Request failed."

                self.delete_objects()
                

            else:
                self.control()
            
                if self.demo._limb.ik_request(self.demo._pose) != False:
                    self.demo._limb.move_to_joint_positions(self.demo._limb.ik_request(self.demo._pose))
                else:
                    print "IK Request failed."

