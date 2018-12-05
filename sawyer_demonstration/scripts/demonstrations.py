#!/usr/bin/env python

# the script to generate expert demonstrations

import argparse

import rospy

import intera_interface
import rospkg

from std_msgs.msg import String
from std_msgs.msg import Bool

from gazebo_msgs.srv import (
    SpawnModel,
    DeleteModel,
)
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from intera_core_msgs.msg import EndpointState

from intera_interface import CHECK_VERSION

import time
import datetime
import sys

from copy import deepcopy
import numpy as np
import csv

import random
from utils import CustomController
from utils import NovintFalcon
from sawyer_demonstration.srv import *
from ros_falcon.msg import falconForces
from gazebo_msgs.msg import ContactsState
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion

from gazebo_msgs.msg import ModelStates
import scipy.signal as signal


class Demonstration(object):
    def __init__(self, task):

        self._contact_data_file = open('/home/arun/data.csv', 'w')
        self._csv_writer = csv.writer(self._contact_data_file)
        self._csv_writer.writerow(['z'])

        self._limb = intera_interface.Limb('right')
        self._task = task
        if task == "reaching":
            self._pose, self._point, self._q = self._set_initial_pose_of_eef()
        elif task == "pih":
            self._pose_peg, self._point_peg, self._q_peg = self._set_initial_peg_pose()

        self._current_x = 0.0
        self._current_y = 0.0
        self._current_z = 0.0
        self._is_joystick = False
        self._is_falcon = True
        # self.joystick = CustomController()
        self.falcon = NovintFalcon()
        self._initial_setup()
        

        self.collision_data_buffer_in = []
        self.collision_data_buffer_out = []
        self.old_pose = 0.0
        self.new_pose = 0.0
        self.collided = False
        
        if self._task == "pih":
            self.peg_publisher = rospy.Publisher('/peg_position', Point, queue_size=10)
            # rospy.Subscriber('/right_gripper_r_finger_contact_sensor_state', ContactsState, self._right_gripper_cb)
            rospy.Subscriber('/peg_ft_contact_sensor_state', ContactsState, self._peg_ft_cb)
            rospy.Subscriber('/gazebo/model_states', ModelStates, self._model_states_cb)
        rospy.Subscriber('/right_gripper_r_finger_contact_sensor_state', ContactsState, self._right_gripper_cb)
        
        rospy.Subscriber('/robot/limb/right/endpnt_state', EndpointState,self._end_point_state_cb)
    

        

    def _initial_setup(self):

        if self._task == "reaching":
            self.move_to_neutral()
            # pass

        self._spawn_table()
        rospy.on_shutdown(self._delete_table)
         
        if self._task == "pih":
            self.spawn_peg_ft()
            rospy.on_shutdown(self._delete_peg_ft)
        
        

    def _set_initial_pose_of_eef(self):
        # End effector orientation is fixed.
        q = Quaternion()
        q.x = 0.642161760993
        q.y = 0.766569045326
        q.z = 0.000271775440016
        q.w = 0.00031241683589

        point = Point()
        pose = Pose()

        pose.position = point
        pose.orientation = q

        return pose, point, q

    def _set_initial_peg_pose(self):
        q = Quaternion()
        q.x = 0
        q.y = 0
        q.z = 0
        q.w = 1

        point = Point()
        pose = Pose()

        pose.position = point
        pose.orientation = q

        return pose, point, q


    def _spawn_table(self, table_pose=Pose(position=Point(x=0.55, y=0.0, z=0.0)),table_reference_frame="world"):
        # Get Models' Path
        model_path = rospkg.RosPack().get_path('sawyer_gazebo_env')+"/models/"
        # Load Table URDF
        table_xml = ''
        with open (model_path + "table/table.urdf", "r") as table_file:
            table_xml=table_file.read().replace('\n', '')
        # Spawn Table URDF
        rospy.wait_for_service('/gazebo/spawn_urdf_model')
        try:
            spawn_urdf = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
            resp_urdf = spawn_urdf("table", table_xml, "/",
                                 table_pose, table_reference_frame)
        except rospy.ServiceException, e:
            rospy.logerr("Spawn SDF service call failed: {0}".format(e))


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
                                   saucer_pose, "world")
        except rospy.ServiceException, e:
            rospy.logerr("Spawn URDF service call failed: {0}".format(e))


    #-------------------------------------------------------------------------------------------#
    def spawn_peg_ft(self):
        # peg_ft_pose=Pose(position=Point(x=0.60, y=0.15, z=0.835))
        peg_ft_pose=Pose(position=Point(x=0.60, y=0.15, z=0.63))
        # Get Models' Path
        model_path = rospkg.RosPack().get_path('sawyer_gazebo_env')+"/models/"
        
        # Load Peg URDF
        peg_ft_xml = ''
        with open (model_path + "peg_ft_coarse/peg_ft.urdf", "r") as peg_ft_file:
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

    def _delete_table(self):
        try:
            delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
            resp_delete = delete_model("table")
        except rospy.ServiceException, e:
            print("Delete Model service call failed: {0}".format(e))

    def _end_point_state_cb(self, data):
        self._current_x = data.pose.position.x
        self._current_y = data.pose.position.y
        self._current_z = data.pose.position.z

    def _right_gripper_cb(self, data):
        if data.states:
            self.collided = True
            force_obj = falconForces()
            force_obj.X = data.states[0].total_wrench.force.x
            force_obj.Y = data.states[0].total_wrench.force.y
            force_obj.Z = data.states[0].total_wrench.force.z
            
            # collision for the first time
            if not self.collided:
                self.old_pose = self.falcon._value_y
                self.new_pose = self.falcon._value_y
                self.collided = True
            # print self.old_pose - self.new_pose
            diff_force = 1000*(self.old_pose - self.new_pose)
            force_obj.Z = diff_force # -ve of -ve force
            self.new_pose = self.falcon._value_y
            # if len(self.collision_data_buffer_in) <= 100:
            #     self.collision_data_buffer_in.append(force_obj.Z)
            # else:
            #     out = signal.savgol_filter(self.collision_data_buffer_in, 51, 3)
            #     print out
            #     self.collision_data_buffer_in = []
            #     force_obj.Z = -(sum(out)/len(out))
            #     self.falcon._pub.publish(force_obj)
            # self.falcon._pub.publish(force_obj)

    def _peg_ft_cb(self, data):
        if data.states:
            self._csv_writer.writerow([data.states[1].total_wrench.force.z])
            
            force_obj = falconForces()
            force_obj.X = 0.0
            force_obj.Y = 0.0
            force_obj.Z = 0.63 - self._point_peg.z
            # force_obj.X = data.states[1].total_wrench.force.x
            # force_obj.Y = data.states[1].total_wrench.force.y
            # force_obj.Z = data.states[1].total_wrench.force.z
            # print force_obj.Z
            # self.falcon._pub.publish(force_obj)


    def _model_states_cb(self, data):

        if data:
            self._point_peg.x = data.pose[4].position.x 
            self._point_peg.y = data.pose[4].position.y
            self._point_peg.z = data.pose[4].position.z 
                 

    def move_to_neutral(self):
        
        # -J sawyer::right_j0 -0.27
        # -J sawyer::right_j1 1.05
        # -J sawyer::right_j2 0.00
        # -J sawyer::right_j3 0.49
        # -J sawyer::right_j4 -0.08
        # -J sawyer::right_j5 -0.06
        # -J sawyer::right_j6 0.027
        # -J sawyer::head_pan 0.00
        self._point.x = 0.4
        self._point.y = 0.0
        self._point.z = 0.0
        
        self._limb.move_to_joint_positions(self._limb.ik_request(self._pose))


    def joystick_control(self):

        if len(self.joystick._controls) > 0:
            # if lefstBumper button is clicked : Open the gripper.
            if self.joystick._controls['leftBumper'] and (not self.joystick.gripper_open):
                self.joystick.gripper.open()
                self.joystick.gripper_open = True
                time.sleep(0.05)
            if self.joystick._controls['leftBumper'] and self.joystick.gripper_open:
                self.joystick.gripper.close()
                self.joystick.gripper_open = False
                time.sleep(0.05)

            if self.joystick._controls['rightBumper']:
                # reset()
                self.move_to_neutral()
                print "reset"

            if self.joystick._controls['leftStickHorz'] < 0.0:
                self._point.y -= 0.005
                # print "right"
            elif self.joystick._controls['leftStickHorz'] > 0.0:
                self._point.y += 0.005
                # print "left"

            if self.joystick._controls['leftStickVert'] < 0.0:
                self._point.x -= 0.005
                # print "down"
            elif self.joystick._controls['leftStickVert'] > 0.0:
                self._point.x += 0.005
                # print "up"

            if self.joystick._controls['rightStickVert'] > 0.0:
                self._point.z -= 0.005
            elif self.joystick._controls['rightStickVert'] < 0.0:
                self._point.z += 0.005

    def falcon_control(self):
        if self.falcon._button == 4:

            if self._task == "reaching":
                # print round(self.falcon._value_x, 3), " ", round(self.falcon._value_y, 3), " ", round(self.falcon._value_z, 3)
                self._point.z += round(self.falcon._value_y, 3)/5.0
                self._point.y += round(self.falcon._value_x, 3)/5.0

                if round(self.falcon._value_z, 3)/5.0 < self.falcon._current_value_z:
                    self._point.x -= round(self.falcon._value_z, 3)/5.0
                    # point.x -= abs(falcon._current_value_z - (round(falcon._value_z, 3)/10.0))
                elif round(self.falcon._value_z, 3)/5.0 > self.falcon._current_value_z:
                    self._point.x += round(self.falcon._value_z, 3)/5.0
                    # point.x += abs(falcon._current_value_z - (round(falcon._value_z, 3)/10.0))

                self.falcon._current_value_z = round(self.falcon._value_z, 3)/5.0
            
            elif self._task == "pih":
                # print round(self.falcon._value_x, 3), " ", round(self.falcon._value_y, 3), " ", round(self.falcon._value_z, 3)
                self._point_peg.z += round(self.falcon._value_y, 3)/500.0
                self._point_peg.y += round(self.falcon._value_x, 3)/500.0

                if round(self.falcon._value_z, 3)/500.0 < self.falcon._current_value_z:
                    self._point_peg.x -= round(self.falcon._value_z, 3)/500.0
                    # point.x -= abs(falcon._current_value_z - (round(falcon._value_z, 3)/10.0))
                elif round(self.falcon._value_z, 3)/500.0 > self.falcon._current_value_z:
                    self._point_peg.x += round(self.falcon._value_z, 3)/500.0
                    # point.x += abs(falcon._current_value_z - (round(falcon._value_z, 3)/10.0))

                self.falcon._current_value_z = round(self.falcon._value_z, 3)/500.0
 

    def replay(self, file_name):

        def try_float(x):
            try:
                return float(x)
            except ValueError:
                return None

        src_file = open(file_name, 'rb')
        reader = csv.reader(src_file)
        reader.next()

        for row in reader:
            j0 = try_float(row[1])
            j1 = try_float(row[2])
            j2 = try_float(row[3])
            j3 = try_float(row[4])
            j4 = try_float(row[5])
            j5 = try_float(row[6])
            j6 = try_float(row[7])
            joint_positions = [j0, j1, j2, j3, j4, j5, j6]
            limb.set_joint_positions(joint_positions)


