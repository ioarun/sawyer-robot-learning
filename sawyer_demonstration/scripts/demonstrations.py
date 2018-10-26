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
from joystick import CustomController
from novint_falcon_joystick import NovintFalcon
from sawyer_demonstration.srv import *
from ros_falcon.msg import falconForces
from gazebo_msgs.msg import ContactsState
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion


class Demonstration(object):
    def __init__(self):
        self._limb = intera_interface.Limb('right')
        self._pose, self._point, self._q = self._set_initial_pose_of_eef()
        self._current_x = 0.0
        self._current_y = 0.0
        self._current_z = 0.0
        self._is_joystick = False
        self._is_falcon = True
        self.joystick = CustomController()
        self.falcon = NovintFalcon()
        rospy.Subscriber('/robot/limb/right/endpnt_state', EndpointState,self._end_point_state_cb)
        rospy.Subscriber('/right_gripper_r_finger_contact_sensor_state', ContactsState, self._right_gripper_cb)
        
        self._initial_setup()

    def _initial_setup(self):
        self.move_to_neutral()
        self._spawn_table()
        rospy.on_shutdown(self._delete_table)

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

    # def _spawn_table(self, table_pose=Pose(position=Point(x=0.75, y=0.0, z=0.0)),table_reference_frame="world"):
    #     # Get Models' Path
    #     model_path = rospkg.RosPack().get_path('sawyer_sim_examples')+"/models/"
    #     # Load Table SDF
    #     table_xml = ''
    #     with open (model_path + "cafe_table/model.sdf", "r") as table_file:
    #         table_xml=table_file.read().replace('\n', '')
    #     # Spawn Table SDF
    #     rospy.wait_for_service('/gazebo/spawn_sdf_model')
    #     try:
    #         spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    #         resp_sdf = spawn_sdf("cafe_table", table_xml, "/",
    #                              table_pose, table_reference_frame)
    #     except rospy.ServiceException, e:
    #         rospy.logerr("Spawn SDF service call failed: {0}".format(e))


    def _spawn_table(self, table_pose=Pose(position=Point(x=0.75, y=0.0, z=0.0)),table_reference_frame="world"):
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
                                   saucer_pose, saucer_reference_frame)
        except rospy.ServiceException, e:
            rospy.logerr("Spawn URDF service call failed: {0}".format(e))

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
            
            force_obj = falconForces()
            force_obj.X = data.states[0].total_wrench.force.x
            force_obj.Y = data.states[0].total_wrench.force.y
            force_obj.Z = data.states[0].total_wrench.force.z
            self.falcon._pub.publish(force_obj)

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
        self._point.z = 0.15
        
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
            print round(self.falcon._value_x, 3), " ", round(self.falcon._value_y, 3), " ", round(self.falcon._value_z, 3)
            self._point.z += round(self.falcon._value_y, 3)/10.0
            self._point.y += round(self.falcon._value_x, 3)/10.0

            if round(self.falcon._value_z, 3)/10.0 < self.falcon._current_value_z:
                self._point.x -= round(self.falcon._value_z, 3)/10.0
                # point.x -= abs(falcon._current_value_z - (round(falcon._value_z, 3)/10.0))
            elif round(self.falcon._value_z, 3)/10.0 > self.falcon._current_value_z:
                self._point.x += round(self.falcon._value_z, 3)/10.0
                # point.x += abs(falcon._current_value_z - (round(falcon._value_z, 3)/10.0))

            self.falcon._current_value_z = round(self.falcon._value_z, 3)/10.0


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


