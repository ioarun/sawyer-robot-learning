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

rospy.init_node('demonstrator_node')


limb = intera_interface.Limb('right')
from geometry_msgs.msg import Pose
pose = Pose()
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
point = Point()

# End effector orientation is fixed.
q = Quaternion()
q.x = 0.642161760993
q.y = 0.766569045326
q.z = 0.000271775440016
q.w = 0.00031241683589

pose.position = point
pose.orientation = q

current_x = 0.0
current_y = 0.0
current_z = 0.0

reset = False

def reset():
    delete_gazebo_models()
    x = (random.uniform(0.5, 0.7))
    y = (random.uniform(-0.4, 0.4))
    spawn_cube(x, y)
    reset = True


def spawn_table(table_pose=Pose(position=Point(x=0.75, y=0.0, z=0.0)),table_reference_frame="world"):
    # Get Models' Path
    model_path = rospkg.RosPack().get_path('sawyer_sim_examples')+"/models/"
    # Load Table SDF
    table_xml = ''
    with open (model_path + "cafe_table/model.sdf", "r") as table_file:
        table_xml=table_file.read().replace('\n', '')
    # Spawn Table SDF
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    try:
        spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        resp_sdf = spawn_sdf("cafe_table", table_xml, "/",
                             table_pose, table_reference_frame)
    except rospy.ServiceException, e:
        rospy.logerr("Spawn SDF service call failed: {0}".format(e))


def spawn_cube(_x, _y, block_reference_frame="world"):
    # block_pose=Pose(position=Point(x=0.4225, y=0.1265, z=0.7725))
    block_pose=Pose(position=Point(x=_x, y=_y, z=0.7725))
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
    saucer_pose=Pose(position=Point(x=_x, y=_y, z=0.7725))
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

def delete_table():
    try:
        delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        resp_delete = delete_model("cafe_table")
    except rospy.ServiceException, e:
        print("Delete Model service call failed: {0}".format(e))

def delete_cube():
    try:
        delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        resp_delete = delete_model("block")
    except rospy.ServiceException, e:
        print("Delete Model service call failed: {0}".format(e))

def delete_saucer():
    try:
        delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        resp_delete = delete_model("plate")
    except rospy.ServiceException, e:
        print("Delete Model service call failed: {0}".format(e))


def move_to_neutral():

    point.x = 0.4
    point.y = 0.0
    point.z = 0.1
    
    limb.move_to_joint_positions(limb.ik_request(pose))

def ping_from_the_recorder_start_cb(data):
    print "recorder pings :", data


def ping_from_the_recorder_stop_cb(data):
    print "recorder pings :", data

def end_point_state_cb(data):
    current_x = data.pose.position.x
    current_y = data.pose.position.y
    current_z = data.pose.position.z

def recorder_cb(msg):
    data = msg.data
    start_recording = data

def joystick_control():
    if len(joystick._controls) > 0:
        # if leftBumper button is clicked : Open the gripper.
        if joystick._controls['leftBumper'] and (not joystick.gripper_open):
            joystick.gripper.open()
            joystick.gripper_open = True
            time.sleep(0.05)
        if joystick._controls['leftBumper'] and joystick.gripper_open:
            joystick.gripper.close()
            joystick.gripper_open = False
            time.sleep(0.05)

        if joystick._controls['rightBumper']:
            # reset()
            move_to_neutral()
            print "reset"

        if joystick._controls['leftStickHorz'] < 0.0:
            point.y -= 0.005
            # print "right"
        elif joystick._controls['leftStickHorz'] > 0.0:
            point.y += 0.005
            # print "left"

        if joystick._controls['leftStickVert'] < 0.0:
            point.x -= 0.005
            # print "down"
        elif joystick._controls['leftStickVert'] > 0.0:
            point.x += 0.005
            # print "up"

        if joystick._controls['rightStickVert'] > 0.0:
            point.z -= 0.005
        elif joystick._controls['rightStickVert'] < 0.0:
            point.z += 0.005

def falcon_control():
    if falcon._button == 4:
        print round(falcon._value_x, 3), " ", round(falcon._value_y, 3), " ", round(falcon._value_z, 3)
        point.z += round(falcon._value_y, 3)/10.0
        point.y += round(falcon._value_x, 3)/10.0

        if round(falcon._value_z, 3)/10.0 < falcon._current_value_z:
            point.x -= round(falcon._value_z, 3)/10.0
            # point.x -= abs(falcon._current_value_z - (round(falcon._value_z, 3)/10.0))
        elif round(falcon._value_z, 3)/10.0 > falcon._current_value_z:
            point.x += round(falcon._value_z, 3)/10.0
            # point.x += abs(falcon._current_value_z - (round(falcon._value_z, 3)/10.0))

        falcon._current_value_z = round(falcon._value_z, 3)/10.0


def try_float(x):
    try:
        return float(x)
    except ValueError:
        return None


def replay(file_name):
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


is_joystick = False
is_falcon = True
joystick = CustomController()
falcon = NovintFalcon()


def main():
    rospy.Subscriber('/robot/limb/right/endpnt_state', EndpointState, end_point_state_cb)

    random.seed(1)

    move_to_neutral()

    spawn_table()
    rospy.on_shutdown(delete_table)
    rospy.on_shutdown(delete_cube)
    
    while not rospy.is_shutdown():
        # move_to_neutral()
        # x = (random.uniform(0.5, 0.7))
        # y = (random.uniform(-0.4, 0.4))
        # point.x = x
        # point.y = y
        # point.z = 0.05
        # spawn_cube(x, y)
        # time.sleep(0.5)

        if is_joystick:
            joystick_control()
        if is_falcon:
            falcon_control()
        
        if limb.ik_request(pose) != False:
            # rospy.wait_for_service('start_recording')
            # try:
            #     start_recording = rospy.ServiceProxy('start_recording', StartRecording)
            #     resp1 = start_recording()
            #     # return resp1
            #     print "resp1"
            # except rospy.ServiceException, e:
            #     print "Service call failed: %s"%e

            limb.move_to_joint_positions(limb.ik_request(pose))


            # rospy.wait_for_service('stop_recording')
            # try:
            #     stop_recording = rospy.ServiceProxy('stop_recording', StopRecording)
            #     resp2 = stop_recording()
            #     # return resp2
            #     print "resp2"
            # except rospy.ServiceException, e:
            #     print "Service call failed: %s"%e 
        else:
            print "IK Request failed."
        
        delete_cube()

    

if __name__ == '__main__':
    main()

    
