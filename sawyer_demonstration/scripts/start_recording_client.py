#!/usr/bin/env python

import sys
import rospy
from sawyer_demonstration.srv import *
from std_srvs.srv import Empty

def start_recording_client():
    rospy.wait_for_service('start_recording')
    try:
        start_recording = rospy.ServiceProxy('start_recording', StartRecording)
        resp1 = start_recording()
        return resp1
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

if __name__ == "__main__":

    print "Requesting start recording"
    print start_recording_client()