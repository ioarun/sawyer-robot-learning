import rospy

import intera_interface

from intera_interface import CHECK_VERSION

from std_msgs.msg import String
from std_msgs.msg import Bool

class JointRecorder(object):
    def __init__(self, rate, side="right"):
        """
        Records joint data to a file at a specified rate.
        """
        self.gripper_name = '_'.join([side, 'gripper'])
        self._raw_rate = rate
        self._rate = rospy.Rate(rate)
        self._start_time = rospy.get_time()
        self._done = True
        self.counter = 0
        self.is_recording = False
        self.filename = str(self.counter)+'.csv'

        self._limb_right = intera_interface.Limb(side)
        try:
            self._gripper = intera_interface.Gripper(self.gripper_name)
            rospy.loginfo("Electric gripper detected.")
        except Exception as e:
            self._gripper = None
            rospy.loginfo("No electric gripper detected.")

        # Verify Gripper Have No Errors and are Calibrated
        if self._gripper:
            if self._gripper.has_error():
                self._gripper.reboot()
            if not self._gripper.is_calibrated():
                self._gripper.calibrate()

        self._cuff = intera_interface.Cuff(side)

    def _time_stamp(self):
        return rospy.get_time() - self._start_time

    def stop(self):
        self._done = True

    def done(self, flag):
        self._done = flag
        return self._done    

    def record(self):
        """
        Records the current joint positions to a csv file if outputFilename was
        provided at construction this function will record the latest set of
        joint angles in a csv format.

        If a file exists, the function will overwrite existing file.
        """
        # if self.filename:
        joints_right = self._limb_right.joint_names()
        with open(self.filename, 'w') as f:
            f.write('time,')
            temp_str = '' if self._gripper else '\n'
            f.write(','.join([j for j in joints_right]) + ',' + temp_str)
            if self._gripper:
                f.write(self.gripper_name+'\n')
            while not self._done:
                if self._gripper:
                    if self._cuff.upper_button():
                        self._gripper.open()
                    elif self._cuff.lower_button():
                        self._gripper.close()
                angles_right = [self._limb_right.joint_angle(j)
                                for j in joints_right]
                f.write("%f," % (self._time_stamp(),))
                f.write(','.join([str(x) for x in angles_right]) + ',' + temp_str)
                if self._gripper:
                    f.write(str(self._gripper.get_position()) + '\n')

                self._rate.sleep()
           

def demo_subscriber_cb(msg):
    print msg
    # if data == "True" and not jr.is_recording:
    #     print "started recording"
    #     jr.is_recording = True
    #     # start recording
    #     jr.done(False)
    # elif data == "False" and jr.is_recording:
    #     print "done recording"
    #     jr.is_recording = False
    #     self.counter += 1
    #     filename = str(self.counter)+'.csv'
    #     self.filename = filename
    #     jr.done(True)
    if msg.data == True:
        print "start it"
        # send back the command to begin the demonstration
        data = Bool()
        data.data = True
        recorder_publisher.publish(data)
        jr._done = False
    elif msg.data == False:
        print "stop it"
        jr._done = True
        jr.counter += 1
        filename = str(jr.counter)+'.csv'
        jr.filename = filename


rospy.init_node('joint_recorder_node')
jr = JointRecorder(100) # frequency of recordin
demo_subscriber = rospy.Subscriber('/demonstrator/message/start', Bool, demo_subscriber_cb)
recorder_publisher = rospy.Publisher('/recorder/message/start', Bool, queue_size=10)
while not rospy.is_shutdown():
    jr.record()