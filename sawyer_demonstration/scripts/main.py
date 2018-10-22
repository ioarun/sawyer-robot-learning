#!/usr/bin/env python

import rospy
import reaching
import argparse
from reaching import ReachingTask

def main():
    rospy.init_node('demonstration_node')

    epilog = """
    Usage : roslaunch sawyer_demonstration sawyer_demonstration.launch \
    task:=reaching controller:=falcon

    """
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__,
                                     epilog=epilog)
    required = parser.add_argument_group('required arguments')
    required.add_argument(
        '-t', '--task', dest='task', required=True,
        help='the file name to record to'
    )

    required.add_argument(
        '-c', '--controller', dest='controller', required=True,
        help='the file name to record to'
    )

    args = parser.parse_args(rospy.myargv()[1:])
    task = args.task
    controller = args.controller

    if task == "reaching":
        current_task = ReachingTask(controller)
    elif task == "peg-in-hole":
        pass # for now

    current_task.run()

if __name__ == '__main__':
    main()