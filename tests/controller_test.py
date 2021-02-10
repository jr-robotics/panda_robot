import rospy
from panda_robot import PandaArm
import numpy as np
# import matplotlib.pyplot as plt
# from std_msgs.msg import Float64
from copy import deepcopy

from rospy.timer import sleep

joint_names = ['panda_joint1', 'panda_joint2', 'panda_joint3',
               'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7']


def _print_values(joint_values, delta, count):
    if count % 500 == 0:
        print('values: {} -- delta {}'.format(joint_values, delta))
        print ('\n ----  \n')
        print (' ')
    count += 1


if __name__ == '__main__':

    rospy.init_node('test_node')
    panda_arm = PandaArm()

    rate = rospy.Rate(400)

    elapsed_time_ = rospy.Duration(0.0)
    period = rospy.Duration(0.005)

    panda_arm.untuck()  # move to neutral pose before beginning

    initial_pose = deepcopy(panda_arm.joint_ordered_angles())

    # raw_input('Hit Enter to Start')
    sleep(5)
    print('commanding')
    joint_values = deepcopy(initial_pose)
    count = 0

    while not rospy.is_shutdown():

        elapsed_time_ += period

        delta = 3.14 / 16.0 * \
            (1 - np.cos(3.14 / 5.0 * elapsed_time_.to_sec())) * 0.2

        for joint, _ in enumerate(joint_values):
            if joint == 4:
                joint_values[joint] = initial_pose[joint] - delta
            else:
                joint_values[joint] = initial_pose[joint] + delta

        _print_values(joint_values, delta, count)

        panda_arm.set_joint_positions_velocities(
            joint_values, [0.0 for _ in range(7)])  # for impedance control
        # r.set_joint_positions(vals) # try this for position control

        rate.sleep()
