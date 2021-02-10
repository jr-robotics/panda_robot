from panda_robot import PandaArm
import rospy
# from franka_tools import FrankaFramesInterface
# import tf
import numpy as np
# import quaternion

if __name__ == '__main__':

    rospy.init_node('test')
    panda_arm = PandaArm(reset_frames=False)

    joint_angles = panda_arm.joint_angles()

    joint_names = panda_arm.joint_names()

    joint_values = [0.000, -0.785, 0.0, -1.9,
            0.0, 1.57, 0.785]

    def convert_to_dict(values):
        retval = {}
        for n, name in enumerate(joint_names):
            retval[name] = values[n]
        return retval

    def send_to(values):
        panda_arm.move_to_joint_positions(convert_to_dict(values))

        print('err: {}'.format(np.asarray(values) -
                               np.asarray(panda_arm.joint_ordered_angles())))

    cm = panda_arm.get_controller_manager()

    cms = cm.stop_controller
    cmstart = cm.start_controller
