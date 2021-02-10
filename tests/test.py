import rospy
# import tf
# import numpy as np
# import quaternion
from panda_robot import PandaArm
# from franka_robot import franka_kinematics
# from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import WrenchStamped

FORCE = 'force'
TORQUE = 'torque'

if __name__ == '__main__':

    rospy.init_node('test')
    panda_arm = PandaArm()

    wrench1 = WrenchStamped()
    wrench1.header.frame_id = 'panda_link0'

    wrench_pub = rospy.Publisher('ee_wrench', WrenchStamped, queue_size=1)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if not panda_arm.in_safe_state():
            print(panda_arm.get_robot_status())
            if panda_arm.error_in_current_state():
                print(panda_arm.what_errors())
            break
        
        print('-----------------')
        print('')

        print('')
        wrench = panda_arm.tip_state()

        wrench1.header.stamp = rospy.Time.now()
        wrench1.wrench.force.x = wrench[FORCE][0]
        wrench1.wrench.force.y = wrench[FORCE][1]
        wrench1.wrench.force.z = wrench[FORCE][2]

        wrench1.wrench.torque.x = wrench[TORQUE][0]
        wrench1.wrench.torque.y = wrench[TORQUE][1]
        wrench1.wrench.torque.z = wrench[TORQUE][2]

        wrench_pub.publish(wrench1)

        print('-----------------')

        print('-----------------')
        print('-----------------')

        rate.sleep()
