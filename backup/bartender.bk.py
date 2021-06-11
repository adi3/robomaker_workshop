from interbotix_xs_modules.arm import InterbotixManipulatorXS
import numpy as np


# This script makes the end-effector perform pick, pour, and place tasks
#
# To get started, open a terminal and type 'roslaunch interbotix_xsarm_control xsarm_control.launch robot_model:=wx250'
# Then change to this directory and type 'python bartender.py'

X_OFFSET = -0.25

def main():
    x = -0.15
    x = x - X_OFFSET
    y = -0.05
    print x
    print y
    waist_angle = np.arctan2(y, x)
    print waist_angle


    bot = InterbotixManipulatorXS("px100", "arm", "gripper")
    # bot.arm.go_to_sleep_pose()
    bot.arm.go_to_home_pose()
    # bot.arm.set_ee_pose_components(x=0.3, z=0.05)
    bot.arm.set_single_joint_position("waist", waist_angle)
    # bot.gripper.open()
    # bot.arm.set_ee_cartesian_trajectory(x=0.1, z=-0.16)
    # bot.gripper.close()
    # bot.arm.go_to_home_pose()
    # bot.arm.go_to_sleep_pose()

    # x = -0.15, y = -.25
    return
    
    bot.arm.set_ee_cartesian_trajectory(x=-0.1, z=0.16)
    bot.arm.set_single_joint_position("waist", -np.pi/2.0)
    bot.arm.set_ee_cartesian_trajectory(pitch=1.5)
    bot.arm.set_ee_cartesian_trajectory(pitch=-1.5)
    bot.arm.set_single_joint_position("waist", np.pi/2.0)
    bot.arm.set_ee_cartesian_trajectory(x=0.1, z=-0.16)
    bot.gripper.open()
    bot.arm.set_ee_cartesian_trajectory(x=-0.1, z=0.16)
    bot.arm.go_to_home_pose()
    bot.arm.go_to_sleep_pose()

if __name__=='__main__':
    main()
