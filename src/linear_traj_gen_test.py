#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose, Point
import rospkg
import sys
import intera_interface
from linear_traj_gen import TrajGen

from gazebo_msgs.srv import (
    SpawnModel,
    DeleteModel,
)


class rico_ik(object):
    def __init__(self, limb = "right"):
        self._limb = intera_interface.Limb("right")
        self.trajgen = TrajGen()
        self._gripper = intera_interface.Gripper()
        #this block of code gives me trouble: 
#        self._rs = intera_interface.RobotEnable(intera_interface.CHECK_VERSION)
#        self._init_state = self._rs.state().enabled
#        print("Enabling robot... ")
#        self._rs.enable()
       
        current_pose = self._limb.endpoint_pose()
        Tf = 1.0  #1s 
        N =200
        self.delta_t = Tf/N
        self.initial_pose = Pose()
        self.initial_pose.position.x = current_pose['position'].x 
        self.initial_pose.position.y = current_pose['position'].y
        self.initial_pose.position.z = current_pose['position'].z
        self.initial_pose.orientation.x = current_pose['orientation'].x
        self.initial_pose.orientation.y = current_pose['orientation'].y
        self.initial_pose.orientation.z = current_pose['orientation'].z
        self.initial_pose.orientation.w = current_pose['orientation'].w
        self.target_pose = self.initial_pose

        self.target_pose.position.x += 0.05
        #self.gripper_open()
        self.gripper_close()



        self.target_pose.orientation.x = 1
        self.target_pose.orientation.y = 0
        self.target_pose.orientation.z = 0
        self.target_pose.orientation.w = 0
        self.trajgen.update_goal_pose(self.target_pose.position.x, self.target_pose.position.y, self.target_pose.position.z)


    def go(self):
#        joint_angles = self._limb.ik_request(self.traj[-1])
#        if joint_angles:
#            self._limb.set_joint_positions(joint_angles)
#            print "limb set successful"
#        else:
        waypoint = self.trajgen.get_next_pose()
        joint_angles = self._limb.ik_request(waypoint)
        if joint_angles:
            self._limb.set_joint_positions(joint_angles)
            print "goal set"
        else:
            print "goal not achievable"

    def gripper_open(self):
        self._gripper.open()
        rospy.sleep(1.0)

    def gripper_close(self):
        self._gripper.close()
        rospy.sleep(1.0)

def main():
    rospy.init_node("rico_ik")
    ik = rico_ik()
    while not rospy.is_shutdown():
        ik.go()


if __name__ == '__main__':
    sys.exit(main())

