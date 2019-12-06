#!/usr/bin/env python
"""  A test node to move the end effector to the desired pose

 SUBSCRIBERS:
  +
  +

PUBLISHERS:
  + /ee_wrench(geometry_msgs/WrenchStamped)
  +

"""

import rospy

from geometry_msgs.msg import Pose, WrenchStamped

import intera_interface

from trajectory_generation import TrajGen

class Move2DesiredPose(object):

    def __init__(self):
        # initializing the member variables
        self.wrenchPublisher = rospy.Publisher("/ee_wrench", WrenchStamped, queue_size=10)
        self.wrenchMsg = WrenchStamped()

        self._limb = intera_interface.Limb("right")
        self._gripper = intera_interface.Gripper()

        self.initial_t = rospy.Time.now().to_sec()

        self.targetEePose = Pose()
        self.desiredEePose = Pose()
        self.initialPose = self._limb.endpoint_pose()

        self.maneuver_time = 4.0;

        self.targetEePose.position.x = 0.0 + self.initialPose['position'].x
        self.targetEePose.position.y = 0.0 + self.initialPose['position'].y
        self.targetEePose.position.z = -0.0 + self.initialPose['position'].z


        self.targetEePose.orientation.x = 1.0
        self.targetEePose.orientation.y = 0.0
        self.targetEePose.orientation.z = 0.0
        self.targetEePose.orientation.w = 0.0

        self.desiredEEForce = 7.5
        self.desiredZposInRightHand = 0.0
        self.kP = 60.0/10000000;
        
        #open
        self.gripper_open()
        #self.gripper_close()
    def advance(self):
        # joint commands are sent in this function
        self.currentPose = self._limb.endpoint_pose()
       # self.desiredEePose.position.z = self.initialPose['position'].z + s * (self.targetEePose.position.z - self.initialPose['position'].z)
        #self.desiredEePose.position.z = self.currentPose['position'].z - self.desiredZposInRightHand # force control
        self.desiredEePose.position.z = self.currentPose['position'].z # force control

        self.desiredEePose.orientation = self.targetEePose.orientation
        
        #Test
        print("Received pose: ", self.desiredEePose.position)


        joint_angles = self._limb.ik_request(self.desiredEePose)
        self._limb.set_joint_positions(joint_angles)

    def forceControl(self):
        # desired z position relative to the end effector is calculated with a simple P controller.
        self.currentEffort = self._limb.endpoint_effort()
        self.desiredZposInRightHand = self.desiredZposInRightHand + self.kP * (self.desiredEEForce - self.currentEffort['force'].z)

    def gripper_open(self):
        self._gripper.open()
        rospy.sleep(1.0)

    def gripper_close(self):
        self._gripper.close()
        rospy.sleep(1.0)

    def publishWrench(self):
        # publish WrecnhStamped for visualization purposes in Rviz
        self.currentEffort = self._limb.endpoint_effort()

        self.wrenchMsg.header.frame_id = "right_hand"
        self.wrenchMsg.wrench.force.x = self.currentEffort['force'].x
        self.wrenchMsg.wrench.force.y = self.currentEffort['force'].y
        self.wrenchMsg.wrench.force.z = self.currentEffort['force'].z
        self.wrenchMsg.wrench.torque.x = self.currentEffort['torque'].x
        self.wrenchMsg.wrench.torque.y = self.currentEffort['torque'].y
        self.wrenchMsg.wrench.torque.z = self.currentEffort['torque'].z

        self.wrenchPublisher.publish(self.wrenchMsg)

def main():
    """ The main() function. """
    rospy.init_node('move_to_desired_pose', anonymous=True)
    eePoseController = Move2DesiredPose()
    # eePoseController.gripper_close()
    # eePoseController.gripper_open()
    trajgen = TrajGen()

    while not rospy.is_shutdown():
        trajgen.update_trajectory_status()
        [eePoseController.desiredEePose.position.x, eePoseController.desiredEePose.position.y] = trajgen.get_xy()
        draw_status = trajgen.get_draw_status() #TODO: use this for force control
        #eePoseController.forceControl()
        eePoseController.advance()
        
        #testing 
        #currentPose = trajgen._limb.endpoint_pose()
       # [x,y,z] = [currentPose['position'].x, currentPose['position'].y, currentPose['position'].z]
       # print "x, y, z = ", x,"  ", y,"  ",z

        eePoseController.publishWrench()
        pass

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
