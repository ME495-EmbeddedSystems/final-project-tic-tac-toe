 #!/usr/bin/env python

import intera_interface
import rospy
import numpy as np
from geometry_msgs.msg import Pose

def QuinticTimeScaling( Tf, t):
    """Computes s(t) for a quintic time scaling

    :param Tf: Total time of the motion in seconds from rest to rest
    :param t: The current time t satisfying 0 < t < Tf
    :return: The path parameter s(t) corresponding to a fifth-order
             polynomial motion that begins and ends at zero velocity and zero
             acceleration

   """
    if t > Tf:
        t = Tf
 

    return 10 * (1.0 * t / Tf) ** 3 - 15 * (1.0 * t / Tf) ** 4 \
           + 6 * (1.0 * t / Tf) ** 5


time_gap = 0.001
Tf = 8

class TrajGen(object):
    def __init__(self):
        global time_gap, Tf        
        self._limb = intera_interface.Limb("right")
        
        self.pstart = self.update_current_pose()
        self.pend = []
        self.update_goal_pose(self.pstart[0]+0.1,self.pstart[1],self.pstart[2])
        self.initial_t = rospy.Time.now().to_sec()  
        self. t = time_gap
    def update_current_pose(self):
        '''
        Returns the current pose in an array
        '''
        current_pose = self._limb.endpoint_pose()
        pstart = [current_pose['position'].x , current_pose['position'].y,current_pose['position'].z,current_pose['orientation'].x, current_pose['orientation'].y, current_pose['orientation'].z, current_pose['orientation'].w ]
        return pstart 

    def update_goal_pose(self,x,y,z_temp=None):
        '''
        Update the goal of the next trip.
        Input: x, y, z locations of the new goal pose, which is vertical to the plane
        Outputs:[cartesian x,y,z coordinates, quaternion x,y,z,w]
        '''
        
        z = z_temp if z_temp!=None else self.pstart[2] #pstart is z
        self.pend = [x,y,z,1,0,0,0] 

    def get_next_pose(self):
        """
        calculates the pose of the next point, with each time estimate being 0.002s
        input: none.
        Outputs: waypoint (Pose)
        """
        t = rospy.Time.now().to_sec() - self.initial_t
        self.pstart = self.update_current_pose()
        s = QuinticTimeScaling(Tf, t)
        coord = s * np.array(self.pend) + (1 - s) * np.array(self.pstart)
        waypoint = Pose()
        waypoint.position.x = coord[0]
        waypoint.position.y = coord[1]
        waypoint.position.z = coord[2]
        waypoint.orientation.x = 1
        waypoint.orientation.y = 0
        waypoint.orientation.z = 0
        waypoint.orientation.w = 0
        return waypoint

        
    
def main():
    rospy.init_node("sawyer")
    trajgen = TrajGen()
    for i in range(10):
       waypoint =  trajgen.get_next_pose()
       print waypoint

if __name__=='__main__':
    main()
