#!/usr/bin/env python

import intera_interface
import rospy
import numpy as np

def InsideDistThresh(pos_start, pos_end, DIST_THRE):
    #Checks if two points' x,y position difference is larger than DIST_THRE
    if np.linalg.norm( pos_start - pos_end ) > DIST_THRE:
        return False
    else:
        return True


TF = 2
DIST_THRE = 0.02
LINE_LENGTH = 0.2
HOLD_TIME = 5.0

class TrajGen(object):
    def __init__(self):
        global TF, DIST_THRE, LINE_LENGTH, HOLD_TIME
        self._limb = intera_interface.Limb("right")
        self.center = np.array([])
        self.target_list = []
        self.draw_status_list = []
        self.hold_init_time = None
        self.line_init_time = None

        self.current_target = np.array([])
        self.current_draw_status = False
        self.if_hold = 0 #0 is not to hold, -1 is not to hold but just get back from hold. 1 is to hold
        self.line_init_pose = None
        
        self.object_2_draw = "idle"
        self.setup_idle_params()

        #Test
        current_pose = self.update_current_pose()
        self.center = np.array([current_pose[0]+0.1, current_pose[1]+0.1])
        self.object_2_draw = "cross"
        self.setup_cross_params()
                
    def update_trajectory_status(self):
        '''
        Updates trajectory variable status, every time it is called in the force control loop. 
        '''
        if self.if_hold != 1:
            
            if self.object_2_draw == "cross":
                
                current_pose = self.update_current_pose()
                if InsideDistThresh( current_pose, self.current_target, DIST_THRE ):

                    #Last point of action has not been reached yet
                    if len( self.target_list )!=0:

                        # If we should hold (go up or down)
                        if (self.if_hold == 0) and np.array_equal(self.target_list[0], self.current_target):
                            print "current target: ", self.current_target, "| target_list: ", self.target_list
                            self.if_hold = 1
                            self.hold_init_time = rospy.Time.now().to_sec()
                            self.line_init_time = rospy.Time.now().to_sec()
                            self.line_init_pose = self.update_current_pose()

                        # If we need to travel horizontally
                        else:
                            self.if_hold = 0 #when if_hold = -1
                            self.line_init_time = rospy.Time.now().to_sec()
                        
                        self.current_target = self.target_list[0]
                        self.target_list.pop(0)
                        self.current_draw_status = self.draw_status_list[0]
                        self.draw_status_list.pop(0)
                        self.line_init_pose = current_pose
                        
                    #last point of action has been reached
                    else:

                        #generate object_2_draw and ceter
                        self.get_next_object_center()
                        
                        #if the next action is cross
                        if self.object_2_draw == "cross":
                            self.setup_cross_params()                       

                        #if the next action is idle
                        elif self.object_2_draw == "idle": 
                            self.setup_idle_params()

                        #if the next action is circle
                        elif self.object_2_draw == "circle":
                            #TODO
                            pass

            # if the current object to draw is idling
            elif self.object_2_draw == "idle":
                #generate object_2_draw and ceter
                self.get_next_object_center()
                
                #if the next action is cross
                if self.object_2_draw == "cross":
                    self.setup_cross_params()                       

                #if the next action is idle
                elif self.object_2_draw == "idle": 
                    self.setup_idle_params()

                #if the next action is circle
                elif self.object_2_draw == "circle":
                    #TODO
                    pass
            
            #if the current object to draw is cirlce
            elif self.object_2_draw == "circle":
                pass
                #TODO
        
        #if we are holding (move up or down)
        else:
            if rospy.Time.now().to_sec() - self.hold_init_time > HOLD_TIME:
                print "hold time is up"
                self.if_hold = -1,
                print "current goal", self.current_target, " | target list: ", self.target_list

#                d = rospy.Duration(3, 0)
#                rospy.sleep(d)
 


    def get_next_object_center(self):
        #call AI function to get center, object to draw

        #Test
        if self.object_2_draw == "cross":
            self.object_2_draw = "idle"


    def setup_cross_params(self):
        '''
        Set up params for generating a cross, after updating the center and object_to_draw
        '''
        self.generate_cross_targets_draws()
        self.current_target = self.target_list[0]
        self.target_list.pop(0)
        
        self.current_draw_status = self.draw_status_list[0]
        self.draw_status_list.pop(0)

        self.line_init_time = rospy.Time.now().to_sec()
        self.line_init_pose = self.update_current_pose()
        
        self.if_hold = False
    
    def setup_idle_params(self):
        '''
        Set up params for idling, after updating the center and object_to_draw
        '''
        self.current_target = self.update_current_pose()
        self.current_draw_status = False
        



    def generate_cross_targets_draws(self):
        '''
        Update target_list and draw status list for drawing a cross
        '''
        delta_y = delta_x = 1.0/(2.0*np.sqrt(2)) * LINE_LENGTH

        #first segment: stand off point -> first point (z will be updated by force control) -> second point (z will be updated by force control). 
        cross_target_poses = [None]*8
        cross_target_poses[0] = self.center + np.array([-1*delta_x, -1*delta_y])                         #a-b
        cross_target_poses[1] = self.center + np.array([-1*delta_x, -1*delta_y])                         #b-b
        cross_target_poses[2] = self.center + np.array([ 1*delta_x,  1*delta_y])                         #b-c
        cross_target_poses[3] = self.center + np.array([ 1*delta_x,  1*delta_y])                         #c-c
        #second segment: stand off point -> first point (z will be updated by force control) -> second point (z will be updated by force control). 
        cross_target_poses[4] = self.center + np.array([ 1*delta_x, -1*delta_y])        #c-d
        cross_target_poses[5] = self.center + np.array([ 1*delta_x, -1*delta_y])        #d-d
        cross_target_poses[6] = self.center + np.array([-1*delta_x,  1*delta_y])        #d-e
        #third segment: stand off point for next action
        cross_target_poses[7] = self.center + np.array([-1*delta_x,  1*delta_y])        #e-e
        
        self.target_list = cross_target_poses
        self.draw_status_list = [False, True, True, False, False, True, True, False]


    def update_current_pose(self):
        #Returns the current pose in an array
        current_pose = self._limb.endpoint_pose()
        full_return_pose = [current_pose['position'].x , current_pose['position'].y,current_pose['position'].z,current_pose['orientation'].x, current_pose['orientation'].y, current_pose['orientation'].z, current_pose['orientation'].w ]
        partial_return_pose = full_return_pose[:2]
        return partial_return_pose 


    def get_xy(self):
        '''
        Returns the waypoint [x,y] for the next instant. 
        '''
        coord = None
        if self.object_2_draw == "circle":
            #TODO
            pass
        elif self.object_2_draw == "cross":
            t = rospy.Time.now().to_sec() - self.line_init_time
#            s = 10 * (1.0 * t / TF) ** 3 - 15 * (1.0 * t / TF) ** 4 + 6 * (1.0 * t / TF) ** 5
            s = 3.0*(t/TF)**2 - 2.0*(t/TF)**3
            if t>TF:
                s = 1
            coord = s * np.array( self.current_target ) + (1 - s) * np.array(self.line_init_pose)

        elif self.object_2_draw == "idle":
            coord = self.current_target
       
       #Test
        return coord



    def get_draw_status(self):
        return self.current_draw_status


def main():
    rospy.init_node("sawyer")
    trajgen = TrajGen()
    trajgen.generate_cross_targets_draws()
    
    r = rospy.Rate(1)
    for i in range(20):
        coord = trajgen.get_xy()
        print coord
        r.sleep()

if __name__=='__main__':
    main()
