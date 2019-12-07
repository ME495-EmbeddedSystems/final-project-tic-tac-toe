#!/usr/bin/env python

import intera_interface
import rospy
import numpy as np

TF = 3
DIST_THRE = 0.003
LINE_LENGTH = 0.06
HOLD_TIME = 2.5

# green_point: x, y, z =  0.583772903694    -0.0935753264747    -0.114156335215
#This is the desired elbow camera pose. Modify for your application
#TODO
#Coordinate of the marking. Modify for your application
#GREEN_POINT_COORD = np.array([0.6386302984868262, -0.04967439369161873]) 
GREEN_POINT_COORD = np.array([0.6263453770987546, -0.12562004467910037]) 
#GREEN_POINT_Z = -0.114156335215
#CHECKER_CENTER_COORD = GREEN_POINT_COORD + 0.01*np.sqrt(2.0)*np.array([1.0+7.0+1.8+3.5, 1.0+7.0+1.8+3.5])    #close to x=0.7881137132331955, y=0.08409585399784152 z=0.08095801755255924
CHECKER_CENTER_COORD = np.array([0.7267684830590353, 0.04111839299227235])

def InsideDistThresh(pos_start, pos_end, DIST_THRE):
    #Checks if two points' x,y position difference is larger than DIST_THRE
    if np.linalg.norm( pos_start - pos_end ) > DIST_THRE:
        return False
    else:
        return True


class TrajGen(object):
    def __init__(self):
        global TF, DIST_THRE, LINE_LENGTH, HOLD_TIME, CHECKER_CENTER_COORD

        self._limb = intera_interface.Limb("right")
        self.center = np.array([])
        self.target_list = []
        self.draw_status_list = []
        self.hold_init_time = None
        self.line_init_time = None

        self.current_target = np.array([])
        self.current_draw_status = 0    #-1 is going up, 0 is not draw, 1 is apply force control
        self.if_hold = 0 #0 is not to hold, -1 is not to hold but just get back from hold. 1 is to hold
        self.line_init_pose = None
        self.s = 0

        self.object_2_draw = "idle"
#        self.setup_idle_params()

        #Test
        #print self._limb.endpoint_pose()
        self.object_2_draw = "cross"
        self.go_to_camera_or_standoff('standoff')
        self.center = CHECKER_CENTER_COORD + 0.01*np.array([7.0+1.8, 7.0+1.8])
        print "center: ", self.center
        
        self.setup_cross_params()

        #Test
                
    def update_trajectory_status(self):
        '''
        Updates trajectory variable status, every time it is called in the force control loop. 
        Key params: 
            1. self.draw_status: -1 means moving up, 0 is moving horizontally, 1 is moving down (force control required)
        '''
        if self.if_hold != 1:

            if self.object_2_draw == "cross":
            
                current_pose = self.update_current_pose()
                # if InsideDistThresh( current_pose, self.current_target, DIST_THRE ):
                print"this is cross"
                if (self.s == 1):
                    self.s = 0
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
                        
                    #Whole Cross is done
                    else:

                        #go to camera position
                        self.go_to_camera_or_standoff('camera')
                       # #checker center is the stand off position
                       # self.go_to_checker_center()
                        
                        #next action is idle
                        self.object_2_draw = "idle" 
                        while self.object_2_draw == "idle":
                            self.get_next_object_center()
            
                        self.setup_cross_params()                       
                        #checker center is the stand off position
                        self.go_to_camera_or_standoff('standoff')
                        
                        #self.setup_idle_params()


#            # if the current object to draw is idling
            elif self.object_2_draw == "idle":
                while self.object_2_draw == 'idle':
                    self.get_next_object_center()
 
                self.setup_cross_params()                       
                 #checker center is the stand off position
                self.go_to_camera_or_standoff('standoff')
#                #go to camera position
#                #self.go_to_camera_pose()
#
#                # Wait for AI to give object_2_draw and ceter
#                self.get_next_object_center()
#               
#                #if the next action is cross
#                if self.object_2_draw == "cross":
#                    self.setup_cross_params()                       
#                    #checker center is the stand off position
#                    self.go_to_camera_or_standoff('standoff')
# 
#                #if the next action is idle
#                elif self.object_2_draw == "idle": 
#                    self.setup_idle_params()

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




    def go_to_camera_or_standoff(self,destination):
        '''
        Destination: 'camera' - camera pose; 'standoff' - stand off pose
        '''
        joint_angles = None
        if destination == 'camera':
            joint_angles = { 'right_j0': -0.341524414063,
                             'right_j1': -0.125116210937,
                             'right_j2': -2.59296875,
                             'right_j3': -0.671841796875,
                             'right_j4': -0.513673828125,
                             'right_j5': -2.89591015625,
                             'right_j6': -4.71028613281}

           
        elif destination == 'standoff':
       
            joint_angles = { 'right_j0': -0.570514648437,
                             'right_j1': -0.405889648437,
                             'right_j2': -2.72518945313,
                             'right_j3': -1.56860742187,
                             'right_j4': -0.524416015625,
                             'right_j5': -0.679075195313,
                             'right_j6': -1.51012792969}

        self._limb.move_to_joint_positions(joint_angles)


    def setup_cross_params(self):
        '''
        Set up params for generating a cross, after updating the center and object_to_draw
        '''
        self.s = 0
        self.generate_cross_targets_draws()
        self.current_target = self.target_list[0]
        self.target_list.pop(0)
        
        self.current_draw_status = self.draw_status_list[0]
        self.draw_status_list.pop(0)

        self.line_init_time = rospy.Time.now().to_sec()
        self.line_init_pose = self.update_current_pose()
        
        self.if_hold = 0
   

#    def setup_idle_params(self):
#        '''
#        Set up params for idling, after updating the center and object_to_draw
#        '''
#        self.current_target = self.update_current_pose()
#        self.current_draw_status = 0
#        self.if_hold = 0 

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
        #self.draw_status_list = [False, True, True, False, False, True, True, False]
        #draw_status: -1 move up, 0 move horizontally, 1 move down(force control)
        self.draw_status_list = [0, 1, 1, -1, 0, 1, 1, -1]



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
        print self.object_2_draw
        coord = None
        if self.object_2_draw == "circle":
            #TODO
            pass
        elif self.object_2_draw == "cross":
            t = rospy.Time.now().to_sec() - self.line_init_time
            self.s = 10 * (1.0 * t / TF) ** 3 - 15 * (1.0 * t / TF) ** 4 + 6 * (1.0 * t / TF) ** 5
            # self.s = 3.0*(t/TF)**2 - 2.0*(t/TF)**3
            if t>TF:
                self.s = 1
            coord = self.s * np.array( self.current_target ) + (1 - self.s) * np.array(self.line_init_pose)

        elif self.object_2_draw == "idle":
            #test
            print "get_xy_idle"
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
        r.sleep()

if __name__=='__main__':
    main()
