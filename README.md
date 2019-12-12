# ME 495 Final Project - Tic Tac Toe on Sawyer

## Usage

Run `rosrun final-project-tic-tac-toe tic_tac_toe`. Make sure board is not further than 5 cm from the end effector along the vertical direction. Otherwise, initial interaction force might be more than desired.

## Hybrid Motion-Force Control

This part is inside the main code, `tic_tac_toe`.  Desired position on the board plane (x,y) is accesed by `trajectoryGenerator.get_xy()`.
In addition to that, the information of whether the robot should draw, move in the air or get away from the board is accessed by `trajectoryGenerator.get_draw_status()`.
The details of how these functions work is explained in the **Trajectory Generation** section.

**_When the robot needs to draw_**:
Force control is used to decide on the desired _z_ position of the end effector.
The force control problem is converted to a position control problem.
Desired position along the *z* axis(axis that is perpendicular to the board) is set by using a simple P controller as follows:

```
desired_Z_pos_InHand = desired_Z_posInHand + P*(desired_EE_Force - currentForce)
desired_Z pos_InWorld = Current_Z_pos - desired_Z_pos_InHand
```

Change in the desired `z` position in hand frame is the output of the controller. And it does not depend on the current end effector position.
This is important because after the marker is contact, its _z_ position is constant.
 
If the applied force is not enough, desired z position will be further from the end effector until it overshoots and then comes to a balance.

**_When the robot needs to lose contact with the board_**: desired _z_ position is set 1.5 cm above the current end effector position.

**_When the robot needs to move in the air_**: desired _z_ position is set to the current end effector position.

After desired _z_ position is found, it is used in combination with the desired (x,y) position obtained from the 
TrajectoryGeneration class and given as input to the built in inverse kinematic function of the Sawyer.



