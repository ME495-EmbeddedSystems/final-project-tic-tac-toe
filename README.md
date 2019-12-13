# ME 495 Final Project - Tic Tac Toe Artist on Sawyer

# Project Overview
The Tic Tac Toe artist is a Sawyer robot based interactive robot Tic Tac Toe player. At the current stage development, the robot goes first and using the [perfect Tic Tac Toe strategy](https://www.wikihow.com/Win-at-Tic-Tac-Toe) (basically you cannot win the robot, and this brutality will be neutralized as part of the future work).

##### Game Demonstration
[![Screenshot from 2019-12-13 13-39-59](https://user-images.githubusercontent.com/39393023/70827219-61c46f80-1dae-11ea-81c7-01a2741d246f.png)](https://youtu.be/F6CpxoHknUA)

## Usage
- To correctly set up the game, external drawing board is required with a fiducial point for board orientation recognition. 
![Screenshot from 2019-12-13 13-14-38](https://user-images.githubusercontent.com/39393023/70825660-90404b80-1daa-11ea-80b3-6a5e40704f1a.png)
- On your game computer, Run `rosrun final-project-tic-tac-toe tic_tac_toe`. Make sure board is not further than 5 cm from the end effector along the vertical direction. Otherwise, initial interaction force might be more than desired.
- When you play the game, hit enter to signal the computer that you have made your move. 

## Code Overview
![Screenshot from 2019-12-13 12-46-22](https://user-images.githubusercontent.com/39393023/70823906-99c7b480-1da6-11ea-8725-56962d8e52a5.png)


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

## Trajectory Generation 

This part encompasses all motions of the robot except drawing using force control. Along the entire trajectory there should not generate any jerks or unsafe moves. The motions are illustrated in the state machine diagram
![Screenshot from 2019-12-12 22-37-46](https://user-images.githubusercontent.com/39393023/70769669-153a4f00-1d30-11ea-9fe1-8f9bf5a67c2c.png)
**_How the state machine works_**:
The robot starts off at a pre-configured camera position in idle mode. When AI sends the position of the center of a new cross, the robot will move to the Write Standoff Position, which is the pre-configured center position. Afterwards, the robot will generate a trajectory of the cross and move to the first point of the trajectory. When the robot finishes drawing the cross, it will go back to the camera position. 

Therefore, to suit your application, adjust the below parameters in trajectory_generation.py
###### Pre-configured Positions and Sizes:
- Camera Position
- Write Standoff Position
- Size of each cell

**_Notes on Parameter Tuning_**:
 When determining the best board position (whose center should be the Write Standoff Position), be aware of 1. the robot's work space (Make sure no awkward motions are shown)2. glare on the board if there is any.

**_Future Work_**:
- Currently the robot can only draw crosses. Therefore, drawing circles will be a desirable feature
- A "kid" mode should be added so the game does not always play perfectly, especially in front of kids (Yep, hopefully kids will like this)

