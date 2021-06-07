# base_nav

This package is a simple control node to move the base of the PR2 robot without any collision detection. 
It can be used to move small distances or to correct position after a move base goal is completed.

The Dock action server takes a target pose as input and gives feedback of the distance to the goal.
