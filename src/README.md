# Southern Arm Control Translators > src

This folder holds all of the source files for the translators.

## Files
### scorbot_ik.cpp
* This file contains the inverse kinematics for the solver.
* The inputs to this file should be an x, y, z, theta_pitch, and theta_roll, time.
* The pitch is the angle the end-effector approaches the target at in relation to the horizontal plane.
* The roll is the angle the end-effector approaches the target at in relation to the horizontal ground plane.
* Pictures to explain and illustrate all of this is contained in this folder.
* All angles coming in and out of this node should be in radians.
* Some of the equations are modified from the paper "Kinematics for Lynxmotion Robot Arm" by Dr. Rainer Hessmer published October 2009

## Folders
### helpers/
* This folder holds any necessary include files for the translators.
