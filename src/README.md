# Southern Arm Control Translators > src

This folder holds all of the source files for the translators.

## Files
scorbot_ik.cpp
* This file contains the inverse kinematics for the solver.
* The inputs to this file should be an x, y, z, theta_pitch, and theta_roll.
* The pitch is the angle the end-effector approaches the target at in relation to the horizontal plane.
* The roll is the angle the end-effector approaches the target at in relation to the horizontal ground plane.
* Pictures to explain and illustrate all of this is contained in this folder.
* All angle coming in and out of this node should be in radians.

## Folders
helpers/
* This folder holds any necessary include files for the translators.

## Notes
* Much of the code for the scorbot_ik comes from http://ijetae.com/files/Volume2Issue3/UETAE_0312_83.pdf

## TODO
* Fill in the files in the in the README.md once the files and system are more complete.
