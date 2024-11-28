# Chesslab_setup Package

This package has mainly been created using the **chesslab_setup** package that was given in this work. Despite that **one main file** was modified.

1. **worlds/flat_chesslab.world**

## flat_chesslab.world changes
A slight modification was made to the file in order to **automatically add the light** needed to see the *aruco*. 

## Other files used form the package
Nevertheless, other files form this package are used and called by one of the launch files used. This can be summaried in the following list:

1. **robot/team_A_arm_gripper.urdf.xacro**: urdf file to simulate the robot arm and gripper.
2. **config/new_controllers.yaml**: configuration file to load all controllers.
3. **config/new_controllers_gripper.yaml**: configuration file to load all gripper controllers.
4. **launch/ur3e_bringup_custom.launch**: Main file to load robot desription, driver, controllers, etc. 
5. **config/my_robot_calibration.yaml**: Configuration of the robot (to be updated when using a real robot or a simulated one). 
6. **launch/ur3_upload_custom.launch**: Robot description file used in the ur3e_bringup_custom.launch. 
7. **robot/camera_right.urdf.xacro**: Camera urdf model.
8. **config/chesslab_simple_cameras.rviz**: To create and set the rviz visualization.
9. **config/team_A_all.perspective**: perspective file. 

As it is explained in the corresponding launch file's section, not all files are called. It depends on the values the **arguments** are set to. 

