# action package

One of the three main packages created for this work. It's main tasks are to **act the previously plan tasks** and **test the different offered services**.

It has the following files:

1. **src/action.cpp**: action node. It starts the services related with action.
2. **src/ur3_action_client.cpp**: node to offer services to control the arm. 
3. **src/gripper_action_client.cpp**: node to offer services to control the gripper. 
4. **launch/action.launch**: main launch file. Launch it to see the work done in this project. 
5. **srv/actionPick.srv**: file used by action_pick service.
6. **srv/actionPlace.srv**: file used by action_place service.
7. **srv/actionMove.srv**: file used by action_move service.
8. **srv/actionKill.srv**: file used by action_kill service.
9. **srv/actionCastle.srv**: file used by action_castle service.
10. **srv/ik.srv**: file used to compute inverse kinematics of the manipulator.
11. **srv/Attach.srv**: file used to attach/detach pieces to the greeper.

A brief description of the nodes and their services can be seen in the following sections.

## Action node

The job of this node is to start the different services offered by the action package. The list below briefly summaries the offered services, which will be explained in following sections.

1. **action_pick**: service to pick a piece.
2. **action_place**: service to place a piece.
3. **action_move**: service to move a piece.
4. **action_kill**: service to kill a piece.
5. **action_castle**: service to do a castle operation.
6. **action_home**: service to take arm to home configuration.
7. **test_move**: service to test the move action.
8. **test_kill**: service to test the kill action.
9. **test_castle**: service to test the castle action.
 
Besides the services, the node also has some helpful functions which are useful for achieving the services objectives. Those funcions are:

1. **coordToCell(x, y)**: function that takes as input x and y of a piece (as *double* values) and returns a string as cell code in which the piece is located (ex: "B5","A2", etc.)

### Action pick

Uses the *srv/actionPick.srv* file.

**Input**: integer indicating the piece to pick (ex; 202, 301, 210, etc.).

To do the task, it calls the *plan_pick* service offered by the *planning* package. It also calls the *move_ur3*, */team_A_arm/gripper_close*, and */link_attacher_node/attach* services offered by different nodes of this same package. 

### Action place

Uses the *srv/actionPlace.srv* file.

**Input**: string indicating the cell to place the picekd piece (ex: "B1", "C6", "H8", etc.).

To do the task, it calls the *plan_place* service offered by the *planning* package. It also calls the *move_ur3*, */team_A_arm/gripper_open*, and */link_attacher_node/detach* services offered by different nodes of this same package. 


### Action move

Uses the *srv/actionMove.srv* file.

**Input**: integer indicating piece to move and string indicating the cell where the piece has to be moved to. 

To do the task, it calls the *plan_move* service offered by the *planning* package. It also calls the *action_pick* and *action_place* services seen above.

### Action kill

Uses the *srv/actionKill.srv* file.

**Input**: integer indicating the piece to kill, and integer indicating the piece that kills (the killer).

To do the task, it calls the *plan_kill* service offered by the *planning* package. It also calls the *action_pick*, *action_place*, *move_ur3*, */team_A_arm/gripper_open*, and */link_attacher_node/detach* services seen above.


### Action castle

Uses the *srv/actionCastle.srv* file.

**Input**: Integer indicating the rook piece, and integer indicating the king piece.

To do the task, it calls the *plan_castle* service offered by the *planning* package. It also calls the *action_move* service seen above.

### Action home

Takes robot to home configuration by calling *move_ur3* service. 

### Test move

Calls *action_move* service to do a small showcase of the service.

<figure class="video_container">
  <iframe src="video/ShowMove.mp4" frameborder="0" allowfullscreen="true"> 
</iframe>
</figure>

### Test kill

Calls *action_kill* service to do a small showcase of the service.

<figure class="video_container">
  <video controls="true" allowfullscreen="true">
    <source src="video/ShowKill.mp4" type="video/mp4">
  </video>
</figure>

### Test castle

Calls *action_castle* service to do a small showcase of the service.

<figure class="video_container">
  <video controls="true" allowfullscreen="true">
    <source src="video/ShowCastle.mp4" type="video/mp4">
  </video>
</figure>

## UR3 action client node

This node controlls the ur3 arm. It subscribes to the */team_A_arm/joint_states* publisher to obtain the current joint states. It offers one main service:

1. **move_ur3**: service to move ur3 arm.

### Move ur3

This service is used to move the ur3 robot to the given pose. It uses the findIK function to compute the inverse kinematics solutions. It later calculates which is the "closest" joint solution and sets it as a new goal by calling *setRectilinearTrajectory* function. It finally moves the robot by calling *moveRobotTrajectory* function.

**Input**: Pose to set the robot to (px, py, pz, qx, qy, qz, and qw)

## Gripper action client node

This node controlls the *simulated gripper*. It offers the following services:

1. **team_A_arm/gripper_open**: service to open the gripper.
2. **team_A_arm/gripper_close**: service to close the gripper.

### Gripper open

It oppens the gripper by setting an apperture of 0.5.

### Gripper close

It closes the gripper by setting an apperture of 0.53.

## Action launch file

The main launch file of this project. It calls/opens the following files:

1. **sensing/launch/sensing.launch**: explained in sensing package.
2. **src/action.cpp**: explained above, creates the action services.
3. **ros2223-final-project/test_hardware/launch/test_hardware_demo.launch.xml**: launch file that starts the simulation, robot, gripper, and other nodes. It is explained in the *ros2223-final-project* package. 
