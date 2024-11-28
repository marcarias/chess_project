# sensing package
One of the three main packages created for this work. It's main tasks are to **sense the environment** and **locate the chess pices**.

It has the following files:

1. **src/sensing.cpp**: sensing node. It starts the services related with sensing.
2. **srv/getPiecePose.srv**: file used by the *get_piece_pose* service of the sensing node.
3. **launch/sensing.launch**: launch file that starts the sensing and planning nodes.

A brief description of the sensing node and it's services can be seen in the following section.

## Sensing node

This node starts the *get_piece_pose* service. This service, takes as an input a piece as an input the frame the piece is related to as a string (ex: "aruco_frame_201") and gives as an output a transformStamped object indicating the translation and rotation of the piece relative to the *world* frame. 

To do so, the node uses a tfListener to obtain the tf tree of the scene. 

