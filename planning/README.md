# planning package

One of the three main packages created for this work. It's main tasks are to **plan the tasks** and **give the point trajectories**.

It has the following files:

1. **src/planning.cpp**: planning node. It starts the services related with planning.
2. **srv/planPick.srv**: file used by plan_pick service.
3. **srv/planPlace.srv**: file used by plan_place service.
4. **srv/planMove.srv**: file used by plan_move service.
5. **srv/planKill.srv**: file used by plan_kill service.
6. **srv/planCastle.srv**: file used by plan_castle service.

A brief description of the planning node and it's services can be seen in the following section.

## Planning node

The job of this node is to start the different services offered by the planning package. The list below briefly summaries the offered services, which will be explained in following sections.

1. **plan_pick**: service to plan the pick trajectory and actions.
2. **plan_place**: service to plan the place trajectory and actions.
3. **plan_move**: service to plan the trajectory and actions to move a piece.
4. **plan_kill**: service to plan the trajectory and actions to kill a piece with another piece.
5. **plan_castle**: service to plan the trajectory and actions to do a castle operation with a rook and a king of the same colour. 

Besides the services, the node also has some helpful functions which are useful for achieving the services objectives. Those funcions are:

1. **coordToCell(x, y)**: function that takes as input x and y of a piece (as *double* values) and returns a string as cell code in which the piece is located (ex: "B5","A2", etc.)
2. **cellToCoord(cell)**: oposite of *coordToCell**. Takes a strin in cell code as an input and returns a touple with the x and y values of the cell. 

### Plan pick

Uses the *srv/planPick.srv* file.

**Input**: integer indicating the piece to pick (ex; 202, 301, 210, etc.).

**Output**: two transforms to create the path to follow by the gripper. First transform (p1) indicates a safe point above the grasping point, second transform (p2) indicates the grasping point itself. The points are calculated kepping the piece heigths in mind. 

The planed actions are:

1. GRIPPER OPEN
2. MOVE TO: p1
3. MOVE TO: p2 
4. GRIPPER CLOSE
5. MOVE TO: p1
6. MOVE TO HOME

### Plan place

Uses the *srv/planPlace.srv* file.

**Input**: string indicating the cell to place the picekd piece (ex: "B1", "C6", "H8", etc.).

**Output**: two transforms to create the path to follow by the gripper. First transform (p1) indicates a safe point above the placing point, second transform (p2) indicates the placing point itself. The points are calculated kepping the piece heigths in mind.

The planed actions are:

1. MOVE TO: p1
2. MOVE TO: p2 
3. GRIPPER OPEN
4. MOVE TO: p1
5. MOVE TO HOME 

### Plan move

Uses the *srv/planMove.srv* file.

**Input**: integer indicating piece to move and string indicating the cell where the piece has to be moved to. 

**Output**: Four points indicating the pick and place positions to do the action. The points are calculated using the pick and place services. 

The planed actions are:

1. PICK PIECE piece
2. PLACE IN cell

### Plan kill

Uses the *srv/planKill.srv* file.

**Input**: integer indicating the piece to kill, and integer indicating the piece that kills (the killer).

**Output**: Four points indicating the pick and place positiond to do the action. The points are calculated using the pick and place services. It uses the *coordToCell* and *cellToCoord* functions to compute the cell in which the killed piece was, in order to place the killer piece there. 

The planed actions are:

1. PICK PIECE killed
2. PLACE IN TRASH
3. PICK PIECE killer
4. PLACE IN cell

### Plan castle

Uses the *srv/planCastle.srv* file.

**Input**: Integer indicating the rook piece, and integer indicating the king piece.

**Output**: 8 points indicating the pick and place positions to do the action. The points are calculated depending on the combination of rook-king used. It uses the *coordToCell* and *cellToCoord* functions to compute the cell in which the king and rook pieces are. 

The planed actions are:

1. PICK PIECE rook
2. PLACE IN cell1
3. PICK PIECE king
4. PLACE IN cell2

