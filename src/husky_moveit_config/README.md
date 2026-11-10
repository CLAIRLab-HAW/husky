# husky_moveit_config

What of the a200-0553's MoveIt configuration the Clearpath generator does not write -- the counterpart of MARWIN 5's
`marwin_moveit_config`, but only its viewing half. The configuration `move_group` runs with (SRDF, kinematics,
controllers, limits) is generated from [`setup/config/robot.yaml`](../../setup/config/robot.yaml) on the robot and in
the mock alike; a static copy of it here would be a second truth beside it.

| File | What it is |
|---|---|
| `config/moveit.rviz` | RViz with the MoveIt MotionPlanning plugin against `/a200_0553/move_group`: absolute topics, planning group `arm_0`, fixed frame `base_footprint`, the Interact tool |
| `config/kinematics.yaml` | KDL for the plugin's interactive marker, loaded onto the rviz2 node; `move_group`'s own solver stays the generated one |

Both are read by `view-moveit` (`deploy/offboard/scripts/view-moveit` in the workspace), which runs in the `lite`
image on the robot and in the offboard viewer, and fetches URDF and SRDF from the running `move_group`.
`MOVEIT_RVIZ_CONFIG` and `MOVEIT_KINEMATICS` override the two paths.
