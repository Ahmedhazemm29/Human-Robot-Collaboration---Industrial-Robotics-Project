# block_sorter

Human-robot collaborative block-sorting demo for a UR5e running MoveIt2
on ROS2 Humble. Built to slot into the same workspace as
`bt_action_server` and reuse the same hand-collision-avoidance pattern.

## What it does

1. Subscribes to `/planning_scene` and waits until three collision
   objects with ids `block_1`, `block_2`, `block_3` are present.
2. Waits for the human to pick one block — detected as a block
   disappearing from the scene (either an explicit `REMOVE` operation
   or simply absent in subsequent scene messages).
3. Picks one of the two remaining blocks, moves it to that block's
   pre-assigned sort slot, releases it. Repeats for the second
   remaining block.
4. While executing any motion, watches for a collision object named
   `hand_exclusion_zone`. On rising-edge detection it calls
   `/check_state_validity` for every waypoint of the stored plan. If
   any waypoint is invalid (i.e. the hand actually intersects the
   planned trajectory), `move_group.stop()` is called, the node waits
   for the hand to leave, then replans to the same goal from the
   robot's current state and re-executes.

## What you need to provide

- A node publishing the three blocks as collision objects in
  `/planning_scene`. Anything that uses MoveIt's `PlanningSceneInterface`
  to `addCollisionObjects()` will work.
- A node publishing the human hand as a `hand_exclusion_zone`
  collision object whenever the hand is in the workspace (camera +
  perception). Same shape as above.
- The standard `ur_moveit` stack running (move_group, robot_description,
  etc.) so `MoveGroupInterface` can plan.

## Hardcoded values you will want to edit

In `src/sort_blocks.cpp`:

| Constant | Default | Meaning |
|----------|---------|---------|
| `kSlotPose` (built in `slotMap()`) | three poses on `x=0.50`, `y={-0.15, 0, +0.15}`, `z=1.10` | Where each block should be placed once sorted |
| `kApproachZOffset` | `0.10 m` | How far above each block/slot to plan |
| `kTopOrientation` | `(-0.087, 0.996, 0.010, 0.007)` | Top-down end-effector orientation |

## What's stubbed

Gripper open/close is left as two `// TODO` lines in `pick_and_place()`.
Drop in whatever your end effector uses (Robotiq URCap service,
OnRobot, custom GPIO, etc.).

## Build

```bash
# from your workspace root
colcon build --packages-select block_sorter
source install/setup.bash
```

## Run

In separate terminals, after `ur_moveit` is up and your scene
publisher is publishing the three blocks:

```bash
ros2 launch block_sorter sort_blocks.launch.py
# or directly
ros2 run block_sorter sort_blocks
```

Pick up `block_2` by hand. You should see:

```
[block_sorter]: Three blocks present. Waiting for the human to pick one...
[block_sorter]: Human picked block_2. Robot will sort the remaining two.
[block_sorter]: Executing motion: approach block_1
[block_sorter]: Hand blocks waypoint at t=1.20s — stopping.   (if you reach in)
[block_sorter]: Waiting for human hand to leave the workspace...
[block_sorter]: Hand cleared — proceeding.
[block_sorter]: Executing motion: approach block_1
[block_sorter]: Reached: approach block_1
...
[block_sorter]: Sorting complete.
```
