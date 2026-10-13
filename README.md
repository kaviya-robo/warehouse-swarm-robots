# Warehouse Swarm Robots – Simscape Multibody

A Simulink / Simscape Multibody simulation of a warehouse with a teleoperated mobile robot and a swarm of
autonomous copies of it. The swarm robots follow their own paths, avoid shelves, dropped boxes and each
other, and give way to the teleoperated robot.

The robot model is imported from SolidWorks (`Assem2`) and driven kinematically as a 4-wheel
differential-drive base.

## Requirements

- MATLAB R2025b (or similar) with Simulink, Simscape and Simscape Multibody

## Run

```matlab
drive_robot
```

or double-click `start_robot.bat`. On first start the model is rebuilt from `robot_world.m`.
Mechanics Explorer shows the 3D scene, and a control window shows a live top-view map.

| Key | Action |
|---|---|
| W / ↑ | forward |
| S / ↓ | backward |
| A / ← , D / → | rotate |
| Space | stop |
| Q / Esc | quit |

## What's in the simulation

- **Warehouse:** 18 m × 14 m floor, 8 shelf racks, pallets, perimeter wall, two dropped boxes in the main aisle.
- **Teleoperated robot:** a collision gate stops it before it hits a rack, wall or swarm robot.
- **Swarm robots (4):** full copies of the robot (chassis, wheels, joints), each patrolling its own loop.

### Swarm controller (`swarm_controller.m`)

Runs every 0.1 s for each robot:

1. **Path following** – pure pursuit on a point 2 m ahead along the robot's closed path.
2. **Lane shift** – if the path ahead is blocked, the target slides sideways (12.5 cm steps, up to 1.5 m)
   to the nearest free corridor; with no free corridor the robot waits 0.5 m before the blockage.
3. **Dynamic Window Approach** – ~70 (speed, turn-rate) candidates are simulated 2 s ahead; any that come
   within 5 cm of an obstacle or robot are rejected, the rest are scored on path tracking, clearance and speed.
4. **Prioritised planning** – right of way: teleoperated robot, then Robot2, Robot3, … Each robot avoids the
   intended 2 s plans of higher-priority robots and reverses to make room for a stuck one.
5. **One-way aisles** – both lanes of the main aisle run the same way, so robots never meet head-on.

## Files

| File | Purpose |
|---|---|
| `drive_robot.m` | Teleoperation window, live map, starts the simulation |
| `robot_world.m` | **Edit here:** floor, racks, obstacles, swarm routes, speeds |
| `swarm_controller.m` | Path following + obstacle avoidance for the swarm |
| `add_world.m` | Adds floor / obstacles and the collision gate to the model |
| `add_swarm.m` | Builds the `Swarm` subsystem (robot copies + controller) |
| `robot_collision_gate.m`, `robot_hits.m`, `robot_move_blocked.m` | Collision checks |
| `build_robot_motion.m` | Turns the imported `Assem2` model into the driveable `Assem2_Drive` |
| `Assem2.slx`, `Assem2_DataFile.m`, `Assem2.xml` | SolidWorks import |
| `Assem2_Drive.slx` | Driveable simulation model |

## Customising

Edit `robot_world.m` and run `drive_robot` again:

- obstacles: rows `[x y length width height yaw_deg]` in `W.boxes`
- swarm robots: add / remove entries in `W.swarm` (start pose, route corners, colour)
- behaviour: `W.swarmSpeed`, `W.swarmYawRate`, `W.swarmLookahead`, `W.swarmHorizon`
