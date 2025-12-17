# Robotic Arm Pick-and-Place (Klampt)

## What I've done'

- Loaded a robot model (`tx90l.rob`) into a Klampt `WorldModel`
- Built a scene with:
  - a ground plane
  - two tables (left and right)
  - three mugs placed on the right table
- Implemented a pick-and-place routine:
  1. move to a safe “home” pose
  2. move above the mug (safe height)
  3. move down to a grasp point
  4. **attach** the mug to the end-effector (simulated grasp)
  5. lift to safe height
  6. move above the destination slot
  7. move down to place
  8. release (stop updating the attachment)

## How the robot moves (high-level)

- **IK (position-only)**: The end-effector is driven to a desired 3D point in world coordinates using Klampt’s IK solver.
- **Smooth motion**: After IK returns a target joint configuration, the robot interpolates from its current configuration to the target configuration over several small steps.
- **Simulated grasp**: When “grasping”, the mug is rigidly attached to the end-effector by storing its relative transform. While attached, the mug’s pose is updated every frame so it follows the end-effector.