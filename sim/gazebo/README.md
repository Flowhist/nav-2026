# Office Corridor Gazebo World

This folder contains an SDF world reconstructed from your hand-drawn office corridor top view.

## File

- `worlds/office_corridor_sketch.world`

## Run in Gazebo

From repository root (`finav`):

```bash
gazebo sim/gazebo/worlds/office_corridor_sketch.world
```

## Notes

- Layout is an engineering approximation based on the sketch.
- Vehicle reference block uses your URDF footprint: `0.95m x 0.62m`.
- Blue obstacle and red start marker are included as static models.
- Recommended robot spawn near start marker:
  - Position: `x=-0.15, y=-5.65`
  - Heading: `yaw=+90deg` (toward +Y)

## Fast tuning points

If you need to refine geometry, edit these model poses/sizes inside the world file:

- Main corridor walls: `wall_main_left_bottom`, `wall_main_right`
- Left branch structure: `wall_left_step_*`, `wall_left_*_horizontal`
- Obstacle: `obstacle_blue_box`
- Start marker: `start_marker_flag`
