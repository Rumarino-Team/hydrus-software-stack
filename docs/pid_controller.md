# PID Controller Usage

The `controllers.py` node now implements a PID based controller. Each movement
axis (depth, rotation and linear motion) has its own PID instance. Default
gains are set in the controller constructor but will likely require tuning for
your platform.

## Parameters
- **Depth PID** (`depth_pid`)
  - Kp: 50.0
  - Ki: 0.0
  - Kd: 0.0
- **Rotation PID** (`rotation_pid`)
  - Kp: 30.0
  - Ki: 0.0
  - Kd: 0.0
- **Linear PID** (`linear_pid`)
  - Kp: 40.0
  - Ki: 0.0
  - Kd: 0.0

All controllers clamp their output between `-200` and `200` PWM units. These
limits and gains can be modified directly in `controllers.py`.

## Tuning Procedure
1. **Set Ki and Kd to zero** and increase `Kp` until the system begins to
   oscillate when responding to a step input. Reduce `Kp` slightly so that the
   response becomes stable.
2. **Increase `Kd`** to dampen overshoot. Raise it until oscillations are
   acceptably reduced.
3. **Increase `Ki`** slowly to eliminate steady state error. Use small steps to
   prevent slow oscillations.
4. Repeat the process for each axis (depth, rotation, linear). Ideally perform
   tuning in a controlled environment such as a test pool.

Remember to restart the controller node after modifying the gains.
