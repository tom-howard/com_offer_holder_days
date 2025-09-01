# COM Offer Holder Day Application for the Waffles (2026+)

## ROS 2 CLI Calls

Calling the `turn.py` script to make the robot turn by `X` degrees:

```bash
ros2 run com_offer_holder_days turn.py --ros_args -p yaw_ang:=X
```

Where `X` is an integer value representing the desired angle (in degrees) for the robot to turn on the spot.

Calling the `forward.py` script to make the robot move forwards by `Y` meters:

```bash
ros2 run com_offer_holder_days forward.py --ros_args -p fwd_dist:=Y
```

Where `Y` is a double precision value representing the desired distance (in meters) for the robot to move forwards.