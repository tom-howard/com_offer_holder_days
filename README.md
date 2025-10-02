# COM Offer Holder Day Application for the Waffles (2026+)

## Installing

(Should work on the Robotics Lab Laptops using the `student` account.)

```
cd ~/ros2_ws/src/
```

```
git clone https://github.com/tom-howard/com_offer_holder_days.git
```

```
cd ~/ros2_ws/ && colcon build --packages-select com_offer_holder_days
```

```
source ~/.bashrc
```

## Robot/laptop pairing & setup

Make sure the laptop is paired with a robot, [as per the instructions here](https://tom-howard.github.io/com2009/waffles/launching-ros/).


## ROS 2 CLI Calls

Calling the `turn.py` script to make the robot turn by `X` degrees:

```bash
ros2 run com_offer_holder_days turn.py --ros-args -p angle:=X
```

Where `X` is an integer value representing the desired angle (in degrees) for the robot to turn on the spot.

Calling the `forward.py` script to make the robot move forwards by `Y` meters:

```bash
ros2 run com_offer_holder_days forward.py --ros-args -p dist:=Y
```

Where `Y` is a double precision value representing the desired distance (in meters) for the robot to move forwards.