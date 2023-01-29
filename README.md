# Separable Jaws Bringup

## Usage

Launch ros2_control for separable jaws.

```bash=
ros2 launch separable_jaws_bringup control.launch.py
```

Test if the robot can run

```bash=
ros2 run separable_jaws_bringup joint_trajectory_controller_test_node
```
