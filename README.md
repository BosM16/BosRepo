# BosRepo

## How to launch demo

### Launch nodes:
```bash
roslaunch bebop_driver bebop_node.launch
roslaunch try_out_vel_com try_out.launch
roslaunch bebop_tools joy_teleop.launch
```

### Start demo:
```bash
rostopic pub --once /demo std_msgs/Empty
```






![Alt text](https://g.gravizo.com/source/custom_mark13?https%3A%2F%2Fraw.githubusercontent.com%2FBosMathias%2FBosRepo%2Fmaster%2Ftiming_diagram.PLANTUML)
<details>
<summary></summary>
</details>
