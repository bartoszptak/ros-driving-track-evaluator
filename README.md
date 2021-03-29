# ROS driving track evaluator - Prius version

This script uses python3 and av_msgs messages from the [car_demo](https://github.com/osrf/car_demo/tree/master/car_demo) package.

## Requirements and installation

```bash
apt install python3-pip
```

```bash
cd /av_ws/src
```

```bash
git clone https://github.com/bartoszptak/ros-driving-track-evaluator.git -b sim/prius
```

```bash
cd ros-driving-track-evaluator
```

```bash
pip3 install requirements.txt
```

```bash
catkin build
```

```bash
source devel/setup.bash
```

```bash
roslaunch ros-driving-track-evaluator run.launch
```

# Info

* Evaluator waits for autonomous mode to start by invoking topic:
```ros
rostopic pub --once /prius/mode av_msgs/Mode "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
selfdriving: true
collect: false"
```

* Terminate and display the penalty total when the selfdriving mode is terminated:
```ros
rostopic pub --once /prius/mode av_msgs/Mode "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
selfdriving: false
collect: false"
```
