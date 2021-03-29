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
