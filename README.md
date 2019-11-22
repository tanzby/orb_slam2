# orb_slam2

Refined version of [ORB_SLAM2](https://github.com/raulmur/ORB_SLAM2) by using catkin.


## install

```
git clone https://github.com/tanzby/orb_slam2
cd orb_slam2
mkdir build
cd build
make -j4
```


## usage

start a stereo node

```
cd build
source devel/setup.bash
roslaunch orb_slam2_ros stereo.launch
```