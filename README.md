# ROS_Autonomous_Driving_pkg

# Sub Project Lists (meaning submodules)
All sub Projects are in **src** directory
- localization_pkgs
- objectDetection_pkgs (YOLO model)
- sensor_pkgs
- default_pkgs (For RC car)
- controller_pkgs (Expected..)

# Installation
### Build workspace
When you clone this repository for the first time, You need to build workspace as follow.

#### particle filter geometry/tf2
```
$ pip3 install pyyaml==5.4.1 <br/>
$ wstool init <br/>
$ wstool set -y src/geometry2 --git  https://github.com/ros/geometry2  -v 0.6.5 <br/>
$ wstool up <br/>
$ rosdep install --from-paths src --ignore-src -y -r <br/>
$ catkin_make --cmake-args -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m -DPYTHON_LIBRARY=/usr/lib/aarch64-linux-gnu/libpython3.6m.so <br/>
```

```
#in the your workspace
$catkin_make
```
# YOLO Model

#### Development Environment (Cuda 11.0)
```
$ pip3 install torch==1.7.1+cu110 torchvision==0.8.2+cu110 torchaudio==0.7.2 -f https://download.pytorch.org/whl/torch_stable.html
```

### Weight and cfg file for YOLO
https://drive.google.com/file/d/12LlgdAFJWwPgN4_0ME4Z3Rpd2qe_uBLk/view?usp=sharing, https://drive.google.com/file/d/1BchLk5e-iF7vailwSVYZ1xbRzpB5cFx6/view?usp=sharing, https://drive.google.com/file/d/1T8HjnDqDdpbK_7yE05saM6WKs3lHUHD7/view?usp=sharing, https://drive.google.com/file/d/1l_UERotZB5QO7OzhDGD8GdVy25Nxl2Qs/view?usp=sharing

