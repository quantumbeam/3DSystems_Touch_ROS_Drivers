3D Systems Touch ROS Driver
============

ROS Packages for connecting *one* or *more* 3D Systems Touch (previously known as Phantom Touch or Geomagic Touch) haptic devices, **USB** version.
This repository is forked from the [Geomagic_Touch_ROS_Drivers](https://github.com/bharatm11/Geomagic_Touch_ROS_Drivers) repository.


## Environment
- **Confirmed Device**:  
  This package is tested with the **3D Systems Touch HID** haptic device.
  While it has not been specifically tested with the **Touch X**, it is likely to work with it as well.  
- **Supported OS**:  
  This package is tested on **Ubuntu 20.04** and **Ubuntu 22.04**.  

- **ROS Version**:  
  Tested with **ROS Noetic**.  

- **Notes for Ubuntu 22.04**:  
  The package has been tested only in a Docker environment running a **ROS Noetic setup on an Ubuntu 20.04 image**.  


## Install Device Driver and SDK (OpenHaptics)

Run the following command to install the device driver.  
**Driver Version**: `TouchDriver_2024_09_19`  
**Compatibility**: Ubuntu 20.04 and 22.04  

> **Note**: Although the official documentation does not explicitly mention support for Ubuntu 20.04, confirmation from 3D Systems indicates that this driver is compatible with Ubuntu 20.04 as well.  

```bash
./install_all.sh
```

You can test the connection of the device by responding to the prompt:  

```bash
./checkup_Touch_device.sh
```

## Launch ROS Nodephantom

After installing the device driver and building the workspace, you can launch the ROS node.
This launch file will start the ROS node and publish the data from the haptic device.
You can read data from the haptic device and write force feedback to the device.
```bash
roslaunch touch_common touch_state.launch
```

Data from the haptic device can be read from the following topics:

  /touch/button
  
  /touch/force_feedback
  
  /touch/joint_states
  
  /touch/pose
  
  /touch/state

With RViz visualization launch is as follows:
```bash
roslaunch touch_common touch.launch
```

## Force feedback safety

`/touch/force_feedback` (`touch_msgs/TouchFeedback`, force in N) is applied with two limits:

| Parameter | Default | Meaning |
|---|---|---|
| `force_timeout_ms` | `100.0` | If no new force arrives for this long (ms), the commanded force becomes 0. Before this change the driver kept applying the last force forever, so a crashed or disconnected sender left the force on. Provisional value (same as gello_ros `touch_timeout_ms`), not tuned on the device. |
| `max_force_n` | `3.3` | The magnitude of the total force (user force + damping) is limited to this value (N). 3.3 N is the Touch's maximum output (datasheet). |

Non-finite components are treated as 0. On shutdown the haptic loop writes zero force for ~10 ms before the scheduler stops.
The publisher must therefore keep publishing (at least every `force_timeout_ms`) while it wants a force applied.

Verification: unit tests (`colcon test --packages-select touch_common`), and a run with a stub OpenHaptics library
(force drops to 0 about 100 ms after the sender stops; 10 N is limited to 3.3 N). Not yet verified on a real Touch.

## Acknowledgements and Citation
This repository is forked from the [Geomagic_Touch_ROS_Drivers](https://github.com/bharatm11/Geomagic_Touch_ROS_Drivers) repository.
The original repository by Francisco Suárez Ruiz, [http://fsuarez6.github.io](http://fsuarez6.github.io) for the Sensable PHANToM haptic device (https://github.com/fsuarez6/phantom_touch).

ROS packages developed by the [Group of Robots and Intelligent Machines](http://www.romin.upm.es/) from the [Universidad Politécnica de Madrid](http://www.upm.es/internacional). This group is part of the [Centre for Automation and Robotics](http://www.car.upm-csic.es/) (CAR UPM-CSIC).


Please cite these papers in your publications if this repository helps your research.

```
@INPROCEEDINGS{8941790,
  author={Mathur, Bharat and Topiwala, Anirudh and Schaffer, Saul and Kam, Michael and Saeidi, Hamed and Fleiter, Thorsten and Krieger, Axel},
  booktitle={2019 IEEE 19th International Conference on Bioinformatics and Bioengineering (BIBE)},
  title={A Semi-Autonomous Robotic System for Remote Trauma Assessment},
  year={2019},
  volume={},
  number={},
  pages={649-656},
  doi={10.1109/BIBE.2019.00122}}
  
@inbook{doi:10.1137/1.9781611975758.2,
author = {B. Mathur and A. Topiwala and H. Saeidi and T. Fleiter and A. Krieger},
title = {Evaluation of Control Strategies for a Tele-manipulated Robotic System for Remote Trauma Assessment},
booktitle = {2019 Proceedings of the Conference on Control and its Applications (CT)},
chapter = {},
pages = {7-14},
doi = {10.1137/1.9781611975758.2},
URL = {https://epubs.siam.org/doi/abs/10.1137/1.9781611975758.2},
eprint = {https://epubs.siam.org/doi/pdf/10.1137/1.9781611975758.2}}
```

