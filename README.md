# rasberry_tracking

![strawberry_localisation](https://user-images.githubusercontent.com/16948324/76231446-2c98b380-621d-11ea-8624-8e472c2f08f8.gif)

The rasberry_tracking package to add tracking functionality to 
[RaymondKirk/rasberry_perception](https://github.com/RaymondKirk/rasberry_perception). 

## Quick start

```bash
roslaunch rasberry_perception detector.launch backend:="detectron2" password:="obtain_from_raymond" image_ns:="/your_camera/colour" depth_ns:="/your_camera/depth" score:="0.5"
```

## Installation

This project is dependent on a modified version of LCAS/bayestracking found at [RaymondKirk/bayestracking](https://github.com/RaymondKirk/bayestracking). 

```bash
cd catkin_ws/src
git clone https://github.com/RaymondKirk/bayestracking
git clone https://github.com/RaymondKirk/rasberry_tracking
catkin build rasberry_tracking
```
