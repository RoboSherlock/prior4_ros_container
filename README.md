# noetic_im_proc_container
Minimalistic example for a ROS package that runs an image processing node in a docker container.
This enables you to run newer versions of ROS and/or python while isolating your dependencies for your image processing modules from the rest of the system.

##  Usage:
* Clone this repo into your catkin workspace
* Compile your catkin workspace

```
   roscd noetic_im_proc_container
   sudo docker build -t ros_noetic_example .
   sudo docker run --rm -it --network host ros_noetic_example
     or if you want to mount this ros package in the docker container directly (currently preferred)
   sudo docker run -v "$(pwd)":/home/catkin_ws/src/noetic_im_proc_container --rm -it --network host ros_noetic_example
   catkin_make
   source ~/.bashrc
   rosrun noetic_im_proc_container image_service
```
