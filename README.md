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
   catkin_make # in the docker container
   source ~/.bashrc # in the docker container
   rosrun noetic_im_proc_container image_service # in the docker container
```

## Testing the service
This package also features a test client which loads a image from the filesystem into a openCV mat and sends it to the service.
You can find it in `scripts/image_client_py2` and execute it like so:

```
rosrun noetic_im_proc_container image_client_py2
```
This script can be run on the host to verify that you can call the service in the docker container.
