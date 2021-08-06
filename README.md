# prior4_ros_container
Minimalistic example for a ROS package that runs an image processing node in a docker container.
This enables you to run newer versions of ROS and/or python while isolating your dependencies for your image processing modules from the rest of the system.

##  Usage:
* Clone this repo into your catkin workspace
* Compile your catkin workspace

```
   roscd prior4_ros_container
   sudo docker build -t prior4_ros_container .
   sudo docker run --rm -it --network host prior4_ros_container
     or if you want to mount this ros package in the docker container directly (currently preferred)
   sudo docker run -v "$(pwd)":/home/catkin_ws/src/prior4_ros_container --rm -it --network host prior4_ros_container
   catkin_make # in the docker container
   source ~/.bashrc # in the docker container
   rosrun prior4_ros_container image_service # in the docker container
```

## Testing the service

### Basic test node
This package also features a test client which loads a image from the filesystem into a openCV mat and sends it to the service.
You can find it in `scripts/image_client_py2` and execute it like so:

```
rosrun prior4_ros_container image_client_py2
```
This script can be run on the host to verify that you can call the service in the docker container.

### Topic-based test node
If you want to directly feed a ROS topic stream into the image service, you can find an example client `scripts/image_topic_client_py2`.
This node reads a synchronized RGB and Depth image tuple and sends it to the service.
You can use this node if you want to work directly on the camera streams of a robot or a bag file.
It can be run with:
```
rosrun prior4_ros_container image_topic_client_py2
```

Important: If you work with bag files, please check that the type of your image streams is `sensor_msgs/Image` and not `sensor_msgs/CompressedImage`.
Usually, we record bag files with `CompressedImage`s. In that case, you simply have to execute the following command to convert them:
```
roslaunch prior4_ros_container uncompress_and_throttle.launch 
```
