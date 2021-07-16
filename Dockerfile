#   Usage:
#   Clone this repo into your catkin workspace
#   Compile your catkin workspace
#   roscd noetic_im_proc_container
#   sudo docker build -t ros_noetic_example .
#   sudo docker run --rm -it --network host ros_noetic_example
#     or if you want to mount this ros package in the docker container directly (currently preferred)
#   sudo docker run -v "$(pwd)":/home/catkin_ws/src/noetic_im_proc_container --rm -it --network host ros_noetic_example
#   catkin_make
#   source ~/.bashrc
#   rosrun noetic_im_proc_container image_service

# Thanks to Timothy patten, the author of a dockerized maskrccn implementation which heavly influenced the creation of this package


# will install a ubuntu 20.04
FROM ros:noetic

RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
RUN mkdir -p /home/catkin_ws/src
WORKDIR /home/catkin_ws/src
RUN /bin/bash -c '. /opt/ros/noetic/setup.bash; catkin_init_workspace /home/catkin_ws/src'

WORKDIR /home/catkin_ws
RUN /bin/bash -c '. /opt/ros/noetic/setup.bash; cd /home/catkin_ws; catkin_make'
RUN echo "source /home/catkin_ws/devel/setup.bash" >> ~/.bashrc

