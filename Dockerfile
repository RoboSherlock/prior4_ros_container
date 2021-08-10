# Thanks to Timothy patten, the author of a dockerized maskrccn implementation which heavly influenced the creation of this package


# will install a ubuntu 20.04
FROM ros:noetic

RUN apt update && apt install -y tmux vim ros-noetic-tf ros-noetic-cv-bridge python3-pip git-all git-lfs
RUN git lfs install

RUN mkdir -p /home/catkin_ws/python_src
WORKDIR /home/catkin_ws/python_src
RUN git clone "https://github.com/RoboSherlock/prior4_detection.git"
RUN pip install /home/catkin_ws/python_src/prior4_detection

RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
RUN mkdir -p /home/catkin_ws/src
WORKDIR /home/catkin_ws/src
RUN /bin/bash -c '. /opt/ros/noetic/setup.bash; catkin_init_workspace /home/catkin_ws/src'

WORKDIR /home/catkin_ws
RUN /bin/bash -c '. /opt/ros/noetic/setup.bash; cd /home/catkin_ws; catkin_make'
RUN echo "source /home/catkin_ws/devel/setup.bash" >> ~/.bashrc

