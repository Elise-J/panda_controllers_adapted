ARG BASE_IMAGE_TAG=noetic
FROM ghcr.io/aica-technology/ros-ws:${BASE_IMAGE_TAG}

WORKDIR /tmp

ARG HOST_GID=1001
ENV USER_GROUP=${USER}

USER root
RUN if [ "${HOST_GID}" != "1000" ];\
    then groupadd --gid ${HOST_GID} host_group && \
    usermod ${USER} -g ${HOST_GID} && \ 
    usermod ${USER} -a -G ${USER_GROUP}; fi
USER ${USER}

RUN echo "deb [arch=amd64] http://robotpkg.openrobots.org/packages/debian/pub $(lsb_release -cs) robotpkg" | sudo tee /etc/apt/sources.list.d/robotpkg.list
RUN curl http://robotpkg.openrobots.org/packages/debian/robotpkg.key | sudo apt-key add -

RUN sudo apt-get update -y
RUN sudo apt-get install ros-noetic-ros-controllers ros-noetic-gazebo-ros-control -y

# Libfranka
RUN sudo apt-get update && sudo apt-get install -y build-essential cmake git libpoco-dev libeigen3-dev
WORKDIR /home/${USER}/ros_ws/src
RUN git clone --recursive https://github.com/frankaemika/libfranka
RUN cd libfranka && git checkout 0.8.0 && git submodule update && mkdir build
WORKDIR /home/${USER}/ros_ws/src/libfranka/build
RUN cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF .. && cmake --build . && make -j && sudo make install -j && sudo ldconfig
WORKDIR ${HOME}

# Lib franka ROS
WORKDIR /home/${USER}/ros_ws/src
RUN git clone --recursive https://github.com/frankaemika/franka_ros --branch noetic-devel
RUN cd franka_ros && git checkout 0.8.0 && git submodule update
WORKDIR /home/${USER}/ros_ws
RUN rosdep install --from-paths src --ignore-src --rosdistro noetic -y --skip-keys libfranka
RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash; catkin_make -DCMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=/home/${USER}/libfranka/build"

WORKDIR /home/${USER}/ros_ws
COPY --chown=${USER} ./panda_controllers ./src/panda_controllers

ENV PYTHONPATH "${PYTHONPATH}:/opt/openrobots/lib/python3.8/site-packages/"
RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash; catkin_make"



