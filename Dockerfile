FROM kristoferb/spweb_ros2:galactic

RUN apt-get update -qqy \
    && apt-get install -qqy \
       ros-galactic-xacro \
       nano 

COPY ./ia_task_2_bringup /ros/src/ia_task_2_bringup/
COPY ./ia_task_2_scene /ros/src/ia_task_2_scene/

RUN cd /ros/src \
    && git clone --depth 1 https://github.com/sequenceplanner/ur_script_driver.git \
    && git clone --depth 1 https://github.com/sequenceplanner/gui_tools.git \
    && git clone --depth 1 https://github.com/sequenceplanner/viz_tools.git \
    && git clone --depth 1 https://github.com/sequenceplanner/tf_tools.git \
    && git clone --depth 1 https://github.com/sequenceplanner/ur_tools.git \
    && git clone --depth 1 https://github.com/sequenceplanner/ur_script_msgs.git \
    && . /opt/ros/$ROS_DISTRO/setup.sh \
    && cd /ros/ \
    && colcon build

COPY launch.bash /
RUN chmod +x /launch.bash &&\
    echo "source /opt/ros/galactic/setup.bash" >> ~/.bashrc &&\
    echo "source /ros/install/setup.bash" >> ~/.bashrc