FROM ros:humble-ros-base

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        ros-humble-rplidar-ros && \
    rm -rf /var/lib/apt/lists/*

ENTRYPOINT ["/ros_entrypoint.sh"]

CMD ["ros2", "launch", "rplidar_ros", "rplidar_a1_launch.py", \
     "serial_port:=/dev/rplidar", \
     "serial_baudrate:=115200", \
     "frame_id:=laser_frame", \
     "inverted:=false", \
     "angle_compensate:=true", \
     "scan_mode:=Standard"]
