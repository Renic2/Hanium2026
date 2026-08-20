FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        python3-serial \
        ros-humble-sensor-msgs && \
    rm -rf /var/lib/apt/lists/*

COPY ebimu_ros2_node.py /app/ebimu_ros2_node.py
COPY calibration_math.py /app/calibration_math.py
COPY entrypoint.sh /imu_entrypoint.sh

RUN chmod +x /app/ebimu_ros2_node.py /imu_entrypoint.sh

ENTRYPOINT ["/imu_entrypoint.sh"]
CMD ["python3", "/app/ebimu_ros2_node.py"]
