FROM ros:humble-ros-base

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        ros-humble-rplidar-ros \
        ros-humble-slam-toolbox \
        ros-humble-foxglove-bridge \
        ros-humble-rosbridge-server \
        ros-humble-tf2-ros && \
    rm -rf /var/lib/apt/lists/*

COPY entrypoint.sh /mapping_entrypoint.sh
COPY healthcheck.sh /mapping_healthcheck.sh
COPY slam_params.yaml /config/slam_params.yaml
COPY dashboard_server.py /dashboard_server.py
COPY image_stream_server.py /image_stream_server.py
COPY dashboard /dashboard

RUN chmod +x /mapping_entrypoint.sh /mapping_healthcheck.sh /dashboard_server.py /image_stream_server.py

ENTRYPOINT ["/mapping_entrypoint.sh"]
