FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8

ARG MIPI_CAM_VERSION=2.4.0-jammy.20251208.035201
ARG HOBOT_CAMERA_VERSION=3.1.0-20251208164524
ARG HOBOT_MULTIMEDIA_VERSION=3.0.4-20251208164341
ARG TROS_BASE_VERSION=2.4.5-jammy.20251028.175810
ARG HBM_IMG_VERSION=2.0.0-jammy.20241225.031409
ARG HOBOT_SHM_VERSION=0.0.1-jammy.20241225.030528
ARG HOBOT_VIO_VERSION=2.1.1-jammy.20241225.023850

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        ca-certificates \
        curl \
        gnupg && \
    curl -fsSL \
        http://archive.d-robotics.cc/keys/sunrise.gpg \
        -o /usr/share/keyrings/sunrise.gpg && \
    echo "deb [arch=arm64 signed-by=/usr/share/keyrings/sunrise.gpg] http://archive.d-robotics.cc/ubuntu-rdk-x5 jammy main" \
        > /etc/apt/sources.list.d/sunrise.list && \
    apt-get update && \
    apt-get install -y --no-install-recommends --allow-downgrades \
        "hobot-multimedia=${HOBOT_MULTIMEDIA_VERSION}" \
        "hobot-camera=${HOBOT_CAMERA_VERSION}" \
        "tros-humble-base=${TROS_BASE_VERSION}" \
        "tros-humble-hbm-img-msgs=${HBM_IMG_VERSION}" \
        "tros-humble-hobot-shm=${HOBOT_SHM_VERSION}" \
        "tros-humble-hobot-vio=${HOBOT_VIO_VERSION}" \
        "tros-humble-mipi-cam=${MIPI_CAM_VERSION}" && \
    apt-mark hold \
        hobot-multimedia \
        hobot-camera \
        tros-humble-base \
        tros-humble-hbm-img-msgs \
        tros-humble-hobot-shm \
        tros-humble-hobot-vio \
        tros-humble-mipi-cam && \
    rm -rf /var/lib/apt/lists/*

COPY entrypoint.sh /camera_entrypoint.sh

RUN chmod +x /camera_entrypoint.sh

ENTRYPOINT ["/camera_entrypoint.sh"]
