FROM robot_sensors-camera:2.4.0-stable

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
      tros-humble-hobot-stereonet=2.5.5-jammy.20260331.045429 \
      tros-humble-hobot-stereonet-utils=2.3.1-jammy.20260312.095725 \
      tros-humble-dnn-node=2.5.9-jammy.20251114.092352 \
      hobot-models-basic=1.2.0 && \
    rm -rf /var/lib/apt/lists/*

COPY entrypoint.sh /depth_entrypoint.sh

RUN chmod +x /depth_entrypoint.sh

ENTRYPOINT ["/depth_entrypoint.sh"]
