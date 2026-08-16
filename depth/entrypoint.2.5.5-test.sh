#!/usr/bin/env bash
set -eo pipefail

source /opt/tros/humble/setup.bash

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-/config/fastdds/udp_only.xml}"
export FASTDDS_DEFAULT_PROFILES_FILE="${FASTDDS_DEFAULT_PROFILES_FILE:-/config/fastdds/udp_only.xml}"
export FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}"

MODEL_PATH="/opt/tros/humble/share/hobot_stereonet/config/DStereoV2.4_int8.bin"

if [[ ! -s "${MODEL_PATH}" ]]; then
    echo "[depth] INT8 model not found: ${MODEL_PATH}" >&2
    exit 1
fi

echo "[depth] model=${MODEL_PATH}"
echo "[depth] input=/image_combine_raw"
echo "[depth] output=/StereoNetNode/stereonet_depth"
echo "[depth] pointcloud=disabled visual=disabled save=disabled"
echo "[depth] infer_thread_num=1"

exec ros2 launch hobot_stereonet stereonet_model.launch.py \
    log_level:=warn \
    stereo_node_name:=StereoNetNode \
    stereonet_model_file_path:="${MODEL_PATH}" \
    stereo_image_topic:=/image_combine_raw \
    camera_info_topic:=/image_right_raw/camera_info \
    left_camera_info_topic:=/image_left_raw/camera_info \
    depth_image_topic:=/StereoNetNode/stereonet_depth \
    depth_camera_info_topic:=/StereoNetNode/stereonet_depth/camera_info \
    stereonet_frame_id:=camera_link \
    stereonet_frame_id_right:=camera_link_right \
    calib_method:=none \
    uncertainty_th:=-0.10 \
    infer_thread_num:=1 \
    publish_pcd_enabled:=False \
    publish_visual_enabled:=False \
    publish_origin_enable:=False \
    publish_rectify_bgr:=False \
    render_perf:=False \
    speckle_filter_enable:=False \
    pcl_filter_enable:=False \
    save_result_flag:=False \
    save_stereo_flag:=False \
    save_origin_flag:=False \
    save_disp_flag:=False \
    save_uncert_flag:=False \
    save_depth_flag:=False \
    save_visual_flag:=False \
    save_pcd_flag:=False
