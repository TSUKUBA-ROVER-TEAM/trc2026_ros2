#!/bin/bash

# ROS 2 Launch等で終了した際に、子プロセスもすべて確実に終了させる
trap "killall go2rtc gst-launch-1.0 2>/dev/null; exit 0" EXIT INT TERM

DIR=$(ros2 pkg prefix trc2026_bringup)
GO2RTC_BIN="$DIR/lib/trc2026_bringup/go2rtc"
CONFIG="/tmp/go2rtc_temp.yaml"

# 念のため既存プロセスを終了
killall go2rtc gst-launch-1.0 2>/dev/null || true
sleep 1

cat <<EOF > "$CONFIG"
api:
  listen: ":1984"
  origin: "*"

rtsp:
  listen: ":8554"

streams:
  cam1: "ffmpeg:udp://127.0.0.1:1234?listen"
  cam2: "ffmpeg:udp://127.0.0.1:1235?listen"
EOF

"$GO2RTC_BIN" -config "$CONFIG" &
sleep 2

# Camera 1 (HiCamera: H.264 natively)
gst-launch-1.0 v4l2src device=/dev/v4l/by-id/usb-Huawei_HiCamera_12345678-video-index0 ! \
    video/x-h264,width=1280,height=720,framerate=30/1 ! \
    h264parse ! \
    mpegtsmux ! \
    udpsink host=127.0.0.1 port=1234 &

# Camera 2 (DV20 USB: MJPEG encoded to H.264)
gst-launch-1.0 v4l2src device=/dev/v4l/by-id/usb-Jieli_Technology_USB_Composite_Device-video-index0 ! \
    image/jpeg,width=1280,height=720,framerate=30/1 ! \
    jpegdec ! \
    videoconvert ! \
    x264enc tune=zerolatency bitrate=2000 speed-preset=ultrafast key-int-max=30 ! \
    video/x-h264,profile=baseline ! \
    h264parse config-interval=-1 ! \
    mpegtsmux ! \
    udpsink host=127.0.0.1 port=1235 &

wait
