export DISPLAY
export WAYLAND_DISPLAY
export XDG_RUNTIME_DIR

docker run -it --rm \
  -v /home/hax/roslearn:/workspace \
  -w /workspace/dev_ws \
  --device=/dev/video0:/dev/video0 --group-add video \
  --network=host \
  -e DISPLAY \
  -e WAYLAND_DISPLAY \
  -e XDG_RUNTIME_DIR \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v /mnt/wslg:/mnt/wslg \
  ros:jazzy \
  bash -c "source /opt/ros/jazzy/setup.bash && exec bash"