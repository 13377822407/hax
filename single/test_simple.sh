
WORKSPACE="/home/hax/roslearn/single"

docker run -it --rm \
  -v "$WORKSPACE:/workspace" \
  -w /workspace \
  --network=host \
  ros:jazzy \
  bash -c "
    # 激活ROS2环境
    source /opt/ros/jazzy/setup.bash
    source /workspace/install/setup.bash
    exec bash
  "


