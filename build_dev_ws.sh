#!/bin/bash
# 编译 dev_ws 中的项目

docker run --rm \
  -v /home/hax/roslearn:/workspace \
  -w /workspace/dev_ws \
  --network=host \
  ros:jazzy \
  bash -c "
    apt-get update && \
    apt-get install -y libopencv-dev ros-jazzy-opencv-apps && \
    source /opt/ros/jazzy/setup.bash && \
    echo '=== Starting colcon build ===' && \
    colcon build --packages-select learning_pkg_c && \
    echo '=== Build completed successfully ===' && \
    ls -la install/learning_pkg_c/lib/
  "
