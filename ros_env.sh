docker run -it --rm \
  -v $(pwd):/workspace \
  -w /workspace \
  reg.ainirobot.com/vision/focal:ros-foxy \
  bash -c "source /opt/ros/foxy/setup.bash && exec bash"
