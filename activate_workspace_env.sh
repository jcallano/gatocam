#!/usr/bin/env bash

# Activate ROS 2 Jazzy overlay (redundant if ~/.bashrc already sources it).
if [ -f /opt/ros/jazzy/setup.bash ]; then
  # shellcheck source=/opt/ros/jazzy/setup.bash
  source /opt/ros/jazzy/setup.bash
else
  echo "warning: /opt/ros/jazzy/setup.bash not found" >&2
fi

if [ -f "$HOME/ros2_ws/install/setup.bash" ]; then
  # shellcheck source=install/setup.bash
  source "$HOME/ros2_ws/install/setup.bash"
else
  echo "warning: $HOME/ros2_ws/install/setup.bash not found" >&2
fi

# Activate workspace virtualenv
if [ -f "$HOME/ros2_ws/.venv/bin/activate" ]; then
  # shellcheck source=.venv/bin/activate
  source "$HOME/ros2_ws/.venv/bin/activate"
else
  echo "warning: $HOME/ros2_ws/.venv/bin/activate not found" >&2
fi
