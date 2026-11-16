#!/usr/bin/env bash
# Start the web-app agent on the TurtleBot's Pi.
# The token lives in ~/.turtlebot_agent_env (AGENT_TOKEN=...), readable only by you: chmod 600.
set -e
if [ -f /etc/turtlebot4/setup.bash ]; then
  source /etc/turtlebot4/setup.bash   # the robot's own ROS environment (domain, RMW, discovery)
else
  source /opt/ros/jazzy/setup.bash
fi
if [ -f "$HOME/.turtlebot_agent_env" ]; then
  set -a
  source "$HOME/.turtlebot_agent_env"
  set +a
fi
exec python3 "$(dirname "$(readlink -f "$0")")/agent.py" "$@"
