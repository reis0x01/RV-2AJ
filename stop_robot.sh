#!/usr/bin/env bash
# Stop the RV-2AJ system started by ./start_robot.sh (the ROS stack).
# The GUI ignores the normal stop signal, so leftovers are force-closed and the
# script waits until everything is really gone (then ./start_robot.sh works at once).
systemctl --user kill --signal=SIGINT rv2aj-stack.scope 2>/dev/null
sleep 4
for pid in $(pgrep -f "lib/rv2aj_control/[a-z_]*") $(pgrep -x move_group) \
           $(pgrep -f "robot_state_publishe[r]") $(pgrep -f "ros2 launch rv2aj_contro[l]"); do
    kill -9 "$pid" 2>/dev/null
done
systemctl --user stop rv2aj-stack.scope 2>/dev/null
# Wait until the scope is really gone ("deactivating" still blocks a new start).
for _ in $(seq 1 30); do
    state=$(systemctl --user show -p ActiveState --value rv2aj-stack.scope 2>/dev/null)
    [ "$state" = "inactive" ] || [ "$state" = "failed" ] || [ -z "$state" ] && break
    sleep 1
done
systemctl --user reset-failed rv2aj-stack.scope 2>/dev/null
echo "RV-2AJ stopped."
