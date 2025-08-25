# Examples:
# bash test.sh control
# bash test.sh calib


mode=${1}


cd ~/ros2_ws

colcon build
source ~/ros2_ws/install/setup.bash

sudo -E ~/ros2_ws/build/gv_package/MotorControl enp4s0 ${mode}
