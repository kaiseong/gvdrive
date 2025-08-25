# Examples:
# bash test.sh control noboard
# bash test.sh calib raihub


mode=${1}

connect=${2}

cd ~/ros2_ws

colcon build
source ~/ros2_ws/install/setup.bash

sudo -E ~/ros2_ws/build/gv_package/MotorControl enp4s0 ${mode} ${connect}
