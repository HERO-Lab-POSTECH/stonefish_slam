#!/bin/bash
# stonefish-ws 컨테이너 entrypoint — ROS 2 + 워크스페이스 overlay를 source 후 명령 실행.
# exec-form 유지(신호 전달). 정본: .omp/env/entrypoint.sh
set -e
source /opt/ros/humble/setup.bash
if [ -f /workspace/install/setup.bash ]; then
  source /workspace/install/setup.bash
fi
exec "$@"
