# syntax=docker/dockerfile:1
# ============================================================================
# stonefish-ws 팀 개발 환경 — full-stack 단일 이미지 (sim + slam 공용)
#
# 정본(canonical): .omp/env/Dockerfile  (omp-env 관리)
# 배포 사본:       stonefish_sim/docker/Dockerfile, stonefish_slam/docker/Dockerfile
#   → 사본을 직접 수정하지 말고 정본을 고친 뒤 동기화할 것 (drift는 omp-env가 해시 대조로 탐지)
#
# 빌드(레포의 docker/ 디렉토리에서):
#   docker compose build
# 베이스 선택 근거: nvidia/cudagl·opengl 이미지는 유지 중단됨 → 현행 표준은
#   osrf/ros:<distro>-desktop + 호스트 nvidia-container-toolkit 런타임 주입.
#   (재현성을 digest 고정까지 올리려면 FROM에 @sha256:<digest>를 붙일 것 — DL3007)
# ============================================================================
FROM osrf/ros:humble-desktop

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

LABEL org.opencontainers.image.title="stonefish-ws dev environment" \
      org.opencontainers.image.description="ROS 2 Humble + Stonefish 1.3.0 + SLAM deps for HERO-Lab underwater robotics" \
      org.opencontainers.image.source="https://github.com/HERO-Lab-POSTECH/stonefish_sim" \
      org.opencontainers.image.licenses="GPL-3.0"

# ----------------------------------------------------------------------------
# 1) 시스템 의존성
#    - Stonefish 코어 빌드 의존(공식 문서 기준 3개): libglm-dev libsdl2-dev libfreetype6-dev
#    - stonefish_ros2/sim: pcl-conversions, image-transport (desktop에 없는 것 명시)
#    - stonefish_slam C++: gtsam, libpointmatcher, octomap, pybind11, eigen
#    - GUI/디버그: mesa-utils(glxinfo로 GPU 렌더러 확인), xvfb(헤드리스 폴백)
# ----------------------------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    git \
    libglm-dev \
    libsdl2-dev \
    libfreetype6-dev \
    ros-humble-pcl-conversions \
    ros-humble-pcl-ros \
    ros-humble-image-transport \
    ros-humble-tf2-geometry-msgs \
    ros-humble-tf-transformations \
    ros-humble-gtsam \
    ros-humble-libpointmatcher \
    ros-humble-octomap-msgs \
    liboctomap-dev \
    pybind11-dev \
    libeigen3-dev \
    libomp-dev \
    python3-opencv \
    python3-scipy \
    python3-sklearn \
    python3-matplotlib \
    python3-pip \
    python3-pytest \
    mesa-utils \
    xvfb \
    sudo \
    && rm -rf /var/lib/apt/lists/*

# Python 의존 (pip 전용 — apt로 해결 불가한 것만)
# ⚠️ gtsam: apt의 ros-humble-gtsam은 C++ 전용. Python 바인딩은 pip 필수 (slam README 참조)
# ⚠️ numpy는 system(1.21.5, apt) 유지 — 업그레이드 시 ROS cv_bridge 등과 ABI 충돌
RUN pip3 install --no-cache-dir \
    gtsam==4.2 \
    shapely \
    transforms3d

# ----------------------------------------------------------------------------
# 2) Stonefish 코어 라이브러리 (HERO fork, 1.3.0)
#    ⚠️ 버전 함정: stonefish_ros2는 find_package(Stonefish REQUIRED 1.3.0) —
#      CMake ExactVersion 정책이라 코어와 브리지 버전이 정확히 일치해야 함.
#      upstream master는 1.6.0-dev이므로 반드시 HERO fork를 사용한다.
#    MAKE_JOBS 기본 4: 저사양 빌드 호스트 OOM 예방 (필요시 --build-arg로 상향)
# ----------------------------------------------------------------------------
ARG STONEFISH_REPO=https://github.com/HERO-Lab-POSTECH/stonefish.git
ARG STONEFISH_REF=main
ARG MAKE_JOBS=4
RUN git clone --depth 1 --branch "${STONEFISH_REF}" "${STONEFISH_REPO}" /tmp/stonefish \
    && cmake -S /tmp/stonefish -B /tmp/stonefish/build -DCMAKE_BUILD_TYPE=Release \
    && cmake --build /tmp/stonefish/build -j "${MAKE_JOBS}" \
    && cmake --install /tmp/stonefish/build \
    && ldconfig \
    && rm -rf /tmp/stonefish

# ----------------------------------------------------------------------------
# 3) 비루트 사용자 (bind-mount 파일 소유권을 호스트 사용자와 일치)
#    UID/GID는 빌드 인자 — compose가 호스트 값(${UID}/${GID})을 넘긴다
# ----------------------------------------------------------------------------
ARG USERNAME=dev
ARG UID=1000
ARG GID=1000
RUN groupadd --gid "${GID}" "${USERNAME}" \
    && useradd --uid "${UID}" --gid "${GID}" -m -s /bin/bash "${USERNAME}" \
    && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME}

# Qt/X11: MIT-SHM은 컨테이너 경계에서 깨짐(BadAccess) → 비활성화
ENV QT_X11_NO_MITSHM=1

# 대화형 셸 편의: ROS + 워크스페이스 자동 source
RUN echo 'source /opt/ros/humble/setup.bash' >> /home/${USERNAME}/.bashrc \
    && echo '[ -f /workspace/install/setup.bash ] && source /workspace/install/setup.bash' >> /home/${USERNAME}/.bashrc

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

USER ${USERNAME}
WORKDIR /workspace
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
