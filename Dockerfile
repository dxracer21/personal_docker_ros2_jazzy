FROM ros:jazzy-ros-base-noble

ARG DEBIAN_FRONTEND=noninteractive
ARG USERNAME=jinsoo
ARG USER_UID=1000
ARG USER_GID=1000

SHELL ["/bin/bash", "-c"]

# 기본 개발 도구, USB 도구, GUI 도구 설치
RUN apt-get update && apt-get install -y --no-install-recommends \
    sudo \
    git \
    git-lfs \
    curl \
    wget \
    vim \
    nano \
    tmux \
    htop \
    tree \
    less \
    bash-completion \
    build-essential \
    cmake \
    pkg-config \
    python3-pip \
    python3-venv \
    python3-dev \
    python3-rosdep \
    python3-colcon-common-extensions \
    iproute2 \
    iputils-ping \
    net-tools \
    dnsutils \
    procps \
    lsof \
    usbutils \
    udev \
    v4l-utils \
    x11-apps \
    x11-utils \
    mesa-utils \
    ros-jazzy-desktop-full \
    ros-jazzy-rqt \
    ros-jazzy-rqt-common-plugins \
    ros-jazzy-rqt-image-view \
    ros-jazzy-rqt-graph \
    ros-jazzy-rviz2 \
    && rm -rf /var/lib/apt/lists/*

# 호스트와 동일한 UID/GID를 사용하는 jinsoo 계정 생성
RUN set -eux; \
    EXISTING_GROUP="$(getent group "${USER_GID}" | cut -d: -f1 || true)"; \
    if [ -n "${EXISTING_GROUP}" ]; then \
        if [ "${EXISTING_GROUP}" != "${USERNAME}" ]; then \
            groupmod --new-name "${USERNAME}" "${EXISTING_GROUP}"; \
        fi; \
    else \
        groupadd --gid "${USER_GID}" "${USERNAME}"; \
    fi; \
    EXISTING_USER="$(getent passwd "${USER_UID}" | cut -d: -f1 || true)"; \
    if [ -n "${EXISTING_USER}" ]; then \
        if [ "${EXISTING_USER}" != "${USERNAME}" ]; then \
            usermod \
                --login "${USERNAME}" \
                --home "/home/${USERNAME}" \
                --move-home \
                "${EXISTING_USER}"; \
        fi; \
        usermod --gid "${USER_GID}" --shell /bin/bash "${USERNAME}"; \
    else \
        useradd \
            --uid "${USER_UID}" \
            --gid "${USER_GID}" \
            --create-home \
            --shell /bin/bash \
            "${USERNAME}"; \
    fi; \
    echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" \
        > "/etc/sudoers.d/${USERNAME}"; \
    chmod 0440 "/etc/sudoers.d/${USERNAME}"

# USB, 시리얼, 카메라, GPU 장치 접근용 그룹
RUN for group in dialout video plugdev render; do \
        if getent group "${group}" >/dev/null; then \
            usermod -aG "${group}" ${USERNAME}; \
        fi; \
    done

# ROS2 작업공간 생성
RUN mkdir -p /home/${USERNAME}/ros2_ws/src \
    /home/${USERNAME}/ros2_ws/build \
    /home/${USERNAME}/ros2_ws/install \
    /home/${USERNAME}/ros2_ws/log \
    && chown -R ${USERNAME}:${USERNAME} /home/${USERNAME}

COPY entrypoint.sh /usr/local/bin/entrypoint.sh

RUN chmod +x /usr/local/bin/entrypoint.sh

ENV HOME=/home/${USERNAME}
ENV USER=${USERNAME}
ENV QT_X11_NO_MITSHM=1
ENV RCUTILS_COLORIZED_OUTPUT=1

USER ${USERNAME}

WORKDIR /home/${USERNAME}/ros2_ws

ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]

CMD ["sleep", "infinity"]
