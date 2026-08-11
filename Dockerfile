FROM robotis/open-manipulator:5.0.0

ARG USERNAME=jinsoo
ARG USER_UID=1000
ARG USER_GID=1000

SHELL ["/bin/bash", "-c"]

# The ROBOTIS image already includes OMY and most MoveIt packages.
# Add only the Python binding used by the target tracker.
RUN set -eux; \
    if ! dpkg -s ros-jazzy-moveit-py >/dev/null 2>&1; then \
        apt-get update && \
        apt-get install -y --no-install-recommends ros-jazzy-moveit-py && \
        rm -rf /var/lib/apt/lists/*; \
    fi

# Keep the official OMY/MoveIt workspace from the base image, and add a
# host-matching user for this personal workspace.
RUN set -eux; \
    if ! command -v sudo >/dev/null 2>&1; then \
        apt-get update && \
        apt-get install -y --no-install-recommends sudo && \
        rm -rf /var/lib/apt/lists/*; \
    fi; \
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
    echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > "/etc/sudoers.d/${USERNAME}"; \
    chmod 0440 "/etc/sudoers.d/${USERNAME}"; \
    for group in dialout video plugdev render; do \
        if getent group "${group}" >/dev/null; then \
            usermod -aG "${group}" "${USERNAME}"; \
        fi; \
    done; \
    mkdir -p \
        "/home/${USERNAME}/ros2_ws/src" \
        "/home/${USERNAME}/ros2_ws/build" \
        "/home/${USERNAME}/ros2_ws/install" \
        "/home/${USERNAME}/ros2_ws/log"; \
    chown -R "${USERNAME}:${USERNAME}" "/home/${USERNAME}"; \
    if [ -d /root/ros2_ws ]; then \
        chmod o+x /root; \
        chmod -R a+rX /root/ros2_ws; \
    fi

COPY entrypoint.sh /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/entrypoint.sh

ENV HOME=/home/${USERNAME}
ENV USER=${USERNAME}
ENV QT_X11_NO_MITSHM=1
ENV RCUTILS_COLORIZED_OUTPUT=1
ENV PERSONAL_WS=/home/${USERNAME}/ros2_ws
ENV OPEN_MANIPULATOR_WS=/root/ros2_ws
ENV AUTO_COLCON_BUILD=1

USER ${USERNAME}
WORKDIR /home/${USERNAME}/ros2_ws

RUN { \
    echo ''; \
    echo '# ROS 2 Jazzy + ROBOTIS OpenMANIPULATOR environment'; \
    echo 'source /opt/ros/jazzy/setup.bash'; \
    echo 'if [ -f /root/ros2_ws/install/setup.bash ]; then'; \
    echo '    source /root/ros2_ws/install/setup.bash'; \
    echo 'fi'; \
    echo 'if [ -f "$HOME/ros2_ws/install/setup.bash" ]; then'; \
    echo '    source "$HOME/ros2_ws/install/setup.bash"'; \
    echo 'fi'; \
    } >> /home/${USERNAME}/.bashrc

ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["sleep", "infinity"]
