FROM paulrobinette/eece5560-jazzy-class-image:v1
ARG USERNAME=USERNAME
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Delete user if it exists in container (e.g Ubuntu Noble: ubuntu)
RUN if id -u $USER_UID ; then userdel `id -un $USER_UID` ; fi

# Create the user
RUN groupadd -f --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME
RUN apt-get update && apt-get upgrade -y
RUN apt-get install -y python3-pip emacs vim nano
ENV SHELL=/bin/bash

COPY --chmod=777 ./src /ros_ws/src/student_src

RUN chmod a+rw -R /ros_ws

RUN chown $USERNAME:$USERNAME -R /ros_ws

USER $USERNAME

WORKDIR /ros_ws

RUN . /class_ws/install/setup.sh && colcon build --symlink-install

USER $USERNAME
CMD ["/bin/bash"]

######### SOURCES:
# https://docs.ros.org/en/iron/How-To-Guides/Setup-ROS-2-with-VSCode-and-Docker-Container.html