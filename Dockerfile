FROM ubuntu:20.04



# Set the non-interactive mode for apt
ARG DEBIAN_FRONTEND=noninteractive

# Set the keyboard layout to English (29)
RUN echo "keyboard-configuration keyboard-configuration/layout select English (US)" | debconf-set-selections && \
    echo "keyboard-configuration keyboard-configuration/layoutcode select 29" | debconf-set-selections

RUN    locale  # check for UTF-8 
RUN    apt update &&  apt install locales 
RUN    locale-gen en_US en_US.UTF-8 
RUN    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
RUN    export LANG=en_US.UTF-8 
RUN    apt install software-properties-common -y  
RUN    add-apt-repository universe -y  
RUN    apt update &&  apt install curl -y  
RUN    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg 
RUN    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null 
RUN    apt update 
RUN    apt-get install git -y 
RUN    apt install ros-foxy-desktop python3-argcomplete -y 
RUN    apt install python3-pip -y && pip3 install -U colcon-common-extensions 
RUN    pip3 install Pillow
RUN    apt-get install ros-foxy-pcl-ros -y && apt-get install ros-foxy-gazebo-ros-pkgs -y && apt-get install ros-foxy-ompl -y && apt-get  install libceres-dev -y &&  apt-get install ros-foxy-test-msgs -y && apt-get install ros-foxy-behaviortree-cpp-v3 -y && apt-get install graphicsmagick libgraphicsmagick++1-dev -y
RUN    apt-get install python3-pil python3-pil.imagetk -y
RUN    apt update 
    
WORKDIR /bgu_social_navigation_ws/src
RUN    apt update 


RUN git clone https://github.com/HRI-ToolKit/bgu_cogniteam_social_navigation.git && cd ..
WORKDIR /bgu_social_navigation_ws/

RUN . /opt/ros/foxy/setup.sh && colcon build --symlink-install --packages-skip social_navigation_launch social_navigation_manager social_navigation_ui

# Set up an entry point script
COPY entrypoint.sh /bgu_social_navigation_ws/entrypoint.sh
RUN chmod +x /bgu_social_navigation_ws/entrypoint.sh

# Entrypoint script
ENTRYPOINT ["/bgu_social_navigation_ws/entrypoint.sh"]

RUN . /opt/ros/foxy/setup.sh . && . /bgu_social_navigation_ws/install/setup.sh && colcon build --symlink-install --packages-select social_navigation_launch social_navigation_manager social_navigation_ui


