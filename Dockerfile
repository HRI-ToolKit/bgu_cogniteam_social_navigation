FROM ros:foxy



WORKDIR /bgu_social_navigation_ws/

RUN apt update && mdkir src && cd src && git clone https://github.com/HRI-ToolKit/bgu_cogniteam_social_navigation.git
RUN . /opt/ros/noetic/setup.sh && catkin_make

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT [ "/entrypoint.sh" ]

