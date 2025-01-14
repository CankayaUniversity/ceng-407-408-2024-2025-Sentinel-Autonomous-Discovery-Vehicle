FROM ros:jazzy

RUN apt-get update \
    && apt-get upgrade -y \
    && apt-get install -y vim wget libopencv-dev \
    && rm -rf /var/lib/apt/lists/

WORKDIR /sentinel

RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc