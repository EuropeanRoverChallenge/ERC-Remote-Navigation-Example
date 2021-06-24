FROM ghcr.io/europeanroverchallenge/erc-remote-image-base:latest

# Install additional packages
RUN apt-get update && apt-get -y upgrade && apt-get -y install \
  tmux \
  && rm -rf /var/lib/apt/lists/*

# Copy packages and build the workspace
WORKDIR /catkin_ws
COPY src ./src
RUN apt-get update \
  && rosdep update \
  && rosdep install --from-paths src -iy \
  && rm -rf /var/lib/apt/lists/*
RUN catkin config --extend /opt/ros/melodic && catkin build

# Install start script
COPY ./start.sh /

CMD ["/start.sh"]
