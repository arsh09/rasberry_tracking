## Basis for running fruit_reid backend in a container.
## Extended from the rasberry_perception:detectron2 image.
##
## Prerequisites:
##   - docker
##   - nvidia driver
##   - nvidia container toolkit
##
## Build with:
##   docker build -t rasberry_perception:fruit_reid .
##
## Save with:
##   docker save rasberry_perception:fruit_reid | gzip > rasberry_perception_fruit_reid.tar.gz
##
## Run with:
##   docker run --network host --gpus all --name fruit_reid --rm -it rasberry_perception:fruit_reid
##

FROM rasberry_perception:detectron2

# TODO: fruit_tracking and rasberry_tracking are private repos, either add keys to docker or run the below commands in this dir first
# git clone https://github.com/RaymondKirk/fruit_tracking.git && git clone https://github.com/RaymondKirk/rasberry_tracking.git

## Meta information
MAINTAINER Raymond Kirk    "rkirk@lincoln.ac.uk"
MAINTAINER Nikolaus Wagner "nwagner@lincoln.ac.uk"

# Clone necessary repositories (clone rasberry_tracking locally)
WORKDIR /catkin_ws/src
COPY rasberry_tracking rasberry_tracking/
RUN rm -rf bayestracking && git clone https://github.com/RaymondKirk/bayestracking.git
# Refresh rasberry_perception repo
RUN rm -rf rasberry_perception && git clone https://github.com/RaymondKirk/rasberry_perception.git

# Install ros-dependencies and build the rasberry_tracking package
WORKDIR /catkin_ws
ENV ROS_DISTRO melodic
ENV TZ Europe/London
RUN DEBIAN_FRONTEND=noninteractive apt update && \
    rosdep update
RUN rosdep install --from-paths src --ignore-src -r -y && \
    catkin config --extend /opt/ros/$ROS_DISTRO && \
    catkin build rasberry_tracking

# Docker clean-up
RUN rm -rf /var/lib/apt/lists/*

# Install fruit tracking repo (clone locally and prep first)
WORKDIR /
COPY fruit_tracking /fruit_tracking/
RUN . detectron2_venv/bin/activate && \
    pip install --no-cache-dir -e fruit_tracking

## Add backends. These files can be obtained from Raymond (rkirk@lincoln.ac.uk) and require citation when used.
COPY bayes_model /bayes_model/

## Create entry point for image (default entry point looks for a start_backend.sh script that describes how to launch the backend)
COPY start_backend.sh .
CMD ["/bin/bash", "-c"]