#!/bin/bash
set -e

# RUN detectron2 backend
source "/detectron2_venv/bin/activate" && \
rosrun rasberry_tracking detection_server.py --backend fruit_reid --config-file /bayes_model/run.yaml --det-config-file /r50_packaged/config.yaml
