# Provides re-id features of bounding box detections
#  Raymond Kirk (Tunstill) Copyright (c) 2021
#  Email: ray.tunstill@gmail.com
#
# This file aims to interface trackers/detectors with github.com/RaymondKirk/rasberry_perception
from __future__ import absolute_import, division, print_function

from rasberry_perception.interfaces.fruitcast import FruitCastServer
from rasberry_perception.interfaces.registry import DETECTION_REGISTRY
from rasberry_perception.utility import function_timer


@DETECTION_REGISTRY.register_detection_backend("DefaultRasberryTracker")
class DefaultRasberryTracker(FruitCastServer):
    def __init__(self, det_config_file, det_model_file=None):
        # Custom code here to generate a tracker
        raise NotImplementedError("Please write a custom tracking interface")
        # Configure the rasberry_perception chosen backend to operate under the tracker (det generation)
        super(DefaultRasberryTracker, self).__init__(det_config_file, det_model_file)

    @function_timer.interval_logger(interval=10)
    def get_detector_results(self, request):
        raise NotImplementedError("Please write a custom tracking interface")
        response = super(DefaultRasberryTracker, self).get_detector_results(request)
        # Run a custom tracker here
        return response
