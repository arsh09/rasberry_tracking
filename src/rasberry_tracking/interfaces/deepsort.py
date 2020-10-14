# Provides re-id features of bounding box detections
#  Raymond Kirk (Tunstill) Copyright (c) 2020
#  Email: ray.tunstill@gmail.com
#
# This file aims to interface github.com/RaymondKirk/fruit_tracking with github.com/RaymondKirk/rasberry_perception
from __future__ import absolute_import, division, print_function

import ros_numpy

from rasberry_perception.interfaces.fruitcast import FruitCastServer
from rasberry_perception.interfaces.registry import DETECTION_REGISTRY
from rasberry_perception.utility import function_timer


@DETECTION_REGISTRY.register_detection_backend("deepsort")
class DeepSortServer(FruitCastServer):
    def __init__(self, config_file, det_config_file, model_file=None, det_model_file=None):
        # Import fruit_tracking repo
        try:
            from detectron2.config import get_cfg
            from fruit_tracking.config import add_deepsort_config
            from fruit_tracking.logging import setup_ds_logger
            from fruit_tracking.deep import FeatureExtractor
        except ImportError:
            print("Please ensure you're in a correctly configured github.com/RaymondKirk/fruit_tracking environment\n")
            raise

        # Load ReID config and model
        self.cfg = get_cfg()
        add_deepsort_config(self.cfg)
        self.cfg.merge_from_file(config_file)
        self.cfg.freeze()
        if model_file:
            self.cfg.MODEL.WEIGHTS = model_file
        setup_ds_logger(self.cfg)
        self.feature_extractor = FeatureExtractor(self.cfg)

        # Configure the detectron2 model to provide bounding boxes
        super(DeepSortServer, self).__init__(det_config_file, det_model_file)

    @function_timer.interval_logger(interval=10)
    def get_detector_results(self, request):
        response = super(DeepSortServer, self).get_detector_results(request)
        if not response.status.OKAY or len(response.results.objects) == 0:
            return response

        try:
            # TODO: Run DeepSort tracker here not only feature extractor
            image = ros_numpy.numpify(request.image)
            bboxes = [[int(getattr(d.roi, o)) for o in ["x1", "x2", "y1", "y2"]] for d in response.results.objects]
            batched_images = [image[y1:y2, x1:x2] for x1, x2, y1, y2 in bboxes]
            feature_vectors, logits = self.feature_extractor(batched_images)
            for batch_idx, detection in enumerate(response.results.objects):
                detection.reid_vector = feature_vectors[batch_idx].ravel()
                detection.reid_logits = logits[batch_idx].ravel()
        except Exception as e:
            print("DeepSortServer error: ", e)
            response.status.ERROR = True
            response.status.OKAY = False

        return response
