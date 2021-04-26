#!/usr/bin/env python

#  Raymond Kirk (Tunstill) Copyright (c) 2021
#  Email: ray.tunstill@gmail.com

# Calls the detection service in rasberry_perception but allows interfaces in rasberry_tracking to be used also

import rasberry_tracking.interfaces  # Import just to ensure all backends registered
from rasberry_perception.detection_server import __detection_server_runner


if __name__ == '__main__':
    __detection_server_runner()
