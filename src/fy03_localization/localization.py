# Description: This file implements the outdoor localization algorithm which fuses UWB and IMU data.

#!/usr/bin/env python

class Localization:
    def __init__(self):
        self.fused_position = { 
            "x": 0,
            "y": 0,
            "z": 0
        }
        # May need to use Quaternions for orientation instead of Euler angles
        self.fused_orientation = {
            "heading": 0,
            "roll": 0,
            "pitch": 0
        }
        # From UWB.
        self.position = { 
            "x": 0,
            "y": 0,
            "z": 0
        }
        # From IMU.
        # May need to use Quaternions for orientation instead of Euler angles
        self.orientation = {
            "heading": 0,
            "roll": 0,
            "pitch": 0
        }
        self.linear_acceleration = { 
            "x": 0,
            "y": 0,
            "z": 0
        }
        self.angular_velocity = { 
            "x": 0,
            "y": 0,
            "z": 0
        }
