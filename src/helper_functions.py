#!/usr/bin/python3


# from itertools import chain
import pybullet as p
import numpy as np
from numpy import sin, cos, pi
import logging

# class Helper():
#     def __init__(self) -> None:
#         pass
    
#     @staticmethod
def load_mesh(mesh_setup = None):

    mesh_id = p.loadURDF(mesh_setup["path"], useFixedBase=mesh_setup["fixed"], 
                            basePosition=mesh_setup["position"], 
                            baseOrientation=mesh_setup["orientation"],
                            globalScaling=mesh_setup["scaling"], 
                            flags=p.URDF_USE_SELF_COLLISION|p.URDF_USE_SELF_COLLISION_INCLUDE_PARENT)
    
    return mesh_id


def colored_logging(name: str):

    logger = logging.getLogger(name=name)
    handler = logging.StreamHandler()
    handler.setFormatter(ColorFormatter())
    logger.addHandler(handler)
    logger.setLevel(logging.INFO)
    logger.propagate = False
    return logger

class ColorFormatter(logging.Formatter):
    """add color to pythons logging.

    """
    red = '\033[91m'
    bold_red = '\033[91m1m'
    green = '\033[92m'
    yellow = '\033[93m'
    blue = '\033[94m'
    reset = '\033[0m'
    format = "\n%(levelname)s | %(name)s | %(message)s\n"
    FORMATS = {logging.DEBUG: blue + format + reset,
                        logging.INFO: green + format + reset,
                        logging.WARNING: yellow + format + reset,
                        logging.ERROR: red + format + reset,
                        logging.CRITICAL: bold_red + format + reset
                        }
    def format(self, record):
        log_fmt = self.FORMATS.get(record.levelno)
        formatter = logging.Formatter(log_fmt)
        return formatter.format(record)