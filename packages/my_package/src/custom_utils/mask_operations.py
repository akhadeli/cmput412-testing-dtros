import numpy as np 
import math

class MaskOperations:
    _instance = None

    def __new__(cls):
        if cls._instance is None:
            cls._instance = super(MaskOperations, cls).__new__(cls)
        return cls._instance

    @staticmethod
    def getActiveCount(mask):
        return np.count_nonzero(mask)
    
    @staticmethod
    def getActiveCenter(mask):
        if mask is None:
            return (-math.inf, -math.inf)

        y_coords, x_coords = np.where(mask > 0)

        if len(x_coords) == 0 or len(y_coords) == 0:
            return (-math.inf, -math.inf)

        center_x = int(np.mean(x_coords))
        center_y = int(np.mean(y_coords))

        return (center_x, center_y)

    @staticmethod
    def computeErrorInAxisX(mask, target_x, pixel_value):
        y_coords, x_coords = np.where(mask > 0)
        
        if len(x_coords) == 0:
            return 0

        errors = (x_coords - target_x) * pixel_value

        return np.sum(errors)