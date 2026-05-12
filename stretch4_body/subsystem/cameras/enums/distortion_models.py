from enum import Enum, auto
import cv2
import numpy as np


class DistortionModels(Enum):
    equidistant = auto()
    equidistant_with_recompute_extrinsics = auto()
    rational_polynomial = auto()
    plumb_bob = auto()
    wide_angle = auto()
    omnidir = auto()

    def get_model_name(self):
        if self is DistortionModels.equidistant or self is DistortionModels.equidistant_with_recompute_extrinsics:
            return "equidistant" # 4 parameters, aka fisheye model in OpenCV
        if self is DistortionModels.rational_polynomial:
            return "rational_polynomial"  # 8 parameters
        if self is DistortionModels.plumb_bob:
            return "plumb_bob"  # 5 parameters, Brown-Conrady model
        if self is DistortionModels.wide_angle:
            return "wide_angle"  # 14 parameters
        if self is DistortionModels.omnidir:
            return "omnidir"
        
        raise NotImplementedError(f"{self} does not have a known model name")
    
    def is_fisheye(self):
        if self == DistortionModels.equidistant:
            return True
        elif self == DistortionModels.equidistant_with_recompute_extrinsics:
            return True
        elif self == DistortionModels.rational_polynomial:
            return False
        elif self == DistortionModels.plumb_bob:
            return False
        elif self == DistortionModels.wide_angle:
            return False
        elif self == DistortionModels.omnidir:
            return True

        raise NotImplementedError(f"Unknown distortion model: {self}")
        

    def get_initial_distortion_coefficients(self) -> np.ndarray:
        if self == DistortionModels.equidistant:
            return np.zeros((4, 1), dtype=np.float32)
        elif self == DistortionModels.equidistant_with_recompute_extrinsics:  # aka fisheye
            return np.zeros((4, 1), dtype=np.float32)
        elif self == DistortionModels.rational_polynomial:
            return np.zeros((8, 1), dtype=np.float32)
        elif self == DistortionModels.plumb_bob:
            return np.zeros((5, 1), dtype=np.float32)
        elif self == DistortionModels.wide_angle:
            return np.zeros((14, 1), dtype=np.float32)
        elif self == DistortionModels.omnidir:
            return np.zeros((4, 1), dtype=np.float32)

        raise NotImplementedError(f"Unknown distortion model: {self}")
    

    def get_initial_intrinsic_guess(self, width, height) -> np.ndarray:
        if self == DistortionModels.equidistant or self == DistortionModels.equidistant_with_recompute_extrinsics:
            cx = width / 2.0
            cy = height / 2.0
            f = width / np.pi
            K_guess = np.array([[f,   0.0, cx ],
                                [0.0, f,   cy ],
                                [0.0, 0.0, 1.0]], dtype=np.float64)
            return K_guess
        elif self == DistortionModels.rational_polynomial:
            return np.zeros((3, 3))
        elif self == DistortionModels.plumb_bob:
            return np.zeros((3, 3))
        elif self == DistortionModels.wide_angle:
            return np.zeros((3, 3))
        elif self == DistortionModels.omnidir:
            return np.zeros((3, 3))

        raise NotImplementedError(f"Unknown distortion model: {self}")

    def get_flags(self) -> int:
        if self == DistortionModels.equidistant:  # aka fisheye
            return cv2.fisheye.CALIB_USE_INTRINSIC_GUESS
        if self == DistortionModels.equidistant_with_recompute_extrinsics:  # aka fisheye
            return cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC + cv2.fisheye.CALIB_USE_INTRINSIC_GUESS + cv2.fisheye.CALIB_CHECK_COND + cv2.fisheye.CALIB_FIX_SKEW
        elif self == DistortionModels.rational_polynomial:
            return (cv2.CALIB_RATIONAL_MODEL
                | cv2.CALIB_FIX_ASPECT_RATIO
            )
        elif self == DistortionModels.plumb_bob:
            return 0
        elif self == DistortionModels.wide_angle:
            return (cv2.CALIB_RATIONAL_MODEL + cv2.CALIB_THIN_PRISM_MODEL + cv2.CALIB_TILTED_MODEL)
        elif self == DistortionModels.omnidir:
            return 0

        raise NotImplementedError(f"Unknown distortion model: {self}")
