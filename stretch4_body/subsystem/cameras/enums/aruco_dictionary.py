import cv2
import cv2.aruco
from enum import Enum

class ArucoDictionary(Enum):
    """Enumeration of ArUco dictionaries mapping to OpenCV constants."""
    
    DICT_4X4_50 = cv2.aruco.DICT_4X4_50
    DICT_4X4_100 = cv2.aruco.DICT_4X4_100
    DICT_4X4_250 = cv2.aruco.DICT_4X4_250
    DICT_4X4_1000 = cv2.aruco.DICT_4X4_1000
    
    DICT_5X5_50 = cv2.aruco.DICT_5X5_50
    DICT_5X5_100 = cv2.aruco.DICT_5X5_100
    DICT_5X5_250 = cv2.aruco.DICT_5X5_250
    DICT_5X5_1000 = cv2.aruco.DICT_5X5_1000
    
    DICT_6X6_50 = cv2.aruco.DICT_6X6_50
    DICT_6X6_100 = cv2.aruco.DICT_6X6_100
    DICT_6X6_250 = cv2.aruco.DICT_6X6_250
    DICT_6X6_1000 = cv2.aruco.DICT_6X6_1000
    
    DICT_7X7_50 = cv2.aruco.DICT_7X7_50
    DICT_7X7_100 = cv2.aruco.DICT_7X7_100
    DICT_7X7_250 = cv2.aruco.DICT_7X7_250
    DICT_7X7_1000 = cv2.aruco.DICT_7X7_1000
    
    DICT_ARUCO_ORIGINAL = cv2.aruco.DICT_ARUCO_ORIGINAL
    
    # AprilTags
    DICT_APRILTAG_16H5 = cv2.aruco.DICT_APRILTAG_16h5
    DICT_APRILTAG_25H9 = cv2.aruco.DICT_APRILTAG_25h9
    DICT_APRILTAG_36H10 = cv2.aruco.DICT_APRILTAG_36h10
    DICT_APRILTAG_36H11 = cv2.aruco.DICT_APRILTAG_36h11
    
    # ArUco MIP
    DICT_ARUCO_MIP_36H12 = cv2.aruco.DICT_ARUCO_MIP_36h12

    @staticmethod
    def all_1000():
        return [ArucoDictionary.DICT_4X4_1000, ArucoDictionary.DICT_5X5_1000, ArucoDictionary.DICT_6X6_1000, ArucoDictionary.DICT_7X7_1000]

    @staticmethod
    def all_250():
        return [ArucoDictionary.DICT_4X4_250, ArucoDictionary.DICT_5X5_250, ArucoDictionary.DICT_6X6_250, ArucoDictionary.DICT_7X7_250]

    def get_dictionary(self):
        return cv2.aruco.getPredefinedDictionary(self.value)

    def get_aruco_detector(self, *, detector_params: cv2.aruco.DetectorParameters|None = None):
        aruco_parameters = cv2.aruco.DetectorParameters() if detector_params is None else detector_params

        return cv2.aruco.ArucoDetector(self.get_dictionary(), aruco_parameters)
