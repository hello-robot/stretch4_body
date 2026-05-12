from stretch4_body.subsystem.cameras.enums.aruco_dictionary import ArucoDictionary
import cv2
import cv2.aruco
from enum import Enum, auto
from dataclasses import dataclass, field

import numpy as np

@dataclass
class CharucoBoardConfig:
    size: tuple[int, int] # (Number of squares horizontally, Number of square vertically)
    square_length: float
    marker_length: float
    dictionary: ArucoDictionary
    aruco_start_id: int
    _aruco_end_id:int|None = None 

    charuco_detector: cv2.aruco.CharucoDetector = field(init=False)

    def set_charuco_detector(self, use_high_MP_corner_refinement: bool):
        self.charuco_detector = self.get_charuco_detector(use_high_MP_corner_refinement=use_high_MP_corner_refinement)
        return self

    def __repr__(self) -> str:
        return f"""
Number of squares: {self.size}
Square Size: {self.square_length}
Marker Size: {self.marker_length}
Dictionary: {self.dictionary.name}
ArUco Start ID: {self.aruco_start_id}
"""

    @property
    def aruco_end_id(self) -> int:
        if self._aruco_end_id is not None: return self._aruco_end_id
        return self.aruco_start_id + np.floor(self.size[0] * self.size[1] // 2)

    @property
    def number_of_inner_corners(self) -> int:
        return ((self.size[0]-1) * (self.size[1]-1))
    
    def check_valid_detection(self, charuco_ids, marker_ids):
        return charuco_ids is not None and len(charuco_ids) > 0 and np.min(marker_ids) >= self.aruco_start_id and np.max(marker_ids) <= self.aruco_end_id
    
    def check_enough_corners_detected(self,charuco_ids,  minimum_percentage_of_corners_required=0.55):
        return charuco_ids is not None and len(charuco_ids) >= int(self.number_of_inner_corners * minimum_percentage_of_corners_required)

    def get_board(self):
        aruco_board = cv2.aruco.CharucoBoard(
            size=self.size,
            squareLength=self.square_length,
            markerLength=self.marker_length,
            dictionary=self.dictionary.get_dictionary(),
            ids=np.arange(self.aruco_start_id, self.aruco_end_id, dtype=np.int32)
        )   
        
        if "DICT_4X4" in self.dictionary.name:
            aruco_board.setLegacyPattern(True)

        return aruco_board

    def get_charuco_detector(self, use_high_MP_corner_refinement) -> cv2.aruco.CharucoDetector:
        """
        `use_high_MP_corner_refinement` updates cornerRefinementMethod params to detect Charuco boards better with high resolution cameras.
        Note:
            When detecting markers for ChArUco boards, and specially when using homography (i.e. during camera calibration - when the camera matrix is not known), it is recommended to disable the corner refinement of markers.
            The reason of this is that, due to the proximity of the chessboard squares, the subpixel process can produce important deviations in the corner positions and these deviations are propagated to the ChArUco corner interpolation, producing poor results.
            https://docs.opencv.org/4.x/df/d4a/tutorial_charuco_detection.html
        """

        aruco_board = self.get_board()

        detector_parameters = cv2.aruco.DetectorParameters()

        detector_parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_APRILTAG

        if use_high_MP_corner_refinement:
            # These params were determined by trail and error
            # Params reference https://docs.opencv.org/4.x/d1/dcd/structcv_1_1aruco_1_1DetectorParameters.html
            detector_parameters.cornerRefinementWinSize = 21 # maximum window size for the corner refinement process (in pixels) (default 5).
            # detector_parameters.aprilTagDeglitch = 1        # should the thresholded image be deglitched? Only useful for very noisy images (default 0).
            # detector_parameters.aprilTagQuadSigma = 1.2    # what Gaussian blur should be applied to the segmented image (used for quad detection?[SIC])
            detector_parameters.aprilTagMinClusterPixels = 21 # reject quads containing too few pixels (default 5).
            # detector_parameters.markerBorderBits = 1 # number of bits of the marker border, i.e. marker border width (default 1).
            detector_parameters.adaptiveThreshWinSizeMax = 31 # maximum window size for adaptive thresholding before finding contours (default 23).
            # detector_parameters.minMarkerPerimeterRate = 0.02 # determine minimum perimeter for marker contour to be detected. This is defined as a rate respect to the maximum dimension of the input image (default 0.03).

        refine_parameters = cv2.aruco.RefineParameters()
        charuco_parameters = cv2.aruco.CharucoParameters()
        return cv2.aruco.CharucoDetector(
            aruco_board, charuco_parameters, detector_parameters, refine_parameters
        )


class CharucoBoards(Enum):
    """An enum of charuco boards used to with camera calibration"""
    BOARD_5x7_37mm_27mm_4x4_start_id_0 = auto()
    BOARD_5x7_37mm_27mm_4x4_start_id_20 = auto()
    BOARD_5x7_37mm_27mm_4x4_start_id_40 = auto()
    BOARD_5x7_30mm_22mm_4x4_start_id_0 = auto()
    def get_board_config(self, use_high_MP_corner_refinement: bool) -> CharucoBoardConfig:
        if self is CharucoBoards.BOARD_5x7_37mm_27mm_4x4_start_id_0: 
            return CharucoBoardConfig(
            size = (7, 5),
            square_length = 0.037,
            marker_length = 0.027,
            dictionary = ArucoDictionary.DICT_4X4_250,
            aruco_start_id=0
            ).set_charuco_detector(use_high_MP_corner_refinement)
        elif self is CharucoBoards.BOARD_5x7_37mm_27mm_4x4_start_id_20: 
            return CharucoBoardConfig(
            size = (7, 5),
            square_length = 0.037,
            marker_length = 0.027,
            dictionary = ArucoDictionary.DICT_4X4_250,
            aruco_start_id=20
            ).set_charuco_detector(use_high_MP_corner_refinement)
        elif self is CharucoBoards.BOARD_5x7_37mm_27mm_4x4_start_id_40: 
            return CharucoBoardConfig(
            size = (7, 5),
            square_length = 0.037,
            marker_length = 0.027,
            dictionary = ArucoDictionary.DICT_4X4_250,
            aruco_start_id=40
            ).set_charuco_detector(use_high_MP_corner_refinement)
        elif self is CharucoBoards.BOARD_5x7_30mm_22mm_4x4_start_id_0: 
            return CharucoBoardConfig(
            size = (7, 5),
            square_length = 0.030,
            marker_length = 0.022,
            dictionary = ArucoDictionary.DICT_4X4_250,
            aruco_start_id=0
            ).set_charuco_detector(use_high_MP_corner_refinement)
            
        raise NotImplementedError(f"{self} does not have a board definition.")
