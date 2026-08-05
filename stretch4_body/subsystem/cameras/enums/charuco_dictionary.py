import logging

logger = logging.getLogger(__name__)
from stretch4_body.subsystem.cameras.enums.aruco_dictionary import ArucoDictionary
import cv2
import cv2.aruco
from enum import Enum, auto
from dataclasses import dataclass, field
import numpy as np
from PIL import Image

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
    

    def generate_pdf(self, filename: str|None = None, page_width_mm: float|None = 279.4, page_height_mm: float|None = 215.9, dpi: int = 300):
        """
        Generates a 1:1 scale printable PDF of the ChArUco board.
        Defaults to US Letter Landscape dimensions (279.4 x 215.9 mm).
        """
        if filename is None:
            filename = self.name.lower() + ".pdf"

        config = self.get_board_config(use_high_MP_corner_refinement=False)

        # Convert class parameters (meters) to millimeters for rendering
        square_length_mm = config.square_length * 1000.0
        cols, rows = config.size
        
        # Calculate pixel scale factors based on target DPI
        mm_to_inch = 25.4
        pixels_per_mm = dpi / mm_to_inch

        # Calculate total board pixels directly to eliminate per-square rounding drift
        board_img_width = int(cols * square_length_mm * pixels_per_mm)
        board_img_height = int(rows * square_length_mm * pixels_per_mm)
        
        logger.info(f"""Board Physical Size: {square_length_mm*cols:.1f}mm x {square_length_mm*rows:.1f}mm
{square_length_mm*cols/mm_to_inch:.2f}in x {square_length_mm*rows/mm_to_inch:.2f}in
Board Pixel Size: {board_img_width}px x {board_img_height}px""")

        # Handle None values safely together, falling back to a tight-fit board layout
        if page_width_mm is None or page_height_mm is None:
            logger.warning(f"page_width_mm or page_height_mm is None, defaulting to US letter paper tight-fit layout")
            page_width_mm = cols * square_length_mm
            page_height_mm = rows * square_length_mm

        # Convert final page boundaries to pixels
        page_width_px = int(page_width_mm * pixels_per_mm)
        page_height_px = int(page_height_mm * pixels_per_mm)

        # Validate that the board actually fits on the specified paper
        if board_img_width > page_width_px or board_img_height > page_height_px:
            raise ValueError(f"Board dimensions ({board_img_width}x{board_img_height} px) exceed paper dimensions ({page_width_px}x{page_height_px} px).")

        # Generate board image using the existing get_board() method
        board = config.get_board()
        board_img = board.generateImage((board_img_width, board_img_height))

        # Create a blank white page
        page_img = np.ones((page_height_px, page_width_px), dtype=np.uint8) * 255

        # Center the board on the page
        x_offset = (page_width_px - board_img_width) // 2
        y_offset = (page_height_px - board_img_height) // 2
        page_img[y_offset:y_offset+board_img_height, x_offset:x_offset+board_img_width] = board_img

        # Convert to Pillow Image and save
        pil_img = Image.fromarray(page_img)
        pil_img.save(filename, "PDF", resolution=dpi)
        logger.info(f"Generated {filename} formatted for {page_width_mm:.1f}x{page_height_mm:.1f}mm paper.")
