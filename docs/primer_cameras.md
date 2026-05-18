# RGB Cameras

## Preview from the Cameras from the command line

You can use the following tool to display the camera feeds in opencv, rerun and store them locally anywhere from the command line: [stretch_camera_show](../stretch4_body/tools/stretch_camera_show.py).

Run `stretch_camera_show --help` for all the available options.

Examples:

```
stretch_camera_show --left --opencv # Display the left head camera only

stretch_camera_show --right --opencv # Display the right head camera only

stretch_camera_show --center --opencv # Display the center head camera only

stretch_camera_show --left_right --opencv # Display the left, right head cameras only

stretch_camera_show --left_right_center --opencv # Display the left, right and center head cameras

stretch_camera_show --gripper --rerun # Display the gripper camera feeds and the point cloud from the stereo depth

stretch_camera_show --left --recording_directory ./recordings # Store images to disk
```

## Using the Cameras with Python


### Stream-API 

The camera subsystem exposes Python [generators](https://book.pythontips.com/en/latest/generators.html) for streaming frames from the cameras. These generators yield [ImageFrame](../stretch4_body/subsystem/cameras/models/image_frame.py) or [SyncedImageFrame](../stretch4_body/subsystem/cameras/models/image_frame.py) objects.

[ImageFrame](../stretch4_body/subsystem/cameras/models/image_frame.py) is a container for a single camera's image and metadata. Access the image using the `image` attribute, timestamp with `timestamp`, and [AI model results](#custom-ai-models-eg-rtmo-pose-estimation) with `ai_model_results`.

[SyncedImageFrame](../stretch4_body/subsystem/cameras/models/image_frame.py) is a container for multiple cameras' images and metadata. You can access the individual camera frames using the `left`, `right`, and `center` attributes of the `SyncedImageFrame` object, which are `ImageFrame` objects.

Using one camera yields an ImageFrame.

```python
from stretch4_body.subsystem.cameras import *

for image_frame in stream_left_camera():
    if image_frame is None: 
        print("No frame returned")
        continue
    print(f"Got image: {image_frame.image.shape=}, {image_frame.timestamp=}")
```


Using multiple cameras from the head cameras simultaneously yields a `SyncedImageFrame`.
```python
from stretch4_body.subsystem.cameras import *

for image_frame in stream_left_right_camera():
    if image_frame is None: 
        print("No frame returned")
        continue
    print(f"Got left image: {image_frame.left.image.shape=}, {image_frame.left.timestamp=}")
    print(f"Got right image: {image_frame.right.image.shape=}, {image_frame.right.timestamp=}")
```


Stream from the gripper RGBD camera. This yields a SyncedImageFrame and populates the pointcloud field.
```python
from stretch4_body.subsystem.cameras import *

for image_frame in stream_gripper_camera():
    if image_frame is None: 
        print("No frame returned")
        continue
    print(f"Got left image: {image_frame.left.image.shape=}, {image_frame.left.timestamp=}")
    if image_frame.pointcloud is not None:
        print(f"Got pointcloud image: {image_frame.pointcloud.shape=}")
```

## Preview RGBD Cameras from the command line

Run `stretch_rgbd_show --help` for all the available options.

Examples:

```
stretch_rgbd_show --left_right # Display the left, right head cameras only

stretch_rgbd_show --left_right_center # Display the left, right and center head cameras

stretch_rgbd_show --left # Display the left head camera only

stretch_rgbd_show --right # Display the right head camera only

stretch_rgbd_show --center # Display the center head camera only
```

If you have ROS2 launch files running for cameras or lidars, you can use the `--use_ros_for_cameras` or `--use_ros_for_lidar` flags to use the ROS2 data instead of direct python-api access to those sensors.

```
stretch_rgbd_show --left --use_ros_for_cameras --use_ros_for_lidar
```

## Using Emulated RGBD with Python

You can capture RGBD data from the head cameras using the following scripts:

```python
from stretch4_body.subsystem.cameras import *

for frame in stream_left_rgbd():
    if frame is None: 
        print("No frame returned")
        continue
    print(f"""Got a point cloud using the left camera and both lidar.
Number of points: {frame.pointcloud.shape[0]}
Depth size: {frame.depth_image.shape}""")

for frame in stream_left_right_center_rgbd():
    if frame is None: 
        print("No frame returned")
        continue
    print(f"""Got a point cloud using the left, right and center cameras and both lidar.
Left:
    Number of points: {frame.left.pointcloud.shape[0]}
    Depth size: {frame.left.depth_image.shape}
Right:
    Number of points: {frame.right.pointcloud.shape[0]}
    Depth size: {frame.right.depth_image.shape}
Center:
    Number of points: {frame.center.pointcloud.shape[0] if frame.center is not None else 'N/A'}
    Depth size: {frame.center.depth_image.shape if frame.center is not None else 'N/A'}""")
```

## Custom AI Models (e.g. RTMO Pose Estimation)

You can pass custom AI models to the camera pipeline by wrapping them in an `AIModelWrapper` instance. The pipeline will automatically route imagery to your model. The results are packed in the `image_frame.ai_model_results` field.

Here is an example wrapping an RTMO model:

First install 

```
git clone https://github.com/hello-robot/stretch4_human_pose_estimation.git
pip install ./stretch4_human_pose_estimation
human_pose_estimation_install_dependencies
human_pose_estimation_setup_models
human_pose_estimation_setup_models --size m
```

Then run this example:

```python
import cv2
import numpy as np
from stretch4_body.subsystem.cameras import stream_left_camera
from stretch4_body.subsystem.cameras.detectors.detector_ai_models import AIModelWrapper
from stretch4_human_pose_estimation.rtmo import RTMOPipeline

class RTMOWrapper(AIModelWrapper):

    def __init__(self):
        self.model = self.init_model()
        
    def name(self) -> str:
        return "RTMO Pose"

    def init_model(self):
        return RTMOPipeline(size="m", device="AUTO")

    def run_model(self, img: np.ndarray, conf_threshold: float = 0.5):
        return self.model.predict(img, conf_threshold=conf_threshold)

    def visualize_results(self, img: np.ndarray, result_from_run_model) -> np.ndarray:
        return self.model.visualize(img, result_from_run_model, kpt_thr=0.3, style="cvpr")

    @staticmethod
    def get_joint(person_res: dict, joint: BodyJoint) -> tuple[float, float]:
        """Returns the (x, y) coordinates of the specified joint for a single person."""
        kpts = person_res.get("keypoints", [])
        if joint < len(kpts):
            return float(kpts[joint][0]), float(kpts[joint][1])
        return 0.0, 0.0

    @staticmethod
    def get_joint_score(person_res: dict, joint: BodyJoint) -> float:
        """Returns the confidence score of the specified joint for a single person."""
        kpts = person_res.get("keypoints", [])
        if joint < len(kpts):
            return float(kpts[joint][2])
        return 0.0

# Instantiate the custom model wrapper
rtmo = RTMOWrapper()

# Start the left camera stream passing the AI model to the pipeline
for image_frame in stream_left_camera(ai_models_to_use=[rtmo]):
    if image_frame is None: 
        continue

    results = image_frame.ai_model_results[0]    

    annotated_image = rtmo.visualize_results(image_frame.image.copy(), results)
        
    cv2.namedWindow(rtmo.name(), cv2.WINDOW_NORMAL)
    cv2.imshow(rtmo.name(), annotated_image)
    if cv2.waitKey(1) == ord('q'):
        break
```

Here is another example wrapping a YOLOX object detection model (from rtmlib) configured to identify general COCO objects like cups, tables, desks, and chairs.

First install rtmlib:

```
pip install rtmlib
pip install openvino # To utilize the NPU on Stretch's NUC
pip install onnxruntime-openvino
```

The model weights will be automatically downloaded by rtmlib on the first run.

```python
import cv2
import numpy as np
from stretch4_body.subsystem.cameras import *
from stretch4_body.subsystem.cameras.detectors.detector_ai_models import AIModelWrapper

# RTMLIB imports
from rtmlib import YOLOX
from rtmlib.tools.base import RTMLIB_SETTINGS

# Configure rtmlib to utilize the Intel GPU or NPU via OpenVINO Execution Provider
RTMLIB_SETTINGS['onnxruntime']['npu'] = ('OpenVINOExecutionProvider', {'device_type': 'NPU'})
RTMLIB_SETTINGS['onnxruntime']['gpu'] = ('OpenVINOExecutionProvider', {'device_type': 'GPU'})


class YOLOXWrapper(AIModelWrapper):

    def __init__(self):
        self.coco_classes = [
            "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat", "traffic light", 
            "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep", "cow", 
            "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee", 
            "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove", "skateboard", "surfboard", 
            "tennis racket", "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple", 
            "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "couch", 
            "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse", "remote", "keyboard", 
            "cell phone", "microwave", "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase", 
            "scissors", "teddy bear", "hair drier", "toothbrush", "monitor", "can", "bottle", "tennis ball", "ball", "tape", "mug", "remote control", "desk"
        ]
        self.model = self.init_model()
        
    def name(self) -> str:
        return "YOLOX Object Detection"

    def init_model(self):
        device = 'gpu'  # npu, cpu, gpu
        backend = 'onnxruntime'  # We use onnxruntime to route to the OpenVINO NPU Provider
        # Provide a URL to a COCO yolox model. rtmlib will cache it automatically.
        url = 'https://github.com/Megvii-BaseDetection/YOLOX/releases/download/0.1.1rc0/yolox_tiny.onnx'
        return YOLOX(url, mode='multiclass',model_input_size=(416, 416), backend=backend, device=device)

    def run_model(self, img: np.ndarray):
        return self.model(img)

    def visualize_results(self, img: np.ndarray, result_from_run_model) -> np.ndarray:
        bboxes, cls_inds = result_from_run_model
        h, w = img.shape[:2]
        img_area = h * w
        
        for bbox, cls_id in zip(bboxes, cls_inds):
            x1, y1, x2, y2 = map(int, bbox[:4])
            area = (x2 - x1) * (y2 - y1)
            
            # Color based on relative size (Green for small, Red for large)
            ratio = min(1.0, max(0.0, np.sqrt(area / img_area)))
            hue = int((1.0 - ratio) * 120) 
            
            hsv = np.uint8([[[hue, 255, 255]]])
            color = tuple(map(int, cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)[0][0]))

            cv2.rectangle(img, (x1, y1), (x2, y2), color, 3)
            
            label = self.coco_classes[int(cls_id)] if int(cls_id) < len(self.coco_classes) else str(cls_id)
            label = f"{label.capitalize()}"
            
            # Render a solid background for the text
            font_scale = 0.8
            thickness = 2
            (t_w, t_h), baseline = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, font_scale, thickness)
            
            y_label_bg = max(y1, t_h + 10)
            cv2.rectangle(img, (x1, y_label_bg - t_h - 10), (x1 + t_w + 10, y_label_bg), color, -1)
            
            # Draw white text over the solid background (anti-aliased)
            cv2.putText(img, label, (x1 + 5, y_label_bg - 5), cv2.FONT_HERSHEY_SIMPLEX, font_scale, (255, 255, 255), thickness, cv2.LINE_AA)
            
        return img

# Instantiate the custom model wrapper
yolox_model = YOLOXWrapper()

# Start the left camera stream passing the AI model to the pipeline
# for image_frame in stream_center_camera(ai_models_to_use=[yolox_model]):
#     if image_frame is None: 
#         continue

#     results = image_frame.ai_model_results[0]    

#     annotated_image = yolox_model.visualize_results(image_frame.image.copy(), results)
        
#     cv2.namedWindow(yolox_model.name(), cv2.WINDOW_NORMAL)
#     cv2.imshow(yolox_model.name(), annotated_image)
#     if cv2.waitKey(1) == ord('q'):
#         break


for synced_frame in stream_gripper_camera(ai_models_to_use=[yolox_model]):
    if synced_frame is None: 
        continue
    image_frame =synced_frame.left

    results = image_frame.ai_model_results[0]    

    annotated_image = yolox_model.visualize_results(image_frame.image.copy(), results)
        
    cv2.namedWindow(yolox_model.name(), cv2.WINDOW_NORMAL)
    cv2.imshow(yolox_model.name(), annotated_image)
    if cv2.waitKey(1) == ord('q'):
        break
    
```

## Calibrating your cameras

Cameras are calibrated in the factory and the calibration files are stored in the robot's home directory, under `$HELLO_FLEET_PATH/$HELLO_FLEET_ID/calibration_cameras/`. 

If you need to re-calibrate your cameras, you can use the following tools. 

First focus your camera lens using [REx_camera_focus](../stretch4_body/tools/factory/REx_camera_focus.py).

Then calibrate the camera intrinsics and extrinsics using [REx_camera_calibrate](../stretch4_body/tools/factory/REx_camera_calibrate.py).

The calibration process does the following:
1. Calibrates the camera intrinsics using [calibrate_camera_intrinsics](../stretch4_body/subsystem/cameras/calibrate_intrinsics.py).
    This saves the calibration yaml file that contains `head_center`, `head_left`, and `head_right` keys with the K and D matrices, along with other information, at `$HELLO_FLEET_PATH/$HELLO_FLEET_ID/calibration_cameras/calibration_rgb_head_camera.yaml` and a few other yaml files for ROS2 to work correctly.
2. Verifies the camera intrinsics using [camera_intrinsics_validate_l2_distance](../stretch4_body/subsystem/cameras/camera_intrinsics_validate_l2_distance.py). This uses pre-tape-measured values to verify the camera intrinsics are correct. The values are expected to vary a little across robots, but it's a good "sanity check".
3. Calibrates the camera-camera extrinsics using [calibrate_extrinsics_cameras](../stretch4_body/subsystem/cameras/calibrate_extrinsics_cameras.py).
    This saves the calibration transforms as a yaml file containing `left_to_center` and `right_to_center` keys at `$HELLO_FLEET_PATH/$HELLO_FLEET_ID/calibration_cameras/camera_extrinsics.yaml`
4. Calibrates the camera-lidar extrinsics using [calibrate_extrinsics_lidars](../stretch4_body/subsystem/cameras/calibrate_extrinsics_lidars.py).
    This appends the camera-lidar extrinsics `transform_right_lidar_to_head_center` key to the `$HELLO_FLEET_PATH/$HELLO_FLEET_ID/calibration_dual_lidar/dual_lidar_calibration.yaml` file.

These values are used to estimate the distance to ArUco markers of known size using [detector_aruco.py](../stretch4_body/subsystem/cameras/detectors/detector_aruco.py) that uses `cv2.solve_pnp` or `cv2.fisheye.solve_pnp` depending on the lens type.

These values are also used by [emulated_rgbd.py](../stretch4_body/subsystem/cameras/emulated_rgbd.py) to create an colored point clouds and depth images using the left and right lidars, and each head camera using `cv2.projectPoints` or `cv2.fisheye.projectPoints` depending on the lens type. This also requires the lidar extrinsics to be calibrated using https://github.com/hello-robot/stretch_dual_lidar_calibration.