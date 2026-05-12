import cv2
import numpy as np

class AIModelWrapper:
    """
    Implement this class to run models in the camera pipeline.

    The result is packed in the `image_frame.ai_model_results` field.

    For example:

    ```
    class MyModelWrapper(AIModelWrapper):
        def name(self) -> str:
            return "My Model"

        def run_model(self, color_image: np.ndarray):
            return self.model(color_image)

        def visualize_results(self, color_image: np.ndarray, result_from_run_model) -> np.ndarray:
            return self.model.visualize_results(color_image, result_from_run_model)

    # Instantiate the custom model wrapper
    my_model = MyModelWrapper()

    # Start the left camera stream passing the AI model to the pipeline
    for image_frame in stream_left_camera(ai_models_to_use=[my_model]):
        if image_frame is None: 
            continue
            
        results = image_frame.ai_model_results[0]    
        annotated_image = my_model.visualize_results(image_frame.image.copy(), results)
            
        cv2.namedWindow(my_model.name(), cv2.WINDOW_NORMAL)
        cv2.imshow(my_model.name(), annotated_image)
        if cv2.waitKey(1) == ord('q'):
            break
    ```
    """
    
    def name(self) -> str:
        raise NotImplementedError("Please implement this method. Must return a model name.")

    def run_model(self, color_image: np.ndarray):
        raise NotImplementedError("Please implement this method. Must return inference results.")

    def visualize_results(self, color_image: np.ndarray, result_from_run_model) -> np.ndarray:
        raise NotImplementedError("Please implement this method. Must return an annotated ndarray image.")


def do_object_detection(color_image: np.ndarray, ai_models: list[AIModelWrapper], is_visualize: bool):
    """
    Runs object detection using the provided AI models wrappers.
    Returns the concatenated visualization image and a list of results.
    """
    output_image: np.ndarray = color_image
    results = []
    
    for wrapper in ai_models:
        result = wrapper.run_model(color_image)
        results.append(result)
    
    if is_visualize:
        input_image = color_image.copy()
        for wrapper, result in zip(ai_models, results):
            annotated_image = wrapper.visualize_results(input_image, result)
            
            cv2.putText(
                img=annotated_image, 
                text=wrapper.name(),
                org=(50,50), # Position text dynamically
                fontFace=cv2.FONT_HERSHEY_SIMPLEX, 
                fontScale=1, 
                color=(0, 0, 255), # Red color
                thickness=2
            )
            
            output_image = np.hstack((output_image, annotated_image))
            
    return output_image, results
