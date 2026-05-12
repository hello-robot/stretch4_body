"""
Writes the calibration created using `calibrate_rgb.py` from ~/stretch_user/stretch-se4-xxxx/calibration_rgb_head_camera.yaml to the eeprom of a Luxonis camera. This was tested with DepthAI==3.1.0.
"""
from pathlib import Path
import depthai as dai
import datetime
import argparse

from stretch4_body.subsystem.cameras.adapters.luxonis_camera_adapter import LuxonisCameraAdapter
from stretch4_body.subsystem.cameras.enums.rgb_camera import RGBCameras
def read_calibration_and_write_to_eeprom(camera_type: RGBCameras, clear_eeprom: bool):
    print(f"Connecting to device to retrieve calibration for {camera_type}...")

    camera_socket = LuxonisCameraAdapter.get_depthai_camera_socket(camera_type)



    camera_calibration = camera_type.load_calibration()
    
    with dai.Device(dai.UsbSpeed.SUPER_PLUS) as device:

        if clear_eeprom:
            print("Clearing EEPROM before writing...")
            device.factoryResetCalibration()
            print("EEPROM cleared.")

        calibration_handler = device.readCalibration()

        # new_distortion_model = "rational_polynomial" if camera_calibration.is_fisheye else "radial_division"
        new_distortion_model = dai.CameraModel.Fisheye if camera_calibration.distortion_model.is_fisheye() else dai.CameraModel.Perspective

        new_config = f"""
                {camera_calibration.camera_matrix.tolist()=}
                {camera_calibration.distortion_coefficients.tolist()=}
                {camera_calibration.distortion_model.is_fisheye()=}
                {camera_calibration.width=} {camera_calibration.height=}
                {new_distortion_model=}
                {new_distortion_model.value=}
        """

        try:
            print(f"""
                


    This script will replace the following calibration on the EEPROM:
                {calibration_handler.getCameraIntrinsics(camera_socket)=}
                {calibration_handler.getDistortionCoefficients(camera_socket)=}
                {calibration_handler.getDistortionModel(camera_socket)=}

                

    With the following calibration: {new_config}

    """)
        except IndexError:
            print(f"""
    No User Calibration found on the EEPROM.
                  
    Writing the following calibration: {new_config}
                  
                """)
        answer = input("Do you want to continue? (yes/no): ").lower()
        if answer[0] != "y":
            print("Aborting without writing to EEPROM.")
            return
        
        eeepromData = calibration_handler.getEepromData()
        print(f'EEPROM VERSION being flashed is {eeepromData.version}')
        date_time_string = datetime.datetime.now().strftime("%m_%d_%y_%H_%M")
        file_name = f"luxonis_eeprom_backup_{date_time_string}.json"
        calibration_handler.eepromToJsonFile(Path(file_name))
        print(f"EEPROM backup saved to {file_name}")
        
        print("Writing calibration to EEPROM...")
        calibration_handler.setCameraIntrinsics(camera_socket, camera_calibration.camera_matrix.tolist(), camera_calibration.width, camera_calibration.height)
        calibration_handler.setDistortionCoefficients(camera_socket, camera_calibration.distortion_coefficients.tolist())
        calibration_handler.setCameraType(camera_socket, new_distortion_model )
        calibration_handler.setFov(camera_socket, 180 if camera_calibration.distortion_model.is_fisheye() else 108) # 108 is the IMX378-W FOV.

        
        device.flashCalibration(calibration_handler)

        device.close()
        
        print("Done writing to EEPROM.")

def main():
    parser = argparse.ArgumentParser(
        prog="Retrieve camera calibration from the Luxonis module and save them to disk."
    )

    parser.add_argument(
        "-l", "--left", action="store_true", help="Use the left RGB camera."
    )
    parser.add_argument(
        "-r", "--right", action="store_true", help="Use the right RGB camera."
    )
    parser.add_argument(
        "-c", "--center", action="store_true", help="Use the center RGB."
    )
    parser.add_argument(
        "--clear", action="store_true", help="Clear the EEPROM before writing."
    )
    
    args = parser.parse_args()
    camera_type = None
    if args.left:
        camera_type = RGBCameras.left()
    elif args.right:
        camera_type = RGBCameras.right()
    elif args.center:
        camera_type = RGBCameras.center()
    else:
        raise Exception(
            "You must specify one of --left, --right, --center to specify the rgb camera to use."
        )
    
    read_calibration_and_write_to_eeprom(camera_type, clear_eeprom=args.clear)

    

if __name__ == "__main__":
    main()