import depthai as dai

def luxonis_list_devices():
    devices = dai.Device.getAllAvailableDevices()

    for info in devices:
        with dai.Device(maxUsbSpeed=dai.UsbSpeed.SUPER_PLUS, nameOrDeviceId=info.deviceId) as device:
            print(f"""
    Device Name: {device.getProductName()}.
    USB port: {info.name}.
    Device ID: {info.deviceId}.
    Connected cameras: {device.getCameraSensorNames()}.
    Platform: {info.platform}.
    info: {info}. {device.isPipelineRunning()=}

    {device.getConnectedCameraFeatures()=}
    """) 

            
            temp_data = device.getChipTemperature()
            print(f"Average Temperature: {temp_data.average:.2f}°C")

            crash_dump = device.getCrashDump()
            crash_reports = crash_dump.crashReports

            if len(crash_reports) > 0:
                print(f"Crash dump found! {[ss.context for crash_report in crash_reports for s in crash_report.threadCallstack for ss in s.callStack]}")
            else:
                print("No crash dump found")

if __name__ == "__main__":
    luxonis_list_devices()