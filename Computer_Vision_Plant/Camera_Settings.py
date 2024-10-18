"""
This block of code was just used to determine what settings are actually available for my RealSense Camera
"""

import pyrealsense2 as rs

context = rs.context()

if len(context.devices) > 0: # Device is connected
    for device in context.devices:
        print(f"Device: {device.get_info(rs.camera_info.name)}")
        print(f"Serial Number: {device.get_info(rs.camera_info.serial_number)}")
        print(f"Firmware Version: {device.get_info(rs.camera_info.firmware_version)}")

        # Get sensors associated with the device
        sensors = device.query_sensors()
        for sensor in sensors:
            print(f"Sensor: {sensor.get_info(rs.camera_info.name)}")

            # List all available stream profiles for the sensor
            for profile in sensor.get_stream_profiles():
                # Only print video stream profiles (not motion or other types)
                if profile.is_video_stream_profile():
                    video_profile = profile.as_video_stream_profile()
                    print(f"  Stream: {profile.stream_name()}")
                    print(f"    Format: {profile.format()}")
                    print(f"    Resolution: {video_profile.width()}x{video_profile.height()}")
                    print(f"    Frame rate: {profile.fps()} FPS")
else:
    print("No Intel RealSense device connected.")
