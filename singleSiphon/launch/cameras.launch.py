from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Camera 0 (C270 on device index 6)
    # cameras used
# - plug in order does not matter
# | Camera         | USB Port            | /dev/video | Resolution  | FPS |
# |----------------|---------------------|------------|-------------|-----|
# | Logitech C270  | Targus back         | /dev/video4 | 1280×720   | 15  |
# | Logitech C990  | Targus front        | /dev/video6 | 800×600    | 15  |
# | Logitech C615  | PC right USB port   | /dev/video8 | 1920×1080  | 15  |
# use this to find devices v4l2-ctl --list-devices
    cam0_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='v4l2_camera_c270',
        namespace='camera_front',
        parameters=[{
            'video_device': '/dev/video4',
            'image_size': [960, 720],
            'framerate': 20.0,
            'pixel_format': 'mjpeg2rgb',
            'image_transport': 'compressed',
            'io_method': 'mmap',
            'color_format': 'jpeg'  

        }]
    )

    # Camera 1 (C990 on device index 0)
    cam1_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='v4l2_camera_c990',
        namespace='camera_left',
        parameters=[{
            'video_device': '/dev/video8',
            'image_size': [960, 720],
            'framerate': 20.0,
            'pixel_format': 'mjpeg2rgb',
            'image_transport': 'compressed',
            'io_method': 'mmap',
            'color_format': 'jpeg'  

        }]
    )

    # Camera 2 (C615 on device index 8)
    cam2_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='v4l2_camera_c615',
        namespace='camera_right',
        parameters=[{
            'video_device': '/dev/video6',
            'image_size': [960, 720],
            'framerate': 20.0,
            'pixel_format': 'mjpeg2rgb',
            'image_transport': 'compressed',
            'io_method': 'mmap',
            'color_format': 'jpeg'  
        }]
    )

    return LaunchDescription([
        cam0_node,
        cam1_node,
        cam2_node
    ])
