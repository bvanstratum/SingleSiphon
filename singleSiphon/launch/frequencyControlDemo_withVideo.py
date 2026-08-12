from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, DeclareLaunchArgument, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from datetime import datetime
import os

def generate_launch_description():
    # Restricts DDS discovery to localhost. Confirmed live (via loadCell's
    # augment_bag_with_filter.py / augment_bag_offline.py debugging): with
    # ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET (a real default on this machine -
    # it has a /15 WiFi subnet), this machine's own local ROS traffic gets
    # delivered via multiple redundant network paths (loopback + WiFi),
    # causing genuine transport-level duplicate message delivery - it
    # inflated force_filter's own loadcell_data_filtered output by ~3-4.5x
    # in real recordings, invisible to `ros2 topic info` (still reports
    # exactly 1 publisher/1 subscriber). Nothing in this launch file needs
    # to reach other machines on the network - foxglove_bridge's remote
    # access is a separate websocket server, unaffected by this.
    os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
    os.environ['ROS_LOCALHOST_ONLY'] = '1'

    # Overlays a locally-patched build of camera_aravis2 ahead of the
    # system-installed one from /opt/ros/jazzy (equivalent to sourcing
    # ~/camera_aravis2_ws/install/setup.bash before launching, done here so
    # it can't be forgotten). The patch disables WITH_MATCHED_EVENTS - a
    # compile flag auto-enabled on every ROS distro except Humble/Iron that
    # drives a subscriber-count-triggered lazy acquisition start/stop. That
    # logic proved unreliable in practice: a startup race (subscribers
    # connecting before the driver's device pointer is initialized silently
    # drops the start event and never retries), a stop that never restarts
    # on re-subscribe, and buffer-pool corruption after a restart - all
    # traced back to this one mechanism while debugging why grasshopper3
    # kept losing its video feed. Disabling it makes the driver start
    # acquisition unconditionally at init and never stop, matching how it
    # already behaves on Humble/Iron.
    camera_aravis2_ws_install = os.path.expanduser('~/camera_aravis2_ws/install')
    overlay_camera_aravis2 = [
        SetEnvironmentVariable(
            'AMENT_PREFIX_PATH',
            f"{camera_aravis2_ws_install}/camera_aravis2:"
            f"{camera_aravis2_ws_install}/camera_aravis2_msgs:"
            + os.environ.get('AMENT_PREFIX_PATH', '')
        ),
        SetEnvironmentVariable(
            'LD_LIBRARY_PATH',
            f"{camera_aravis2_ws_install}/camera_aravis2/lib:"
            f"{camera_aravis2_ws_install}/camera_aravis2_msgs/lib:"
            + os.environ.get('LD_LIBRARY_PATH', '')
        ),
        SetEnvironmentVariable(
            'PYTHONPATH',
            f"{camera_aravis2_ws_install}/camera_aravis2_msgs/lib/python3.12/site-packages:"
            + os.environ.get('PYTHONPATH', '')
        ),
    ]

    debug_arg = DeclareLaunchArgument(
        'debug', default_value='false',
        description='Open a serial debug monitor in xterm'
    )
    serial_port_arg = DeclareLaunchArgument(
        'serial_port', default_value='/dev/ttyACM1',
        description='Serial port for the debug monitor'
    )
    plotjuggler_arg = DeclareLaunchArgument(
        'plotjuggler', default_value='false',
        description='Launch PlotJuggler for topic visualization'
    )
    rosbag_arg = DeclareLaunchArgument(
        'rosbag', default_value='false',
        description='Record a rosbag of all topics for this session'
    )
    front_camera_arg = DeclareLaunchArgument(
        'front_camera', default_value='grasshopper3',
        description="Which camera to use for camera_front: 'grasshopper3' (FLIR/Point "
                    "Grey Grasshopper3 GS3-U3-32S4C over Aravis) or 'c270' (old Logitech "
                    "webcam fallback)"
    )
    cam0_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='v4l2_camera_c270',
        namespace='camera_front',
        parameters=[{
            'video_device': '/dev/video4',
            'image_size': [960, 720],
            'framerate': 30.0,
            'pixel_format': 'mjpeg2rgb',
            'image_transport': 'compressed',
            'io_method': 'mmap',
            'color_format': 'jpeg' ,
            'brightness': 120,

        }],
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration('front_camera'), "' == 'c270'"
        ])),
    )

    # Grasshopper3 (FLIR/Point Grey GS3-U3-32S4C, USB3 Vision) via Aravis.
    # GUID is specific to this physical camera (Aravis.get_device_id(0) when
    # it's the only USB3 Vision device attached) — needs a udev rule granting
    # non-root access, see /etc/udev/rules.d/40-pointgrey.rules.
    grasshopper3_node = Node(
        package='camera_aravis2',
        executable='camera_driver_uv',
        name='grasshopper3',
        namespace='camera_front',
        output='screen',
        parameters=[{
            'guid': 'Point Grey Research-1E100109601F-0109601F',
            'frame_id': 'camera_front',
            'stream_names': ['vis'],
            'camera_info_urls': [
                'file://' + os.path.expanduser(
                    '~/SIPHION_Master_Folder/singleSiphon/config/grasshopper3_camera_info.yaml'
                )
            ],
            # Lets Gain/ExposureTime be set live via `ros2 param set` after
            # startup - see the AnalogControl comment below and
            # grasshopper3_dynamic_params.yaml for why (a real ordering bug
            # in this camera's own GenICam feature tree, not just a
            # nice-to-have). Plain path, NOT a file:// URI despite the
            # "_url" name - camera_aravis2's own README example uses a bare
            # path, and a file:// prefix here fails with
            # "YAML file cannot be loaded: bad file" (confirmed).
            'dynamic_parameters_yaml_url': os.path.expanduser(
                '~/SIPHION_Master_Folder/singleSiphon/config/grasshopper3_dynamic_params.yaml'
            ),
            'ImageFormatControl': {
                # Mono8, not BayerGB8/RGB8: BayerGB8 avoided the RGB8
                # bandwidth ceiling (39.99fps -> 121.21fps at full res) but
                # needed host-side debayering to turn into a real color
                # image, and demosaicing a full 2048x1536 frame in software
                # up to 120x/sec turned out to be its own bottleneck -
                # camera_info (untouched by that pipeline) ran near 110fps
                # but every compressed image topic was stuck around 27fps.
                # Mono8 skips debayering entirely (no color to reconstruct),
                # just straight to JPEG encoding, at the cost of no color
                # image at all. debayer_node is removed below since there's
                # nothing for it to do now.
                'PixelFormat': ['Mono8'],
                'Width': 2048,
                'Height': 1536,
            },
            'AcquisitionControl': {
                'AcquisitionFrameRateEnable': True,
                # Mono8's own camera-side bandwidth ceiling at full
                # resolution is 61.40fps (confirmed directly against the
                # camera) - lower than BayerGB8's 121.21fps despite the same
                # 1 byte/pixel payload size, for reasons specific to this
                # camera/format combo, not something we control.
                'AcquisitionFrameRate': 61.0,
                # ExposureMode/TriggerMode set explicitly rather than left to
                # whatever the camera happened to persist from earlier
                # testing - a log showed "Exposure Mode: TriggerWidth" (not
                # something this file ever set), which explains the
                # persistent black image regardless of ExposureTime: in that
                # mode, exposure duration comes from an external trigger
                # pulse's width, not the ExposureTime register at all.
                'ExposureMode': 'Timed',
                'TriggerMode': 'Off',
                # Manual exposure instead of ExposureAuto=Continuous (the
                # default), per your own tuning for motion blur. ExposureTime
                # itself is NOT set here, same reasoning as Gain above -
                # ExposureLocked_Int (confirmed via the camera's GenICam XML)
                # checks the live state of ExposureMode/ExposureAuto's
                # control register, so it needs to be applied after those
                # have already settled, not declared alongside them. See
                # dynamic_parameters_yaml_url and the delayed `ros2 param
                # set` calls below.
                'ExposureAuto': 'Off',
            },
            'AnalogControl': {
                # Gain itself is deliberately NOT set here - see comment
                # above dynamic_parameters_yaml_url. GainAuto's own lock
                # doesn't depend on another feature's live state (confirmed
                # via the camera's GenICam XML), so it applies reliably at
                # startup on its own; Gain does not.
                'GainAuto': 'Off',
            },
        }],
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration('front_camera'), "' == 'grasshopper3'"
        ])),
    )

    # Applies Gain/ExposureTime a few seconds after grasshopper3_node starts,
    # via the dynamic_parameters_yaml_url mechanism (see that comment and
    # grasshopper3_dynamic_params.yaml) instead of the static
    # AnalogControl/AcquisitionControl blocks above - both features' locks
    # depend on the live state of a separate Auto-mode feature's control
    # register, so setting them at startup in the same batch as GainAuto/
    # ExposureAuto/ExposureMode is a race that silently fails. 3s gives the
    # node time to attach the camera, apply the startup config, and settle
    # before these land. Same values you'd already tuned by hand.
    # max gain is 47.99
    grasshopper3_gain_exposure_action = TimerAction(
        period=3.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'param', 'set', '/camera_front/grasshopper3', 'Gain', '30.0'],
                output='screen',
            ),
            ExecuteProcess(
                cmd=['ros2', 'param', 'set', '/camera_front/grasshopper3', 'ExposureTime', '1500.0'],
                output='screen',
            ),
        ],
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration('front_camera'), "' == 'grasshopper3'"
        ])),
    )

    # Camera 1 (Logitech HD Pro Webcam C920 - replaced an old QuickCam Pro
    # 9000 that had been mislabeled "C990" in this file; that camera hard-
    # capped at 15fps at 960x720 per its own UVC descriptor, confirmed via
    # direct v4l2-ctl testing, hence the swap). Uses /dev/cam_c920, a stable
    # symlink from a custom udev rule
    # (/etc/udev/rules.d/71-camera-aliases.rules, keyed on USB
    # vendor/product/serial) rather than /dev/videoN — plain numbers get
    # reassigned by the kernel any time USB devices re-enumerate (e.g. moving
    # another camera to a different hub), even for devices that didn't move.
    # Confirmed happening in the past with the old camera + C615 swapping
    # /dev/video6 <-> /dev/video8 after moving the Grasshopper3 to a USB-C hub.
    #
    # NOTE: this is NOT /dev/v4l/by-id/... — usb_cam has a bug where it
    # readlink()s a symlink and blindly prepends "/dev/" to the *relative*
    # target instead of resolving it relative to the symlink's own directory.
    # /dev/v4l/by-id/foo -> ../../video6 becomes the nonsense "/dev/../../video6"
    # and fails with "not a vaild V4L2 device". A symlink living directly in
    # /dev/ (relative target is just "video6", no "../../") sidesteps the bug
    # since "/dev/" + "video6" is correct. Confirmed via strace.
    cam1_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='v4l2_camera_c920',
        namespace='camera_left',
        parameters=[{
            'video_device': '/dev/cam_c920',
            'image_size': [960, 720],
            'framerate': 30.0,
            'pixel_format': 'mjpeg2rgb',
            'image_transport': 'compressed',
            'io_method': 'mmap',
            'color_format': 'jpeg',
            'brightness': 120,
            # Logitech UVC extension control: lets the camera silently drop
            # below the configured framerate to allow longer auto-exposure
            # integration time in dim light. Kept here as cheap insurance
            # (same mechanism confirmed capping the old camera's framerate);
            # harmless if the C920 doesn't use it the same way.
            'exposure_dynamic_framerate': 0,
        }]
    )

    # usb_cam's own 'focus_auto' parameter maps to the standard
    # V4L2_CID_FOCUS_AUTO control by ID and doesn't reliably disable autofocus
    # on this camera in practice (tried it - autofocus still visibly hunts).
    # Bypassing usb_cam's ROS parameter layer entirely and hitting the device
    # directly with v4l2-ctl, using this camera's actual control name as
    # confirmed via `v4l2-ctl --device=/dev/cam_c920 --list-ctrls`. Delayed
    # so it runs after usb_cam has opened the device and finished its own
    # startup control writes.
    cam1_disable_autofocus_action = TimerAction(
        period=3.0,
        actions=[
            ExecuteProcess(
                cmd=['v4l2-ctl', '--device=/dev/cam_c920',
                     '--set-ctrl=focus_automatic_continuous=0'],
                output='screen',
            ),
        ],
    )

    # One-time manual focus position, applied slightly after autofocus is
    # disabled above (focus_absolute reports "inactive" and rejects writes
    # until the auto-off write has actually settled on the hardware - same
    # race already seen elsewhere in this file). 0 is a placeholder - find
    # the real sharp value while this launch is running with
    # `v4l2-ctl --device=/dev/cam_c920 --set-ctrl=focus_absolute=<N>`
    # (0-250, step 5) while watching the image in Foxglove, then put that
    # number here.
    cam1_set_focus_action = TimerAction(
        period=3.5,
        actions=[
            ExecuteProcess(
                cmd=['v4l2-ctl', '--device=/dev/cam_c920',
                     '--set-ctrl=focus_absolute=60'],
                output='screen',
            ),
        ],
    )

    # Camera 2 (C615) — runs dark by default, brightness bumped up. Stable
    # udev-alias path, see cam1_node comment above for why it's /dev/cam_c615
    # and not /dev/v4l/by-id/...
    cam2_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='v4l2_camera_c615',
        namespace='camera_right',
        parameters=[{
            'video_device': '/dev/cam_c615',
            'image_size': [960, 720],
            'framerate': 30.0,
            'pixel_format': 'mjpeg2rgb',
            'image_transport': 'compressed',
            'io_method': 'mmap',
            'color_format': 'jpeg',
            'brightness': 50,
        }]
    )

    # Same recipe as the C920 (cam1_disable_autofocus_action /
    # cam1_set_focus_action above): bypass usb_cam's own 'focus_auto'
    # ROS param entirely and hit the device directly with v4l2-ctl, using
    # this camera's real control name confirmed via
    # `v4l2-ctl --device=/dev/cam_c615 --list-ctrls`
    # (hardwareTestingScripts/c615_focus_controls.py). Focus set is delayed
    # slightly past the autofocus-disable so it doesn't race the "inactive
    # until auto-off settles" control-state dependency. 65 confirmed sharp
    # via that script.
    cam2_disable_autofocus_action = TimerAction(
        period=3.0,
        actions=[
            ExecuteProcess(
                cmd=['v4l2-ctl', '--device=/dev/cam_c615',
                     '--set-ctrl=focus_automatic_continuous=0'],
                output='screen',
            ),
        ],
    )

    cam2_set_focus_action = TimerAction(
        period=3.5,
        actions=[
            ExecuteProcess(
                cmd=['v4l2-ctl', '--device=/dev/cam_c615',
                     '--set-ctrl=focus_absolute=65'],
                output='screen',
            ),
        ],
    )

    foxglove = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge'
    )

    # Everything from this run (main rosbag + telemetry buffer bag) lands
    # together under one dated session folder in esp32_bags.
    session_dir = os.path.expanduser(
        f'~/SIPHION_Master_Folder/esp32_bags/{datetime.now().strftime("%Y%m%d_%H%M%S")}'
    )
    os.makedirs(session_dir, exist_ok=True)

    # rosbag2 always writes a bag as <dir>/metadata.yaml + <dir>/<dir_name>_0.mcap —
    # that wrapper directory can't be avoided without losing ros2 bag play/info
    # compatibility. So we keep it, but hard-link the .mcap up as a flat sibling in
    # session_dir so both recordings show up side by side and can be multi-selected
    # together in Foxglove's file-open dialog. (A symlink looks right in `ls` but
    # Foxglove's file picker won't follow it — a hard link is indistinguishable
    # from a real file.)
    bag_name = 'computer'
    bag_path = os.path.join(session_dir, bag_name)

    rosbag_record_action = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record', '-a',
            # Skips the raw/theora/zstd duplicate transports for every camera
            # (only need the compressed one) - grasshopper3 publishes Mono8
            # directly as image_raw (no debayer step, no separate
            # image_color/image_mono topics), so image_raw/compressed is a
            # real usable feed here same as camera_left/camera_right.
            '--exclude-regex',
            r'image_raw$'
            r'|image_raw/(theora|zstd|compressedDepth)$',
            '-o', bag_path,
        ],
        output='screen',
        condition=IfCondition(LaunchConfiguration('rosbag')),
    )

    # rosbag2 creates the .mcap within ~1s of the recorder starting (well before
    # you stop recording), so poll for it and link it early rather than hooking
    # the recorder's process-exit: the only time that process actually exits is
    # as part of the whole launch's Ctrl-C shutdown, and ros2 launch doesn't
    # reliably run on-exit follow-up actions once that global shutdown has begun
    # (confirmed: computer.mcap never appeared when linking was done that way).
    link_computer_mcap_action = ExecuteProcess(
        cmd=['bash', '-c',
             'for i in $(seq 1 50); do [ -f "$1" ] && break; sleep 0.2; done; ln -f "$1" "$2"',
             'link_computer_mcap',
             os.path.join(bag_path, f'{bag_name}_0.mcap'),
             os.path.join(session_dir, f'{bag_name}.mcap')],
        output='screen',
        condition=IfCondition(LaunchConfiguration('rosbag')),
    )

    return LaunchDescription([
        *overlay_camera_aravis2,
        debug_arg,
        serial_port_arg,
        plotjuggler_arg,
        rosbag_arg,
        front_camera_arg,
        cam0_node,
        grasshopper3_node,
        grasshopper3_gain_exposure_action,
        cam1_node,
        cam1_disable_autofocus_action,
        cam1_set_focus_action,
        cam2_node,
        cam2_disable_autofocus_action,
        cam2_set_focus_action,
        foxglove,

        # micro-ROS agent
        ExecuteProcess(
            cmd=[
                os.path.expanduser('~/uagent_ws/src/Micro-XRCE-DDS-Agent/build/MicroXRCEAgent'),
                'udp4', '--port', '8888', '-v', '0'
            ],
            cwd=os.path.expanduser('~/uagent_ws/src/Micro-XRCE-DDS-Agent/build'),
            output='screen'
        ),

        # Teleop node
        Node(
            package='singleSiphon',
            executable='actuator_teleop',
            output='screen',
            prefix='xterm -geometry +1058+1295 -e',
        ),

        # Kalman-filters loadcell_data (wrench.force.z) and republishes it on
        # loadcell_data_filtered. Values rescaled from the original
        # estimate_force_noise.py/tune_filter_offline.py-derived numbers
        # (3e6 / 1000.0) now that esp32LoadCell_mROS.ino publishes
        # /loadcell_data in millinewtons (slope=-0.08355) instead of raw
        # counts - see ForceFilterNode.py's declare_parameter comment for
        # the scaling math. Re-derive properly against real mN-scale data
        # once available, rather than trusting this linear-transform estimate.
        Node(
            package='singleSiphon',
            executable='force_filter',
            output='screen',
            parameters=[{
                'measurement_noise_variance': 20941.8,
                'process_noise_accel_stddev': 83.55,
            }],
        ),

         Node(
                    package='singleSiphon',
                    executable='force_calibrator',
                    output='screen',
                    parameters=[{
                        'calibration_slope': -0.08355,
                        'calibration_intercept': -31.958331,
                    }],
                ),

        # Current buffer saver — stays running, re-triggerable from a Foxglove
        # Publish panel (topic actuator_1/save_buffer_trigger, std_msgs/Bool, {"data": true})
        ExecuteProcess(
            cmd=[
                '/usr/bin/python3',
                os.path.expanduser('~/SIPHION_Master_Folder/esp32_micro_ros/save_current_buffer_sync.py'),
                'hires',
            ],
            cwd=session_dir,
            output='screen',
        ),

        # Optional rosbag recording of all topics (pass rosbag:=true to enable).
        # Skips the raw/theora/zstd camera transports — image_transport advertises
        # those alongside the compressed one, but they're just bulkier duplicates
        # of the same video and Foxglove can't play theora/zstd back anyway.
        rosbag_record_action,
        link_computer_mcap_action,

        # Optional PlotJuggler (pass plotjuggler:=true to enable)
        Node(
            package='plotjuggler',
            executable='plotjuggler',
            output='screen',
            additional_env={'QT_QPA_PLATFORM': 'xcb'},
            arguments=(
                ['--layout', os.path.expanduser('~/SIPHION_Master_Folder/singleSiphon/config/demo.xml')]
                if os.path.exists(os.path.expanduser('~/SIPHION_Master_Folder/singleSiphon/config/demo.xml'))
                else []
            ),
            condition=IfCondition(LaunchConfiguration('plotjuggler')),
        ),

        # Optional serial debug monitor (pass debug:=true to enable).
        # Logs to ~/SIPHION_Master_Folder/esp32_logs/esp32_debug_<timestamp>.log
        # while showing live output in xterm.
        ExecuteProcess(
            cmd=[
                'xterm', '-title', 'Serial Debug', '-geometry', '+1044+881', '-e',
                'bash', '-c',
                [
                    'mkdir -p $HOME/SIPHION_Master_Folder/esp32_logs && '
                    'LOG=$HOME/SIPHION_Master_Folder/esp32_logs/esp32_debug_$(date +%Y%m%d_%H%M%S).log && '
                    'echo "Logging to: $LOG" && '
                    'fuser -k ', LaunchConfiguration('serial_port'), ' 2>/dev/null; '
                    'sleep 0.3; '
                    'screen -L -Logfile $LOG ', LaunchConfiguration('serial_port'), ' 115200; '
                    'echo "Log saved to: $LOG  -- press Enter to close"; read'
                ]
            ],
            output='screen',
            condition=IfCondition(LaunchConfiguration('debug')),
        ),
        


    ])
