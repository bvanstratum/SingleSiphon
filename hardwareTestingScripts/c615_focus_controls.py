#!/usr/bin/env python3
"""
Interactive live-tuning tool for the Logitech C615 focus - mirrors
c920_focus_controls.py. Currently the C615 is the camera physically
mounted viewing the front, even though its launch-file namespace is
camera_right (cam2_node) - namespaces there are position-based and can
drift from physical placement when cameras get swapped around.

The C615 is commonly documented as fixed-focus with no motor, unlike the
C920. Rather than assume, this script probes the camera's actual control
list at startup via `v4l2-ctl --list-ctrls` and only shows the focus
trackbars if a focus control genuinely exists - otherwise it tells you
plainly and just shows the preview (still useful for framing/exposure
checks).

Uses the stable udev alias /dev/cam_c615 (see
/etc/udev/rules.d/71-camera-aliases.rules) by default, so it keeps working
regardless of which USB port the camera is plugged into. Pass a different
device path as the first argument if needed.

Must be run standalone, i.e. NOT while the ROS launch file already has this
camera open (usb_cam_node_exe) - V4L2 streaming I/O is exclusive, so this
script's own capture would fail to start with the device busy.

Keys in the preview window:
    q or ESC  - quit
"""

import re
import subprocess
import sys

import cv2
import numpy as np

# Matches lines like "     focus_absolute 0x009a090a (int)    : min=0 ..."
# - v4l2-ctl indents every control name with leading spaces, so a plain
# "not line.startswith(' ')" check (an earlier version of this script) ends
# up excluding every real control instead of just the section headers.
CTRL_LINE_RE = re.compile(r'^\s*(\w+)\s+0x[0-9a-fA-F]+\s+\(')

WINDOW = 'C615 focus (q to quit)'
STATUS_WINDOW = 'Values'
FOCUS_MAX = 250
FOCUS_STEP = 5


def set_ctrl(device: str, name: str, value: int) -> None:
    subprocess.run(
        ['v4l2-ctl', f'--device={device}', f'--set-ctrl={name}={value}'],
        check=False,
    )


def list_ctrl_names(device: str) -> set:
    result = subprocess.run(
        ['v4l2-ctl', f'--device={device}', '--list-ctrls'],
        capture_output=True, text=True, check=False,
    )
    return {m.group(1) for line in result.stdout.splitlines()
            if (m := CTRL_LINE_RE.match(line))}


def main():
    device = sys.argv[1] if len(sys.argv) > 1 else '/dev/cam_c615'

    cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
    if not cap.isOpened():
        print(f'Could not open {device}. Make sure the camera is plugged '
              "in and the ROS launch file isn't already holding it open.")
        sys.exit(1)

    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 960)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    cap.set(cv2.CAP_PROP_FPS, 30)

    ctrl_names = list_ctrl_names(device)
    has_focus_absolute = 'focus_absolute' in ctrl_names
    has_autofocus_ctrl = next(
        (n for n in ('focus_automatic_continuous', 'focus_auto') if n in ctrl_names),
        None,
    )

    if not has_focus_absolute:
        print(f"No 'focus_absolute' control found on {device} - this "
              "camera likely has a fixed-focus lens with nothing to tune "
              "in software. Showing the preview only so you can still "
              "check framing/exposure.")

    cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
    cv2.namedWindow(STATUS_WINDOW, cv2.WINDOW_NORMAL)

    state = {'autofocus': 0, 'focus': 0}

    if has_autofocus_ctrl:
        set_ctrl(device, has_autofocus_ctrl, 0)

        def on_autofocus(v):
            state['autofocus'] = v
            set_ctrl(device, has_autofocus_ctrl, v)

        cv2.createTrackbar('AutoFocus(0=off)', WINDOW, 0, 1, on_autofocus)

    if has_focus_absolute:
        def on_focus(v):
            snapped = round(v / FOCUS_STEP) * FOCUS_STEP
            state['focus'] = snapped
            set_ctrl(device, 'focus_absolute', snapped)

        cv2.createTrackbar('Focus', WINDOW, 0, FOCUS_MAX, on_focus)

    def render_status():
        canvas = np.zeros((100, 300, 3), dtype=np.uint8)
        cv2.putText(canvas, f"AutoFocus: {state['autofocus']}", (10, 35),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(canvas, f"Focus: {state['focus']}", (10, 75),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.imshow(STATUS_WINDOW, canvas)

    print('Press q or ESC in the window to quit.')

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                continue
            cv2.imshow(WINDOW, frame)
            render_status()
            key = cv2.waitKey(1) & 0xFF
            if key in (ord('q'), 27):  # 27 = ESC
                break
    finally:
        cap.release()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
