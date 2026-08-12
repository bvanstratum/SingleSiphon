#!/usr/bin/env python3
"""
Interactive live-tuning tool for the Logitech C920 (camera_left) focus.

Shows a live preview window with an OpenCV trackbar for focus_absolute and
a trackbar to toggle continuous autofocus, so you can find a sharp manual
focus value while watching the image update in real time.

Controls are applied via v4l2-ctl subprocess calls (not OpenCV's
CAP_PROP_FOCUS/CAP_PROP_AUTOFOCUS) - this camera's real autofocus control is
named 'focus_automatic_continuous' by v4l2-ctl, not the generic 'focus_auto'
most drivers assume, and this is the exact recipe confirmed working live in
frequencyControlDemo_withVideo.py's cam1_disable_autofocus_action /
cam1_set_focus_action.

Must be run standalone, i.e. NOT while the ROS launch file already has this
camera open (usb_cam_node_exe) - V4L2 streaming I/O is exclusive, so this
script's own capture would fail to start with the device busy.

Usage:
    python3 c920_focus_controls.py

Keys in the preview window:
    q or ESC  - quit
"""

import subprocess
import sys

import cv2
import numpy as np

DEVICE = '/dev/cam_c920'
WINDOW = 'C920 focus (q to quit)'
STATUS_WINDOW = 'Values'
FOCUS_MAX = 250
FOCUS_STEP = 5


def set_ctrl(name: str, value: int) -> None:
    subprocess.run(
        ['v4l2-ctl', f'--device={DEVICE}', f'--set-ctrl={name}={value}'],
        check=False,
    )


def main():
    cap = cv2.VideoCapture(DEVICE, cv2.CAP_V4L2)
    if not cap.isOpened():
        print(f'Could not open {DEVICE}. Is the ROS launch file already '
              'running with this camera open? Stop it first.')
        sys.exit(1)

    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 960)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    cap.set(cv2.CAP_PROP_FPS, 30)

    # Start with autofocus off, matching the production launch config.
    set_ctrl('focus_automatic_continuous', 0)

    cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
    cv2.namedWindow(STATUS_WINDOW, cv2.WINDOW_NORMAL)

    state = {'autofocus': 0, 'focus': 0}

    def on_autofocus(v):
        state['autofocus'] = v
        set_ctrl('focus_automatic_continuous', v)

    def on_focus(v):
        # Snap to the control's real step size - values in between are
        # accepted but silently rounded by the driver anyway.
        snapped = round(v / FOCUS_STEP) * FOCUS_STEP
        state['focus'] = snapped
        set_ctrl('focus_absolute', snapped)

    cv2.createTrackbar('AutoFocus(0=off)', WINDOW, 0, 1, on_autofocus)
    cv2.createTrackbar('Focus', WINDOW, 0, FOCUS_MAX, on_focus)

    def render_status():
        canvas = np.zeros((100, 300, 3), dtype=np.uint8)
        cv2.putText(canvas, f"AutoFocus: {state['autofocus']}", (10, 35),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(canvas, f"Focus: {state['focus']}", (10, 75),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.imshow(STATUS_WINDOW, canvas)

    print('Drag "Focus" while watching the image; toggle "AutoFocus" to '
          'compare. Press q or ESC in the window to quit. Once you have a '
          'sharp value, put it in frequencyControlDemo_withVideo.py\'s '
          "cam1_set_focus_action ('focus_absolute=<N>').")

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
