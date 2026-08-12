#!/usr/bin/env python3
"""
Interactive live-tuning tool for the Grasshopper3 GS3-U3-32S4C over Aravis.
Shows a live preview window with OpenCV trackbars for the camera's real
GenICam controls, and a resolution/frame-rate preset selector.

No "focus" slider: this camera's Tamron lens is fully manual (no motor, no
GenICam feature for it) — instead there's a live sharpness readout (variance
of Laplacian) overlaid on the preview so you can watch it while turning the
lens's focus ring by hand, same metric as grasshopper3_focus_loop.py.

Controls (all read live bounds from the camera, not hardcoded):
    Exposure (us)   - ExposureTime, auto-exposure disabled on start
    Gain (x0.1 dB)  - Gain, auto-gain disabled on start
    BlackLvl(x0.1)  - BlackLevel ("brightness"), auto disabled on start
    Resolution      - preset index -> stops/reconfigures/restarts the stream
                      (ROI can't be changed while streaming)
    Framerate (fps) - AcquisitionFrameRate, applied live without restarting

Must run under the system python3 (/usr/bin/python3), not a conda env — the
gi/Aravis bindings are system packages. See grasshopper3_test.py in this same
folder for setup notes (udev rule, apt packages).

Usage:
    /usr/bin/python3 grasshopper3_controls.py

Keys in the preview window:
    q or ESC  - quit
"""

import sys

import gi
gi.require_version('Aravis', '0.8')
from gi.repository import Aravis
import numpy as np
import cv2

WINDOW = 'Grasshopper3 controls (q to quit)'
MAX_PREVIEW_WIDTH = 1000

# (width, height) presets, largest first — index 0 is full sensor resolution,
# filled in for real once we know the sensor size.
RESOLUTION_PRESETS = [
    (2048, 1536),
    (1600, 1200),
    (1280, 960),
    (1024, 768),
    (800, 600),
    (640, 480),
]


class CameraSession:
    """Owns the camera + stream, and knows how to safely reconfigure the ROI
    (which requires stopping acquisition) versus live controls (which don't)."""

    def __init__(self, cam):
        self.cam = cam
        self.stream = None
        self.width = 0
        self.height = 0

    def start(self, width, height):
        if self.stream is not None:
            self.cam.stop_acquisition()
        self.cam.set_region(0, 0, width, height)
        self.width, self.height = self.cam.get_region()[2:4]
        self.stream = self.cam.create_stream(None, None)
        payload = self.cam.get_payload()
        for _ in range(4):
            self.stream.push_buffer(Aravis.Buffer.new_allocate(payload))
        self.cam.start_acquisition()

    def stop(self):
        if self.stream is not None:
            self.cam.stop_acquisition()
            self.stream = None

    def read(self):
        """Non-blocking-ish pop; returns a single-channel Mono8 ndarray or None."""
        buf = self.stream.timeout_pop_buffer(100_000)  # 100ms
        if buf is None:
            return None
        frame = None
        if buf.get_status() == Aravis.BufferStatus.SUCCESS:
            data = buf.get_data()
            frame = np.frombuffer(data, dtype=np.uint8).reshape(self.height, self.width).copy()
        self.stream.push_buffer(buf)
        return frame


def sharpness(gray: np.ndarray) -> float:
    return cv2.Laplacian(gray, cv2.CV_64F).var()


def main():
    Aravis.update_device_list()
    if Aravis.get_n_devices() == 0:
        print('No Aravis-visible camera found. Check the udev rule and try '
              'unplugging/replugging the USB cable.')
        sys.exit(1)

    cam = Aravis.Camera.new(None)
    print(f'Connected: {cam.get_device_id()}')

    # Mono8, matching the production launch config (frequencyControlDemo_
    # withVideo.py) - skips debayering entirely, which turned out to be a
    # CPU bottleneck of its own at full resolution/high framerate with RGB8
    # or BayerGB8 (see that file's PixelFormat comment for the full story).
    cam.set_pixel_format_from_string('Mono8')

    sensor_w, sensor_h = cam.get_sensor_size()
    RESOLUTION_PRESETS[0] = (sensor_w, sensor_h)
    # Drop any preset that's actually bigger than the sensor.
    presets = [(w, h) for w, h in RESOLUTION_PRESETS if w <= sensor_w and h <= sensor_h]

    exp_min, exp_max = cam.get_exposure_time_bounds()
    gain_min, gain_max = cam.get_gain_bounds()
    black_min, black_max = cam.get_black_level_bounds()

    # Manual control only makes sense with the corresponding auto mode off —
    # otherwise the camera overwrites whatever the slider says every frame.
    cam.set_exposure_time_auto(Aravis.Auto.OFF)
    cam.set_gain_auto(Aravis.Auto.OFF)
    cam.set_black_level_auto(Aravis.Auto.OFF)
    cam.set_frame_rate_enable(True)

    session = CameraSession(cam)
    session.start(*presets[0])

    # Queried AFTER session.start(), not before: the frame rate ceiling is
    # resolution-dependent (39.99fps at full 2048x1536 vs 334.85fps at
    # 320x240), and querying it before the ROI is actually set to presets[0]
    # picks up whatever ROI the camera was left in from a previous session
    # instead - a stale, possibly much larger bound than what's really
    # achievable at the resolution the slider claims to be showing.
    fps_min, fps_max = cam.get_frame_rate_bounds()
    cur_fps = min(max(cam.get_frame_rate(), fps_min), fps_max)

    cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)

    def set_exposure(v):
        cam.set_exposure_time(max(exp_min, min(exp_max, float(v))))

    def set_gain(v):
        cam.set_gain(max(gain_min, min(gain_max, v / 10.0)))

    def set_black_level(v):
        cam.set_black_level(max(black_min, min(black_max, v / 10.0)))

    def set_fps(v):
        nonlocal cur_fps
        cur_fps = max(fps_min, min(fps_max, float(v)))
        try:
            cam.set_frame_rate(cur_fps)
        except Exception as e:
            print(f'\nset_frame_rate failed: {e}')

    def set_resolution(idx):
        w, h = presets[idx]
        print(f'\nSwitching resolution to {w}x{h} (stream restart)...')
        session.start(w, h)
        # Re-apply frame rate — bounds/current value can shift with ROI.
        nonlocal fps_min, fps_max, cur_fps
        fps_min, fps_max = cam.get_frame_rate_bounds()
        cur_fps = max(fps_min, min(fps_max, cur_fps))
        try:
            cam.set_frame_rate(cur_fps)
        except Exception as e:
            print(f'set_frame_rate after resolution change failed: {e}')
        # Without these, the slider keeps showing whatever range it had at
        # startup regardless of the new resolution's real bounds - which is
        # exactly the "why does this say 300fps" confusion this fixes.
        cv2.setTrackbarMax('Framerate(fps)', WINDOW, int(fps_max))
        cv2.setTrackbarMin('Framerate(fps)', WINDOW, int(fps_min))
        cv2.setTrackbarPos('Framerate(fps)', WINDOW, int(cur_fps))

    cv2.createTrackbar('Exposure(us)', WINDOW, int(cam.get_exposure_time()), int(exp_max), set_exposure)
    cv2.setTrackbarMin('Exposure(us)', WINDOW, int(exp_min))
    cv2.createTrackbar('Gain(x0.1dB)', WINDOW, int(cam.get_gain() * 10), int(gain_max * 10), set_gain)
    cv2.createTrackbar('BlackLvl(x0.1)', WINDOW, int(cam.get_black_level() * 10), int(black_max * 10), set_black_level)
    cv2.createTrackbar('Resolution', WINDOW, 0, len(presets) - 1, set_resolution)
    cv2.createTrackbar('Framerate(fps)', WINDOW, int(cur_fps), int(fps_max), set_fps)
    cv2.setTrackbarMin('Framerate(fps)', WINDOW, int(fps_min))

    print('Adjust sliders in the preview window. No focus control exists for '
          "this lens — watch the 'sharpness' readout in the image and turn "
          'the lens ring by hand. Press q or ESC in the window to quit.')

    try:
        while True:
            frame = session.read()
            if frame is None:
                continue

            # frame is already single-channel Mono8 - no color conversion
            # needed for the sharpness calc, just a BGR copy so
            # cv2.putText's green text (and imshow) work as expected.
            score = sharpness(frame)

            bgr = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
            h, w = bgr.shape[:2]
            if w > MAX_PREVIEW_WIDTH:
                scale = MAX_PREVIEW_WIDTH / w
                bgr = cv2.resize(bgr, (int(w * scale), int(h * scale)))

            cv2.putText(bgr, f'{w}x{h}  sharpness={score:.1f}', (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.imshow(WINDOW, bgr)

            key = cv2.waitKey(1) & 0xFF
            if key in (ord('q'), 27):  # 27 = ESC
                break
    finally:
        session.stop()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
