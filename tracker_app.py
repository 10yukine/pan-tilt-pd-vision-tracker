"""2-axis colour tracker: HSV segmentation -> PD controller -> Arduino pan/tilt servos."""

import argparse
import tkinter as tk
from tkinter import ttk
from typing import Optional

import cv2
import numpy as np

from pd_controller import HOME_PAN, HOME_TILT, PDConfig, PDTrackerController
from serial_bridge import SerialServoBridge

WINDOW = "2-Axis PD Vision Tracker"
MIN_TARGET_AREA = 500  # px^2; smaller blobs are treated as noise
LOOP_MS = 20


class TrackerGUI:
    # name: (h_min, s_min, v_min, h_max, s_max, v_max, h2_min, h2_max, use_second_hue_band)
    PRESETS = {
        "Red": (0, 125, 100, 10, 255, 255, 160, 179, True),  # red wraps around hue 0/179
        "Blue": (100, 100, 100, 130, 255, 255, 0, 0, False),
        "Yellow": (20, 100, 100, 40, 255, 255, 0, 0, False),
        "Green": (40, 60, 45, 95, 255, 255, 0, 0, False),
    }

    def __init__(self, camera_index: int = 0, port: Optional[str] = None, config: Optional[PDConfig] = None):
        self.cap = cv2.VideoCapture(camera_index)
        if not self.cap.isOpened():
            raise SystemExit(f"Could not open camera {camera_index}. Try a different --camera index.")
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        self.bridge = SerialServoBridge(port=port)
        self.controller = PDTrackerController(config)
        self.is_following = False
        self.kernel = np.ones((5, 5), np.uint8)

        self.root = tk.Tk()
        self.root.title(WINDOW)
        self._init_vars()
        self._build_ui()
        self.apply_preset("Yellow")
        self.reset_servos()

    def _init_vars(self) -> None:
        self.h_min, self.h_max = tk.IntVar(), tk.IntVar()
        self.s_min, self.s_max = tk.IntVar(), tk.IntVar()
        self.v_min, self.v_max = tk.IntVar(), tk.IntVar()
        self.h2_min, self.h2_max = tk.IntVar(), tk.IntVar()
        self.use_mask2 = tk.BooleanVar()
        self.pan_kp = tk.IntVar(value=60)  # slider value / 1000 = degrees per pixel
        self.tilt_kp = tk.IntVar(value=55)

    def _build_ui(self) -> None:
        frame = ttk.Frame(self.root, padding=10)
        frame.pack(fill=tk.BOTH, expand=True)

        sliders = [
            ("Hue Min", self.h_min, 179),
            ("Hue Max", self.h_max, 179),
            ("Sat Min", self.s_min, 255),
            ("Sat Max", self.s_max, 255),
            ("Val Min", self.v_min, 255),
            ("Val Max", self.v_max, 255),
            ("Pan Gain (Kp)", self.pan_kp, 100),
            ("Tilt Gain (Kp)", self.tilt_kp, 100),
            ("Hue 2 Min", self.h2_min, 179),
            ("Hue 2 Max", self.h2_max, 179),
        ]
        for row, (label, var, max_val) in enumerate(sliders):
            ttk.Label(frame, text=label).grid(row=row, column=0, padx=5, sticky="w")
            tk.Scale(frame, variable=var, to=max_val, orient="horizontal", length=240).grid(row=row, column=1)

        ttk.Checkbutton(frame, text="Use second hue band (needed for red)", variable=self.use_mask2).grid(
            row=len(sliders), column=0, columnspan=2, sticky="w", pady=4
        )

        preset_bar = ttk.Frame(self.root)
        preset_bar.pack(pady=5)
        for name in self.PRESETS:
            ttk.Button(preset_bar, text=name, command=lambda n=name: self.apply_preset(n)).pack(
                side=tk.LEFT, padx=3
            )

        self.btn_follow = ttk.Button(self.root, text="Start Following", command=self.toggle_follow)
        self.btn_follow.pack(pady=4)
        ttk.Button(
            self.root, text=f"Reset Servos ({HOME_PAN:.0f}°, {HOME_TILT:.0f}°)", command=self.reset_servos
        ).pack(pady=4)

    def apply_preset(self, name: str) -> None:
        h1m, s1m, v1m, h1x, s1x, v1x, h2m, h2x, m2 = self.PRESETS[name]
        self.h_min.set(h1m)
        self.s_min.set(s1m)
        self.v_min.set(v1m)
        self.h_max.set(h1x)
        self.s_max.set(s1x)
        self.v_max.set(v1x)
        self.h2_min.set(h2m)
        self.h2_max.set(h2x)
        self.use_mask2.set(m2)

    def toggle_follow(self) -> None:
        self.is_following = not self.is_following
        self.btn_follow.config(text="Stop Following" if self.is_following else "Start Following")

    def reset_servos(self) -> None:
        pan, tilt = self.controller.reset_angles()
        self.bridge.send_angles(pan, tilt, force=True)

    def _compute_mask(self, hsv: np.ndarray) -> np.ndarray:
        s_lo, s_hi = self.s_min.get(), self.s_max.get()
        v_lo, v_hi = self.v_min.get(), self.v_max.get()
        mask = cv2.inRange(hsv, (self.h_min.get(), s_lo, v_lo), (self.h_max.get(), s_hi, v_hi))
        if self.use_mask2.get():
            band2 = cv2.inRange(hsv, (self.h2_min.get(), s_lo, v_lo), (self.h2_max.get(), s_hi, v_hi))
            mask = cv2.bitwise_or(mask, band2)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)  # drop speckle
        return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel)  # fill holes

    def update_loop(self) -> None:
        ret, frame = self.cap.read()
        if not ret:
            self.root.after(LOOP_MS, self.update_loop)
            return

        frame = cv2.flip(frame, 1)
        h, w = frame.shape[:2]

        mask = self._compute_mask(cv2.cvtColor(frame, cv2.COLOR_BGR2HSV))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        largest = max(contours, key=cv2.contourArea, default=None)

        if largest is not None and cv2.contourArea(largest) > MIN_TARGET_AREA:
            x, y, bw, bh = cv2.boundingRect(largest)
            sm_x, sm_y = self.controller.step(
                x + bw / 2,
                y + bh / 2,
                w / 2,
                h / 2,
                self.pan_kp.get() / 1000.0,
                self.tilt_kp.get() / 1000.0,
                self.is_following,
            )
            cv2.rectangle(frame, (x, y), (x + bw, y + bh), (0, 255, 0), 2)
            cv2.drawMarker(frame, (sm_x, sm_y), (0, 0, 255), cv2.MARKER_CROSS, 22, 2)
        else:
            self.controller.reset_history()

        pan, tilt = self.bridge.send_angles(self.controller.pan_angle, self.controller.tilt_angle)

        status = "FOLLOWING" if self.is_following else "PAUSED"
        color = (0, 255, 0) if self.is_following else (0, 255, 255)
        cv2.putText(frame, f"MOTORS: {status}", (20, 40), cv2.FONT_HERSHEY_PLAIN, 2, color, 2)
        cv2.putText(
            frame, f"Pan: {pan} deg | Tilt: {tilt} deg", (20, 75), cv2.FONT_HERSHEY_PLAIN, 1.4, (255, 255, 255), 1
        )
        if not self.bridge.connected:
            cv2.putText(frame, "NO SERIAL (preview only)", (20, 105), cv2.FONT_HERSHEY_PLAIN, 1.4, (0, 0, 255), 1)

        view = np.hstack((frame, cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)))
        cv2.imshow(WINDOW, cv2.resize(view, (0, 0), fx=0.8, fy=0.8))

        if cv2.waitKey(1) & 0xFF == ord("q"):
            self.root.quit()
            return
        self.root.after(LOOP_MS, self.update_loop)

    def run(self) -> None:
        try:
            self.update_loop()
            self.root.mainloop()
        finally:
            self.cap.release()
            self.bridge.close()
            cv2.destroyAllWindows()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="2-Axis PD-Stabilized Color Tracker")
    parser.add_argument("--camera", type=int, default=0, help="Webcam index")
    parser.add_argument("--port", help="Serial port, e.g. COM3 or /dev/ttyACM0 (auto-detected if omitted)")
    parser.add_argument("--invert-pan", action="store_true", help="Flip pan direction")
    parser.add_argument("--invert-tilt", action="store_true", help="Flip tilt direction")
    args = parser.parse_args()

    config = PDConfig(invert_pan=args.invert_pan, invert_tilt=args.invert_tilt)
    TrackerGUI(camera_index=args.camera, port=args.port, config=config).run()
