"""PD control law for the pan/tilt turret. Pure Python so it can be tested without hardware."""

from dataclasses import dataclass
from typing import Optional, Tuple

HOME_PAN = 90.0
HOME_TILT = 30.0


@dataclass
class PDConfig:
    kd: float = 0.1  # derivative gain, in degrees per pixel-of-error-change per frame
    target_smooth: float = 0.3  # EMA factor for the target centroid (1.0 = no smoothing)
    angle_smooth: float = 0.3  # scales every correction; lower = slower, steadier servos
    deadzone: int = 25  # pixels around frame centre where the servos hold still
    invert_pan: bool = False  # flip if the turret runs away from the target
    invert_tilt: bool = False


def _clamp(angle: float) -> float:
    return max(0.0, min(180.0, angle))


class PDTrackerController:
    """Smooths the target centroid and turns pixel error into servo angle updates."""

    def __init__(self, config: Optional[PDConfig] = None):
        self.cfg = config or PDConfig()
        self.pan_angle = HOME_PAN
        self.tilt_angle = HOME_TILT
        self.reset_history()

    def reset_history(self) -> None:
        """Forget the smoothed target. Call when the target is lost."""
        self.smooth_cx: Optional[float] = None
        self.smooth_cy: Optional[float] = None
        self.prev_err_x = 0.0
        self.prev_err_y = 0.0

    def reset_angles(self) -> Tuple[float, float]:
        self.pan_angle, self.tilt_angle = HOME_PAN, HOME_TILT
        self.reset_history()
        return self.pan_angle, self.tilt_angle

    def step(
        self,
        raw_cx: float,
        raw_cy: float,
        center_x: float,
        center_y: float,
        kp_pan: float,
        kp_tilt: float,
        active: bool,
    ) -> Tuple[int, int]:
        """Feed one detection. Returns the smoothed centroid; angles are on the instance."""
        first_sample = self.smooth_cx is None
        if first_sample:
            self.smooth_cx, self.smooth_cy = float(raw_cx), float(raw_cy)
        else:
            a = self.cfg.target_smooth
            self.smooth_cx = a * raw_cx + (1.0 - a) * self.smooth_cx
            self.smooth_cy = a * raw_cy + (1.0 - a) * self.smooth_cy

        err_x = self.smooth_cx - center_x
        err_y = self.smooth_cy - center_y
        if first_sample:
            # No previous error yet: without this the D term sees a jump from 0 to the
            # full error and kicks the servos every time a target (re)appears.
            self.prev_err_x, self.prev_err_y = err_x, err_y
        # ponytail: derivative is per-frame, not per-second. Fine at a steady ~50 Hz loop;
        # divide by dt here if the frame rate ever varies enough to matter.
        d_err_x = err_x - self.prev_err_x
        d_err_y = err_y - self.prev_err_y

        if active and (abs(err_x) > self.cfg.deadzone or abs(err_y) > self.cfg.deadzone):
            d_pan = self.cfg.angle_smooth * (err_x * kp_pan + d_err_x * self.cfg.kd)
            d_tilt = self.cfg.angle_smooth * (err_y * kp_tilt + d_err_y * self.cfg.kd)
            self.pan_angle = _clamp(self.pan_angle + (-d_pan if self.cfg.invert_pan else d_pan))
            self.tilt_angle = _clamp(self.tilt_angle + (-d_tilt if self.cfg.invert_tilt else d_tilt))

        self.prev_err_x, self.prev_err_y = err_x, err_y
        return int(self.smooth_cx), int(self.smooth_cy)
