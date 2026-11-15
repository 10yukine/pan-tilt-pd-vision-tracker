"""Run with: python test_pd_controller.py"""

from pd_controller import HOME_PAN, HOME_TILT, PDConfig, PDTrackerController

CX, CY = 320, 240
KP = 0.06


def test_first_sample_has_no_derivative_kick():
    c = PDTrackerController()
    c.step(CX + 100, CY, CX, CY, KP, KP, active=True)
    # Pure proportional response: angle_smooth * error * kp, no D contribution.
    assert abs(c.pan_angle - (HOME_PAN + 0.3 * 100 * KP)) < 1e-9
    assert c.tilt_angle == HOME_TILT


def test_holds_still_inside_deadzone_and_when_paused():
    c = PDTrackerController()
    c.step(CX + 10, CY - 10, CX, CY, KP, KP, active=True)
    assert (c.pan_angle, c.tilt_angle) == (HOME_PAN, HOME_TILT)
    c.step(CX + 200, CY + 200, CX, CY, KP, KP, active=False)
    assert (c.pan_angle, c.tilt_angle) == (HOME_PAN, HOME_TILT)


def test_invert_flips_direction():
    c = PDTrackerController(PDConfig(invert_pan=True, invert_tilt=True))
    c.step(CX + 100, CY + 100, CX, CY, KP, KP, active=True)
    assert c.pan_angle < HOME_PAN and c.tilt_angle < HOME_TILT


def test_angles_stay_in_servo_range():
    c = PDTrackerController()
    for _ in range(500):
        c.step(CX + 300, CY - 300, CX, CY, KP, KP, active=True)
    assert c.pan_angle == 180.0 and c.tilt_angle == 0.0


def test_reset_restores_home():
    c = PDTrackerController()
    c.step(CX + 100, CY + 100, CX, CY, KP, KP, active=True)
    assert c.reset_angles() == (HOME_PAN, HOME_TILT)
    assert c.smooth_cx is None


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn()
            print(f"ok  {name}")
