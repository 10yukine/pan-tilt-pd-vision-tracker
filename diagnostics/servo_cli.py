"""Drive the turret by hand to check wiring and range of motion.

Run from the repository root:  python -m diagnostics.servo_cli [--port COM3]
Talks to firmware/pan_tilt_controller.
"""

import argparse
import sys
import time

from pd_controller import HOME_PAN, HOME_TILT
from serial_bridge import BAUD_RATE, SerialServoBridge


def sweep(bridge: SerialServoBridge) -> None:
    for pan in range(0, 181, 10):
        bridge.send_angles(pan, HOME_TILT)
        time.sleep(0.08)
    for tilt in range(0, 181, 10):
        bridge.send_angles(HOME_PAN, tilt)
        time.sleep(0.08)
    bridge.send_angles(HOME_PAN, HOME_TILT)


def main() -> None:
    parser = argparse.ArgumentParser(description="Interactive pan/tilt servo CLI")
    parser.add_argument("--port", help="Serial port (auto-detected if omitted)")
    parser.add_argument("--baud", type=int, default=BAUD_RATE)
    args = parser.parse_args()

    bridge = SerialServoBridge(args.port, args.baud)
    if not bridge.connected:
        sys.exit(1)

    print("Enter 'PAN TILT' in degrees (0-180), 's' to sweep both axes, 'h' for home, 'q' to quit.")
    try:
        while True:
            cmd = input("> ").strip().lower()
            if cmd == "q":
                break
            if cmd == "s":
                sweep(bridge)
            elif cmd == "h":
                bridge.send_angles(HOME_PAN, HOME_TILT)
            else:
                try:
                    pan, tilt = (float(v) for v in cmd.replace(",", " ").split())
                except ValueError:
                    print("Expected two numbers, e.g. '90 30'.")
                    continue
                print("Sent pan=%d tilt=%d" % bridge.send_angles(pan, tilt))
    except (KeyboardInterrupt, EOFError):
        print()
    finally:
        bridge.close()


if __name__ == "__main__":
    main()
