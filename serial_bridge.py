"""Serial link to the pan/tilt firmware. Protocol: one ASCII line "pan,tilt\\n" per update."""

import time
from typing import Optional, Tuple

import serial
from serial.tools import list_ports

BAUD_RATE = 115200
_BOARD_HINTS = ("arduino", "ch340", "cp210", "ftdi", "usb serial", "acm")


def find_port() -> Optional[str]:
    """Return the first serial port that looks like an Arduino-style board, if any."""
    for p in list_ports.comports():
        text = f"{p.description} {p.manufacturer or ''}".lower()
        if any(hint in text for hint in _BOARD_HINTS):
            return p.device
    return None


class SerialServoBridge:
    """Sends servo angles, skipping repeats. Degrades to a no-op if no board is attached."""

    def __init__(self, port: Optional[str] = None, baud_rate: int = BAUD_RATE):
        self.port = port or find_port()
        self.conn: Optional[serial.Serial] = None
        self.last_sent: Optional[Tuple[int, int]] = None
        if self.port is None:
            print("[serial] No board found. Running without servos (pass --port to override).")
            return
        try:
            self.conn = serial.Serial(self.port, baud_rate, timeout=0.1, write_timeout=0.1)
            time.sleep(2.0)  # opening the port resets the Arduino; wait for its bootloader
            print(f"[serial] Connected on {self.port} @ {baud_rate} baud.")
        except serial.SerialException as e:
            print(f"[serial] Could not open {self.port}: {e}. Running without servos.")

    @property
    def connected(self) -> bool:
        return self.conn is not None

    def send_angles(self, pan: float, tilt: float, force: bool = False) -> Tuple[int, int]:
        """Clamp to 0-180, send if changed, and return the integer angles."""
        angles = (round(max(0.0, min(180.0, pan))), round(max(0.0, min(180.0, tilt))))
        if self.conn and (force or angles != self.last_sent):
            try:
                self.conn.write(f"{angles[0]},{angles[1]}\n".encode("ascii"))
                self.last_sent = angles
            except serial.SerialException as e:
                print(f"[serial] Lost connection: {e}")
                self.close()
        return angles

    def close(self) -> None:
        if self.conn:
            self.conn.close()
            self.conn = None
