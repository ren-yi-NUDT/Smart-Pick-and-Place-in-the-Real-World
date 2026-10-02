"""
Robotiq 2F-85 / 2F-140 Modbus RTU driver.

Reference: Robotiq 2F-85 & 2F-140 Instruction Manual (2018-11-30)
  https://assets.robotiq.com/website-assets/support_archives/document_en/2F-85_2F-140_Instruction_Manual_PDF_20181130.pdf

Register layout (manual section 4.2 — byte numeration starts at 0):
  WRITE (Robot Output, FC=0x10):
    byte 0  -> ACTION REQUEST  (rACT bit0, rGTO bit3, rATR bit4, rARD bit5)
    byte 1  -> GRIPPER OPTIONS (reserved)
    byte 2  -> GRIPPER OPTIONS 2 (reserved)
    byte 3  -> POSITION REQUEST rPR (0..255, 0=open, 255=closed)
    byte 4  -> SPEED           rSP (0..255)
    byte 5  -> FORCE           rFR (0..255)

  READ (Robot Input, FC=0x03):
    byte 0  -> GRIPPER STATUS  (gACT bit0, gGTO bit3, gSTA bits4-5, gOBJ bits6-7)
    byte 1  -> RESERVED
    byte 2  -> FAULT STATUS    (gFLT bits0-3, kFLT bits4-7)
    byte 3  -> POSITION REQUEST ECHO gPR
    byte 4  -> POSITION        gPO  (actual finger position 0..255)
    byte 5  -> CURRENT         gCU  (motor current, units of 10 mA)

Wire byte ordering (manual section 4.7): data is sent little-endian on the wire,
but pymodbus interprets each register as big-endian. The first wire byte of a
register therefore ends up in the HIGH byte of the value pymodbus returns. So:
  regs[0] = (byte0 << 8) | byte1
  regs[1] = (byte2 << 8) | byte3
  regs[2] = (byte4 << 8) | byte5
"""

import time
from typing import Optional

from pymodbus.client.sync import ModbusSerialClient
from pymodbus.exceptions import ModbusIOException


# --------------------------------------------------------------------------- #
# Register addresses
# --------------------------------------------------------------------------- #
REG_ACTION = 0x03E8        # 1000 (write)
REG_OPTIONS = 0x03E9       # 1001 (write, options+position)
REG_SPD_FORCE = 0x03EA     # 1002 (write, speed+force)

REG_STATUS = 0x07D0        # 2000 (read)

# Action byte (byte 0 of write) bit flags
ACT_RACT = 0x01            # bit 0: Activate
ACT_RGTO = 0x08            # bit 3: Go To Position
ACT_RATR = 0x10            # bit 4: Automatic Release
ACT_RARD = 0x20            # bit 5: Auto-Release Direction (0=close, 1=open)

# Position semantics (manual section 4.3 POSITION REQUEST)
POS_FULLY_OPEN = 0x00      # 0x00 = fully opened
POS_FULLY_CLOSED = 0xFF    # 0xFF = fully closed

# Defaults
DEFAULT_SPEED = 0xFF       # 0..255
DEFAULT_FORCE = 0xFF       # 0..255

# Fault code meanings (manual section 4.4 gFLT)
FLT_TEXT = {
    0x00: "no fault",
    0x05: "action delayed, activation needed",
    0x07: "activation bit not set",
    0x08: "max temperature exceeded (minor)",
    0x0A: "under minimum voltage (major)",
    0x0B: "auto-release in progress (major)",
    0x0C: "internal fault (major)",
    0x0D: "activation fault (major)",
    0x0E: "overcurrent (major)",
    0x0F: "auto-release completed (major)",
}


class Robotiq85:
    """Robotiq 2F-85 / 2F-140 gripper driven via Modbus RTU over RS485."""

    def __init__(
        self,
        port: str = "/dev/ttyUSB0",
        baudrate: int = 115200,
        slave: int = 9,
        timeout: float = 1.0,
    ):
        self.port = port
        self.baudrate = baudrate
        self.slave = slave
        self.timeout = timeout
        self.client: Optional[ModbusSerialClient] = None

    # ------------------------------------------------------------------ #
    # Lifecycle
    # ------------------------------------------------------------------ #
    def connect(self) -> None:
        self.client = ModbusSerialClient(
            method="rtu",
            port=self.port,
            baudrate=self.baudrate,
            bytesize=8,
            parity="N",
            stopbits=1,
            timeout=self.timeout,
        )
        if not self.client.connect():
            raise ConnectionError(
                f"Cannot open {self.port}. Check: (1) cable plugged in, "
                "(2) user is in the 'dialout' group (`sudo usermod -aG dialout $USER`), "
                "(3) no other process is holding the port."
            )

    def disconnect(self) -> None:
        if self.client is not None:
            try:
                self.client.close()
            except Exception:
                pass
            self.client = None

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, *exc):
        self.disconnect()

    # ------------------------------------------------------------------ #
    # Low-level register access
    # ------------------------------------------------------------------ #
    def _read_holding(self, address: int, count: int):
        if self.client is None:
            raise RuntimeError("Not connected. Call connect() first.")
        resp = self.client.read_holding_registers(address, count, unit=self.slave)
        if isinstance(resp, ModbusIOException) or (hasattr(resp, "isError") and resp.isError()):
            raise IOError(f"Modbus read failed at 0x{address:04X}: {resp}")
        return resp.registers

    def _write_holding(self, address: int, values):
        """Use FC=0x10 (Write Multiple Registers); Robotiq reliably replies to this."""
        if self.client is None:
            raise RuntimeError("Not connected. Call connect() first.")
        regs = list(values)
        if len(regs) == 1:
            regs = regs + [0x0000]  # pad so we always use FC=0x10
        resp = self.client.write_registers(address, regs, unit=self.slave)
        if isinstance(resp, ModbusIOException) or (hasattr(resp, "isError") and resp.isError()):
            raise IOError(f"Modbus write failed at 0x{address:04X}: {resp}")

    # ------------------------------------------------------------------ #
    # Activation
    # ------------------------------------------------------------------ #
    def activate(self, reset_first: bool = True, timeout: float = 10.0) -> bool:
        """Run the activation sequence. Call once after power-up.

        reset_first=True sends rACT=0 then rACT=1; recommended by the manual
        so the activation runs even if the gripper was previously activated.
        """
        if reset_first:
            self._write_holding(REG_ACTION, [0x0000, 0x0000, 0x0000])
            time.sleep(0.5)
        # rACT=1 only
        self._write_holding(REG_ACTION, [(ACT_RACT << 8) | 0x00, 0x0000, 0x0000])

        t0 = time.time()
        while time.time() - t0 < timeout:
            s = self.read_status()
            # gSTA bits 4-5: 0=reset, 1=activating, 2=unused, 3=complete
            if s["gSTA"] == 0b11:
                return True
            time.sleep(0.1)
        return False

    def is_activated(self) -> bool:
        return self.read_status()["gSTA"] == 0b11

    # ------------------------------------------------------------------ #
    # Movement
    # ------------------------------------------------------------------ #
    def move_to(
        self,
        pos: int,
        speed: int = DEFAULT_SPEED,
        force: int = DEFAULT_FORCE,
    ) -> None:
        """Command the gripper to a target position (0=open, 255=closed)."""
        if not (0x00 <= pos <= 0xFF):
            raise ValueError(f"pos must be 0..255, got {pos}")
        if not (0x00 <= speed <= 0xFF):
            raise ValueError(f"speed must be 0..255, got {speed}")
        if not (0x00 <= force <= 0xFF):
            raise ValueError(f"force must be 0..255, got {force}")

        if not self.is_activated():
            raise RuntimeError(
                "Gripper is not activated. Call activate() once after power-up."
            )

        # 3 registers = 6 bytes packed as (byte0<<8)|byte1, (byte2<<8)|byte3, (byte4<<8)|byte5
        action_byte = ACT_RACT | ACT_RGTO  # 0x09
        self._write_holding(
            REG_ACTION,
            [
                (action_byte << 8) | 0x00,   # byte 0 = action, byte 1 = 0
                (0x00 << 8) | pos,           # byte 2 = 0, byte 3 = pos
                (speed << 8) | force,        # byte 4 = speed, byte 5 = force
            ],
        )

    def open(self, speed: int = DEFAULT_SPEED, force: int = DEFAULT_FORCE) -> None:
        self.move_to(POS_FULLY_OPEN, speed=speed, force=force)

    def close(self, speed: int = DEFAULT_SPEED, force: int = DEFAULT_FORCE) -> None:
        self.move_to(POS_FULLY_CLOSED, speed=speed, force=force)

    def auto_release(self, direction_open: bool = True, duration: float = 2.0) -> None:
        """Trigger rATR (Automatic Release) — slow PWM-limited motion.

        Use only in emergencies per manual (e.g. after E-stop). After auto-release
        the gripper reports a fault (gFLT=0x0F) and must be re-activated.
        """
        action_byte = ACT_RACT | ACT_RATR
        if direction_open:
            action_byte |= ACT_RARD
        self._write_holding(REG_ACTION, [(action_byte << 8) | 0x00, 0x0000, 0x0000])
        time.sleep(duration)
        # Clear rATR
        self._write_holding(REG_ACTION, [(ACT_RACT << 8) | 0x00, 0x0000, 0x0000])

    def wait_until_idle(
        self,
        timeout: float = 5.0,
        poll_interval: float = 0.05,
    ) -> dict:
        """Block until motion completes or gFLT becomes non-zero.

        Motion is complete when gOBJ != 0 (1=stopped while opening before target,
        2=stopped while closing on object, 3=at requested position).
        """
        t0 = time.time()
        last = self.read_status()
        while time.time() - t0 < timeout:
            last = self.read_status()
            if last["gOBJ"] != 0 or last["gFLT"] != 0:
                return last
            time.sleep(poll_interval)
        return last

    # ------------------------------------------------------------------ #
    # Status
    # ------------------------------------------------------------------ #
    def read_status(self) -> dict:
        """Read 3 registers (6 bytes) of gripper input and decode per manual 4.4."""
        regs = self._read_holding(REG_STATUS, 3)
        # pymodbus returns big-endian register values; first wire byte = high byte.
        status_byte = (regs[0] >> 8) & 0xFF   # byte 0: GRIPPER STATUS
        fault_byte = (regs[1] >> 8) & 0xFF    # byte 2: FAULT STATUS
        pos_req_echo = regs[1] & 0xFF         # byte 3: POSITION REQUEST ECHO (gPR)
        position = (regs[2] >> 8) & 0xFF      # byte 4: POSITION (gPO) -- actual position
        current = regs[2] & 0xFF              # byte 5: CURRENT (gCU) -- 10 mA units

        return {
            # Gripper status bits (manual 4.4)
            "gACT": status_byte & 0x01,           # bit 0
            "gGTO": (status_byte >> 3) & 0x01,    # bit 3
            "gSTA": (status_byte >> 4) & 0x03,    # bits 4-5
            "gOBJ": (status_byte >> 6) & 0x03,    # bits 6-7
            # Fault status (byte 2)
            "gFLT": fault_byte & 0x0F,            # bits 0-3 (gripper faults)
            "kFLT": (fault_byte >> 4) & 0x0F,     # bits 4-7 (controller faults, usually 0)
            # Echo and measurements
            "pos_request_echo": pos_req_echo,     # what we commanded
            "position": position,                 # actual finger position (0=open, 255=closed)
            "current_ma": current * 10,           # motor current
            "raw_status": status_byte,
            "raw_fault": fault_byte,
        }


def describe_status(s: dict) -> str:
    """Human-readable status string for CLI output."""
    obj_text = {
        0: "in motion",
        1: "contact while opening",
        2: "contact while closing",
        3: "at requested position",
    }.get(s["gOBJ"], "?")
    sta_text = {
        0: "reset",
        1: "activating",
        2: "(unused)",
        3: "activated",
    }.get(s["gSTA"], "?")
    flt_text = FLT_TEXT.get(s["gFLT"], f"unknown 0x{s['gFLT']:X}")
    return (
        f"gACT={s['gACT']} gSTA={sta_text} gGTO={s['gGTO']} gOBJ={obj_text} "
        f"flt=0x{s['gFLT']:02X}({flt_text}) pos={s['position']}/255 cur={s['current_ma']}mA"
    )
