"""Drive and steer controller implementations."""

from __future__ import annotations

import json
import logging
import math
import threading
import time
from pathlib import Path
from typing import Callable, Dict, Iterable, List, Optional

from .can_bus import ensure_buses, send_sdo_read, send_sdo_write

logger = logging.getLogger(__name__)

StatusCallback = Callable[[str], None]

# --- Drive constants ---------------------------------------------------------
WHEEL_RADIUS_M = 0.075
WHEEL_CIRCUMFERENCE_M = 2.0 * math.pi * WHEEL_RADIUS_M
DRIVE_REDUCTION_RATIO = 20.0
FEED_CONSTANT = 10000.0

DRIVE_NODE_CONFIG: Dict[int, str] = {
    1: "can1",  # Front Left
    3: "can0",  # Front Right
    5: "can1",  # Rear Left
    7: "can0",  # Rear Right
}

DRIVE_NODES_TO_INVERT = {1, 5}
OD_CONTROL_WORD = 0x6040
OD_MODES_OF_OPERATION = 0x6060
OD_TARGET_VELOCITY = 0x60FF
OD_TARGET_POSITION = 0x607A
OD_PROFILE_VELOCITY = 0x6081
OD_PROFILE_ACCELERATION = 0x6083
OD_PROFILE_DECELERATION = 0x6084
OD_ACTUAL_POSITION = 0x6064
OD_ACTUAL_VELOCITY = 0x606C
OD_MOTOR_DIRECTION = 0x607E

DRIVE_SPEED_STEP_MPS = 0.05
DRIVE_ACCEL_STEP_PPS2 = 10000
DRIVE_MIN_ACCEL_PPS2 = 1000
DRIVE_READ_TIMEOUT = 0.5

# --- Steer constants ---------------------------------------------------------
TOTAL_REDUCTION_RATIO = 122.5
USER_PULSES_PER_REVOLUTION = 18000.0
PULSE_PER_DEGREE = (USER_PULSES_PER_REVOLUTION * TOTAL_REDUCTION_RATIO) / 360.0

STEER_NODE_CONFIG: Dict[int, str] = {
    2: "can1",  # Front Left
    4: "can0",  # Front Right
    6: "can1",  # Rear Left
    8: "can0",  # Rear Right
}

STEER_DEG_STEP = 1
STEER_VEL_STEP_DPS = 5.0
STEER_ACCEL_STEP_DPS2 = 20.0
STEER_MIN_VEL_DPS = 1.0
STEER_MIN_ACCEL_DPS2 = 5.0
STEER_READ_TIMEOUT = 0.5

STEER_OFFSET_STEP_DEG = 0.01
DEFAULT_STEER_OFFSET_FILENAME = "steer_offsets.json"


# --- Helper functions --------------------------------------------------------

def mps_to_pps(mps: float) -> int:
    wheel_rps = mps / WHEEL_CIRCUMFERENCE_M
    motor_rps = wheel_rps * DRIVE_REDUCTION_RATIO
    return int(round(motor_rps * FEED_CONSTANT))


def pps_to_mps(pps: int) -> float:
    motor_rps = pps / FEED_CONSTANT
    wheel_rps = motor_rps / DRIVE_REDUCTION_RATIO
    return wheel_rps * WHEEL_CIRCUMFERENCE_M


def pps2_to_mps2(pps2: float) -> float:
    return pps_to_mps(pps2)


def mps2_to_pps2(mps2: float) -> int:
    return mps_to_pps(mps2)


def pulses_to_meters(pulses: int) -> float:
    motor_rev = pulses / FEED_CONSTANT
    wheel_rev = motor_rev / DRIVE_REDUCTION_RATIO
    return wheel_rev * WHEEL_CIRCUMFERENCE_M


def deg_to_pulses(deg: float) -> int:
    return int(round(deg * PULSE_PER_DEGREE))


def pulses_to_deg(pulses: int) -> float:
    return pulses / PULSE_PER_DEGREE


def dps_to_pps(dps: float) -> int:
    return int(round(dps * PULSE_PER_DEGREE))


def pps_to_dps(pps: int) -> float:
    return pps / PULSE_PER_DEGREE


def dps2_to_pps2(dps2: float) -> int:
    return int(round(dps2 * PULSE_PER_DEGREE))


def pps2_to_dps2(pps2: int) -> float:
    return pps2 / PULSE_PER_DEGREE


# --- Controller implementations ---------------------------------------------

class DriveController:
    def __init__(self, nodes: List[int], status_callback: Optional[StatusCallback] = None):
        self.nodes = nodes
        self.node_to_bus = {node: DRIVE_NODE_CONFIG[node] for node in nodes}
        self.current_speed_mps = 0.0
        self.current_accel_pps2 = 50000
        self.state_lock = threading.Lock()
        self._status_callback = status_callback

    def _log(self, message: str) -> None:
        if self._status_callback:
            self._status_callback(message)
        else:
            logger.info(message)

    def setup(self) -> None:
        for node in self.nodes:
            bus_name = self.node_to_bus[node]
            if node in DRIVE_NODES_TO_INVERT:
                send_sdo_write(bus_name, node, OD_MOTOR_DIRECTION, 0, 64, 1)
                self._log(f"[Setup] Node {node} direction has been inverted.")

            send_sdo_write(bus_name, node, OD_CONTROL_WORD, 0, 0x06, 2)
            send_sdo_write(bus_name, node, OD_CONTROL_WORD, 0, 0x07, 2)
            send_sdo_write(bus_name, node, OD_MODES_OF_OPERATION, 0, 3, 1)
            send_sdo_write(bus_name, node, OD_CONTROL_WORD, 0, 0x0F, 2)
            self._push_accel_to_node(node)
            self._push_speed_to_node(node)

    def _for_each_node(self) -> Iterable[tuple[int, str]]:
        for node in self.nodes:
            yield node, self.node_to_bus[node]

    def _push_speed_to_node(self, node: int) -> None:
        bus_name = self.node_to_bus[node]
        speed_pps = mps_to_pps(self.current_speed_mps)
        send_sdo_write(bus_name, node, OD_TARGET_VELOCITY, 0, speed_pps, 4)

    def _push_accel_to_node(self, node: int) -> None:
        bus_name = self.node_to_bus[node]
        accel = max(DRIVE_MIN_ACCEL_PPS2, int(round(self.current_accel_pps2)))
        send_sdo_write(bus_name, node, OD_PROFILE_ACCELERATION, 0, accel, 4)
        send_sdo_write(bus_name, node, OD_PROFILE_DECELERATION, 0, accel, 4)

    def adjust_speed(self, delta: float) -> None:
        with self.state_lock:
            self.current_speed_mps += delta
            target = self.current_speed_mps
        for node, _ in self._for_each_node():
            self._push_speed_to_node(node)
        self._log(f"[Drive] Target speed set to {target:.2f} m/s")

    def set_speed(self, value: float) -> None:
        with self.state_lock:
            self.current_speed_mps = value
        for node, _ in self._for_each_node():
            self._push_speed_to_node(node)
        self._log(f"[Drive] Target speed forced to {value:.2f} m/s")

    def adjust_accel(self, delta_pps2: int) -> None:
        with self.state_lock:
            self.current_accel_pps2 = max(
                DRIVE_MIN_ACCEL_PPS2, self.current_accel_pps2 + delta_pps2
            )
            accel_pps2 = self.current_accel_pps2
        for node, _ in self._for_each_node():
            self._push_accel_to_node(node)
        accel_mps2 = pps2_to_mps2(accel_pps2)
        self._log(
            f"[Drive] Profile accel set to {accel_mps2:.2f} m/s² ({accel_pps2} pps²)"
        )

    def fetch_status(self) -> Dict[str, object]:
        with self.state_lock:
            tgt_speed = self.current_speed_mps
            tgt_accel_pps2 = self.current_accel_pps2

        actual_velocity: Dict[int, Optional[float]] = {}
        actual_position: Dict[int, Optional[float]] = {}
        for node, bus_name in self._for_each_node():
            try:
                raw_vel = send_sdo_read(bus_name, node, OD_ACTUAL_VELOCITY, 0, DRIVE_READ_TIMEOUT)
                raw_pos = send_sdo_read(bus_name, node, OD_ACTUAL_POSITION, 0, DRIVE_READ_TIMEOUT)
                actual_velocity[node] = pps_to_mps(raw_vel)
                actual_position[node] = pulses_to_meters(raw_pos)
            except Exception as exc:
                actual_velocity[node] = None
                actual_position[node] = None
                self._log(f"[Drive] Read failed on node {node}: {exc}")

        return {
            "target_speed_mps": tgt_speed,
            "target_accel_mps2": pps2_to_mps2(tgt_accel_pps2),
            "actual_velocity_mps": actual_velocity,
            "actual_position_m": actual_position,
        }

    def emergency_stop(self) -> None:
        self.set_speed(0.0)

    def print_actual_state(self) -> None:
        status = self.fetch_status()
        lines = ["[Drive] Actual velocity (m/s):"]
        for node, value in status["actual_velocity_mps"].items():
            txt = f"  Node {node}: {value:.3f}" if value is not None else f"  Node {node}: N/A"
            lines.append(txt)
        lines.append("[Drive] Actual position (m):")
        for node, value in status["actual_position_m"].items():
            txt = f"  Node {node}: {value:.3f}" if value is not None else f"  Node {node}: N/A"
            lines.append(txt)
        for line in lines:
            self._log(line)


class SteerController:
    def __init__(
        self,
        nodes: List[int],
        status_callback: Optional[StatusCallback] = None,
        offset_path: Optional[Path] = None,
    ):
        self.nodes = nodes
        self.node_to_bus = {node: STEER_NODE_CONFIG[node] for node in nodes}
        self.current_target_deg = 0.0
        self.profile_velocity_dps = 30.0
        self.profile_accel_dps2 = 80.0
        self.state_lock = threading.Lock()
        self._status_callback = status_callback
        self.offset_path = Path(offset_path) if offset_path else Path(DEFAULT_STEER_OFFSET_FILENAME)
        self.angle_offsets_deg: Dict[int, float] = {node: 0.0 for node in nodes}
        self.load_offsets()

    def _log(self, message: str) -> None:
        if self._status_callback:
            self._status_callback(message)
        else:
            logger.info(message)

    def setup(self) -> None:
        for node, bus_name in self._for_each_node():
            send_sdo_write(bus_name, node, OD_MODES_OF_OPERATION, 0, 1, 1)
            send_sdo_write(bus_name, node, OD_CONTROL_WORD, 0, 0x06, 2)
            send_sdo_write(bus_name, node, OD_CONTROL_WORD, 0, 0x07, 2)
            send_sdo_write(bus_name, node, OD_CONTROL_WORD, 0, 0x0F, 2)
        self._push_profile_velocity()
        self._push_profile_accel()
        self._push_position()
        self._trigger_motion()

    def _for_each_node(self) -> Iterable[tuple[int, str]]:
        for node in self.nodes:
            yield node, self.node_to_bus[node]

    def _push_profile_velocity(self) -> None:
        value = max(STEER_MIN_VEL_DPS, self.profile_velocity_dps)
        pulses = dps_to_pps(value)
        for node, bus_name in self._for_each_node():
            send_sdo_write(bus_name, node, OD_PROFILE_VELOCITY, 0, pulses, 4)

    def _push_profile_accel(self) -> None:
        value = max(STEER_MIN_ACCEL_DPS2, self.profile_accel_dps2)
        pulses = dps2_to_pps2(value)
        for node, bus_name in self._for_each_node():
            send_sdo_write(bus_name, node, OD_PROFILE_ACCELERATION, 0, pulses, 4)
            send_sdo_write(bus_name, node, OD_PROFILE_DECELERATION, 0, pulses, 4)

    def _push_position(self) -> None:
        for node, bus_name in self._for_each_node():
            with self.state_lock:
                offset = self.angle_offsets_deg.get(node, 0.0)
                final_target_deg = self.current_target_deg + offset
            pulses = deg_to_pulses(final_target_deg)
            send_sdo_write(bus_name, node, OD_TARGET_POSITION, 0, pulses, 4)

    def _trigger_motion(self) -> None:
        for node, bus_name in self._for_each_node():
            send_sdo_write(bus_name, node, OD_CONTROL_WORD, 0, 0x1F, 2)
        time.sleep(0.05)
        for node, bus_name in self._for_each_node():
            send_sdo_write(bus_name, node, OD_CONTROL_WORD, 0, 0x0F, 2)

    def adjust_position(self, delta_deg: float) -> None:
        with self.state_lock:
            self.current_target_deg += delta_deg
            target = self.current_target_deg
        self._push_position()
        self._trigger_motion()
        self._log(f"[Steer] Target angle set to {target:.2f} deg")

    def set_position(self, value_deg: float) -> None:
        with self.state_lock:
            self.current_target_deg = value_deg
        self._push_position()
        self._trigger_motion()
        self._log(f"[Steer] Target angle forced to {value_deg:.2f} deg")

    def adjust_offset(self, node_id: int, delta_deg: float) -> None:
        if node_id not in self.nodes:
            return
        with self.state_lock:
            self.angle_offsets_deg[node_id] += delta_deg
            offset = self.angle_offsets_deg[node_id]
        self._push_position()
        self._trigger_motion()
        self._log(f"[Calib] Node {node_id} offset adjusted to {offset:.3f} deg")

    def save_offsets(self) -> None:
        with self.state_lock:
            data_to_save = {str(k): v for k, v in self.angle_offsets_deg.items()}
        try:
            self.offset_path.write_text(json.dumps(data_to_save, indent=4))
            self._log(f"[Calib] Offsets saved to {self.offset_path}")
        except Exception as exc:  # pragma: no cover - file system failure
            self._log(f"[Error] Failed to save offsets: {exc}")

    def load_offsets(self) -> None:
        if not self.offset_path.exists():
            self._log("[Info] Offset file not found. Using default zeros.")
            return
        try:
            loaded_data = json.loads(self.offset_path.read_text())
            loaded_offsets = {int(k): v for k, v in loaded_data.items()}
            with self.state_lock:
                for node_id in self.nodes:
                    if node_id in loaded_offsets:
                        self.angle_offsets_deg[node_id] = loaded_offsets[node_id]
            self._log(f"[Calib] Offsets loaded from {self.offset_path}")
            self.print_offsets()
        except Exception as exc:
            self._log(f"[Error] Failed to load offsets: {exc}")

    def print_offsets(self) -> None:
        with self.state_lock:
            offsets_copy = self.angle_offsets_deg.copy()
        self._log("[Calib] Current steer offsets (deg):")
        for node, offset in sorted(offsets_copy.items()):
            self._log(f"  Node {node}: {offset:.3f}")

    def adjust_velocity(self, delta_dps: float) -> None:
        with self.state_lock:
            self.profile_velocity_dps = max(
                STEER_MIN_VEL_DPS, self.profile_velocity_dps + delta_dps
            )
            velocity = self.profile_velocity_dps
        self._push_profile_velocity()
        self._log(f"[Steer] Profile velocity set to {velocity:.2f} deg/s")

    def adjust_accel(self, delta_dps2: float) -> None:
        with self.state_lock:
            self.profile_accel_dps2 = max(
                STEER_MIN_ACCEL_DPS2, self.profile_accel_dps2 + delta_dps2
            )
            accel = self.profile_accel_dps2
        self._push_profile_accel()
        self._log(f"[Steer] Profile accel set to {accel:.2f} deg/s²")

    def fetch_status(self) -> Dict[str, object]:
        with self.state_lock:
            tgt_deg = self.current_target_deg
            vel_dps = self.profile_velocity_dps
            accel_dps2 = self.profile_accel_dps2
            offsets = self.angle_offsets_deg.copy()

        actual_angle: Dict[int, Optional[float]] = {}
        actual_velocity: Dict[int, Optional[float]] = {}
        for node, bus_name in self._for_each_node():
            try:
                raw_pos = send_sdo_read(bus_name, node, OD_ACTUAL_POSITION, 0, STEER_READ_TIMEOUT)
                raw_vel = send_sdo_read(bus_name, node, OD_ACTUAL_VELOCITY, 0, STEER_READ_TIMEOUT)
                actual_angle[node] = pulses_to_deg(raw_pos)
                actual_velocity[node] = pps_to_dps(raw_vel)
            except Exception as exc:
                actual_angle[node] = None
                actual_velocity[node] = None
                self._log(f"[Steer] Read failed on node {node}: {exc}")

        return {
            "target_deg": tgt_deg,
            "profile_vel_dps": vel_dps,
            "profile_accel_dps2": accel_dps2,
            "actual_angle_deg": actual_angle,
            "actual_velocity_dps": actual_velocity,
            "offsets_deg": offsets,
        }

    def print_profile_accel(self) -> None:
        status = self.fetch_status()
        pulses = dps2_to_pps2(status["profile_accel_dps2"])
        self._log("[Steer] Profile accel per node (deg/s²):")
        for node in self.nodes:
            self._log(
                f"  Node {node}: {status['profile_accel_dps2']:.2f} deg/s² ({pulses} pps²)"
            )

    def print_profile_velocity(self) -> None:
        status = self.fetch_status()
        pulses = dps_to_pps(status["profile_vel_dps"])
        self._log("[Steer] Profile velocity per node (deg/s):")
        for node in self.nodes:
            self._log(
                f"  Node {node}: {status['profile_vel_dps']:.2f} deg/s ({pulses} pps)"
            )

    def print_actual_position(self) -> None:
        status = self.fetch_status()
        self._log("[Steer] Actual position/velocity:")
        for node in self.nodes:
            angle = status["actual_angle_deg"].get(node)
            vel = status["actual_velocity_dps"].get(node)
            angle_txt = f"{angle:.2f}" if angle is not None else "N/A"
            vel_txt = f"{vel:.2f}" if vel is not None else "N/A"
            self._log(f"  Node {node}: angle {angle_txt} deg, vel {vel_txt} deg/s")


__all__ = [
    "DRIVE_NODE_CONFIG",
    "DRIVE_NODES_TO_INVERT",
    "DRIVE_SPEED_STEP_MPS",
    "DRIVE_ACCEL_STEP_PPS2",
    "STEER_NODE_CONFIG",
    "STEER_DEG_STEP",
    "STEER_VEL_STEP_DPS",
    "STEER_ACCEL_STEP_DPS2",
    "STEER_OFFSET_STEP_DEG",
    "DEFAULT_STEER_OFFSET_FILENAME",
    "DriveController",
    "SteerController",
    "ensure_buses",
    "mps_to_pps",
    "pps_to_mps",
    "pps2_to_mps2",
    "mps2_to_pps2",
    "deg_to_pulses",
    "pulses_to_deg",
    "dps_to_pps",
    "pps_to_dps",
    "dps2_to_pps2",
    "pps2_to_dps2",
]
