#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import math
import struct
import sys
import threading
import time
from typing import Dict, Iterable, List, Optional, Set
import json

import can
from pynput import keyboard

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
OD_INTERNAL_POSITION = 0x6063
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

# ✨ Calibration & profiles ---------------------------------------------------
STEER_OFFSET_STEP_DEG = 0.001
STEER_OFFSET_STEP_DEG1 = 0.01
STEER_OFFSET_STEP_DEG2 = 0.1
STEER_OFFSET_FILENAME = "steer_offsets.json"             # 레거시 단일 파일
PROFILE_JSON = "steer_offset_profiles.json"              # 신형 3프로파일 파일
AUTO_SWITCH_SPEED_EPS = 0.02                             # |v| <= eps → neutral

# 기본 프로파일 값
DEFAULT_FORWARD_OFFSETS = {2: -0.656, 4: -0.686, 6: -0.138, 8: -0.138}  # 10차 직진
DEFAULT_REVERSE_OFFSETS = {2: -0.726, 4: -0.756, 6: -0.484, 8: -0.484}  # 14차 후진
DEFAULT_NEUTRAL_OFFSETS = {2: -0.656, 4: -0.686, 6: -0.313, 8: -0.313}  # 2차 near-stop

# 현재 보정 스텝(기본: 가장 미세 스텝)
current_offset_step_deg = STEER_OFFSET_STEP_DEG

# --- Zero-radius rotation constants ----------------------------------------
ZERO_TURN_WHEEL_BASE_M = 1.1211306
ZERO_TURN_TRACK_FRONT_M = 0.8415904
ZERO_TURN_TRACK_REAR_M = 0.6965428
ZERO_TURN_STEP_ANGLE_DEG = 0.2
ZERO_TURN_STEP_DURATION_S = 0.4
ZERO_TURN_STEER_SETTLE_S = 2.0
ZERO_TURN_STEER_NODES = {2, 4, 6, 8}
ZERO_TURN_DRIVE_NODES = {1, 3, 5, 7}
ZERO_TURN_STEER_NODE_BY_WHEEL = {"FL": 2, "FR": 4, "RL": 6, "RR": 8}
ZERO_TURN_DRIVE_NODE_BY_WHEEL = {"FL": 1, "FR": 3, "RL": 5, "RR": 7}

# --- Shared CAN infrastructure ---------------------------------------------
BUS_POOL: Dict[str, can.interface.Bus] = {}
BUS_LOCKS: Dict[str, threading.Lock] = {}
BUS_POOL_LOCK = threading.Lock()

print_lock = threading.Lock()
status_len_lock = threading.Lock()
last_status_len = 0
is_running = True

def ensure_buses(channels: Iterable[str]) -> None:
    with BUS_POOL_LOCK:
        for name in channels:
            if name in BUS_POOL:
                continue
            BUS_POOL[name] = can.interface.Bus(channel=name, bustype="socketcan")
            BUS_LOCKS[name] = threading.Lock()

def shutdown_buses() -> None:
    with BUS_POOL_LOCK:
        for bus in BUS_POOL.values():
            bus.shutdown()
        BUS_POOL.clear()
        BUS_LOCKS.clear()

def send_sdo_write(
    bus_name: str,
    node_id: int,
    index: int,
    sub_index: int,
    value: int,
    size: int,
    sleep: float = 0.01,
) -> None:
    if bus_name not in BUS_POOL:
        raise RuntimeError(f"Bus '{bus_name}' not initialised.")
    if size == 1:
        cmd, fmt = 0x2F, "<B"
    elif size == 2:
        cmd, fmt = 0x2B, "<H"
    elif size == 4:
        cmd, fmt = 0x23, "<i"
    else:
        raise ValueError("Unsupported SDO size (must be 1/2/4).")

    cob_id = 0x600 + node_id
    index_bytes = struct.pack("<H", index)
    if size == 1:
        value &= 0xFF
    elif size == 2:
        value &= 0xFFFF
    data_bytes = struct.pack(fmt, value)
    payload = [cmd, index_bytes[0], index_bytes[1], sub_index] + list(data_bytes)
    payload.extend([0] * (8 - len(payload)))
    msg = can.Message(arbitration_id=cob_id, data=payload, is_extended_id=False)

    lock = BUS_LOCKS[bus_name]
    with lock:
        BUS_POOL[bus_name].send(msg)
        time.sleep(sleep)

def send_sdo_read(
    bus_name: str,
    node_id: int,
    index: int,
    sub_index: int,
    timeout: float,
) -> int:
    if bus_name not in BUS_POOL:
        raise RuntimeError(f"Bus '{bus_name}' not initialised.")

    cob_id_req = 0x600 + node_id
    cob_id_resp = 0x580 + node_id
    payload = [0x40, index & 0xFF, index >> 8, sub_index, 0, 0, 0, 0]
    request = can.Message(arbitration_id=cob_id_req, data=payload, is_extended_id=False)

    lock = BUS_LOCKS[bus_name]
    with lock:
        BUS_POOL[bus_name].send(request)
        deadline = time.time() + timeout
        while time.time() < deadline:
            response = BUS_POOL[bus_name].recv(timeout=timeout)
            if response is None:
                continue
            if response.arbitration_id != cob_id_resp:
                continue
            cmd = response.data[0]
            if cmd == 0x60:
                # Write acknowledgement leaked into the queue; drop it and continue.
                continue
            if (cmd & 0xE0) != 0x40:
                raise RuntimeError(f"Unexpected SDO response 0x{cmd:02X}")
            expedited = bool(cmd & 0x02)
            if not expedited:
                raise RuntimeError("Segmented SDO transfers are not supported.")
            if cmd & 0x01:
                unused = (cmd >> 2) & 0x03
                data_len = 4 - unused
            else:
                data_len = 4
            raw = bytes(response.data[4:4 + data_len]) + b"\x00" * (4 - data_len)
            return struct.unpack("<i", raw)[0]
    raise TimeoutError(f"SDO read timeout 0x{index:04X}:{sub_index} on node {node_id}")

def padded_status_print(text: str) -> None:
    global last_status_len
    with status_len_lock:
        pad = " " * max(0, last_status_len - len(text))
        msg = "\r" + text + pad
        with print_lock:
            sys.stdout.write(msg)
            sys.stdout.flush()
        last_status_len = len(text)

# --- Unit helpers ------------------------------------------------------------
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

# --- Drive controller --------------------------------------------------------
class DriveController:
    def __init__(self, nodes: List[int]):
        self.nodes = nodes
        self.node_to_bus = {node: DRIVE_NODE_CONFIG[node] for node in nodes}
        self.current_speed_mps = 0.0
        self.current_accel_pps2 = 50000
        self.state_lock = threading.Lock()

    def setup(self) -> None:
        for node in self.nodes:
            bus_name = self.node_to_bus[node]
            if node in DRIVE_NODES_TO_INVERT:
                send_sdo_write(bus_name, node, OD_MOTOR_DIRECTION, 0, 64, 1)
                with print_lock:
                    print(f"\n[Setup] Node {node} direction has been inverted.")

            send_sdo_write(bus_name, node, OD_CONTROL_WORD, 0, 0x06, 2)
            send_sdo_write(bus_name, node, OD_CONTROL_WORD, 0, 0x07, 2)
            send_sdo_write(bus_name, node, OD_MODES_OF_OPERATION, 0, 3, 1)
            send_sdo_write(bus_name, node, OD_CONTROL_WORD, 0, 0x0F, 2)
            self._push_accel_to_node(node)
            self._push_speed_to_node(node)

    def _for_each_node(self):
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

        with print_lock:
            print(f"\n[Drive] Target speed set to {target:.2f} m/s")

    def set_speed(self, value: float) -> None:
        with self.state_lock:
            self.current_speed_mps = value
        for node, _ in self._for_each_node():
            self._push_speed_to_node(node)
        with print_lock:
            print(f"\n[Drive] Target speed forced to {value:.2f} m/s")

    def adjust_accel(self, delta_pps2: int) -> None:
        with self.state_lock:
            self.current_accel_pps2 = max(
                DRIVE_MIN_ACCEL_PPS2, self.current_accel_pps2 + delta_pps2
            )
            accel_pps2 = self.current_accel_pps2
        for node, _ in self._for_each_node():
            self._push_accel_to_node(node)
        accel_mps2 = pps2_to_mps2(accel_pps2)
        with print_lock:
            print(f"\n[Drive] Profile accel set to {accel_mps2:.2f} m/s² ({accel_pps2} pps²)")

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
                with print_lock:
                    print(f"\n[Drive] Read failed on node {node}: {exc}")

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
        with print_lock:
            print("\n[Drive] Actual velocity (m/s):")
            for node, value in status["actual_velocity_mps"].items():
                print(f"  Node {node}: {value:.3f}" if value is not None else f"  Node {node}: N/A")
            print("[Drive] Actual position (m):")
            for node, value in status["actual_position_m"].items():
                print(f"  Node {node}: {value:.3f}" if value is not None else f"  Node {node}: N/A")

# --- Steer controller w/ profiles -------------------------------------------
class SteerController:
    def __init__(self, nodes: List[int]):
        self.nodes = nodes
        self.node_to_bus = {node: STEER_NODE_CONFIG[node] for node in nodes}
        self.current_target_deg = 0.0
        self.profile_velocity_dps = 30.0
        self.profile_accel_dps2 = 80.0
        self.state_lock = threading.Lock()

        # 프로파일 상태
        self.profile_mode_auto = True              # Auto / Manual
        self.active_profile = "forward"            # "forward" | "reverse" | "neutral"

        # 각 프로파일 저장소
        self.offsets_forward_deg: Dict[int, float] = {node: 0.0 for node in nodes}
        self.offsets_reverse_deg: Dict[int, float] = {node: 0.0 for node in nodes}
        self.offsets_neutral_deg: Dict[int, float] = {node: 0.0 for node in nodes}

        # 런타임 적용 딕셔너리 (현재 활성 프로파일이 여기 복사됨)
        self.angle_offsets_deg: Dict[int, float] = {node: 0.0 for node in nodes}

        # 프로파일 로드 (없으면 기본값 주입 + 레거시 흡수)
        if not self.load_profiles():
            for n in self.nodes:
                if n in DEFAULT_FORWARD_OFFSETS:
                    self.offsets_forward_deg[n] = DEFAULT_FORWARD_OFFSETS[n]
                if n in DEFAULT_REVERSE_OFFSETS:
                    self.offsets_reverse_deg[n] = DEFAULT_REVERSE_OFFSETS[n]
                if n in DEFAULT_NEUTRAL_OFFSETS:
                    self.offsets_neutral_deg[n] = DEFAULT_NEUTRAL_OFFSETS[n]
            # 레거시 단일 파일을 발견하면 forward에 흡수
            self._import_legacy_into_forward()
        self._sync_active_into_runtime()

    # ----- Setup & motion ----------------------------------------------------
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

    def _for_each_node(self):
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

    # ----- Commands ----------------------------------------------------------
    def adjust_position(self, delta_deg: float) -> None:
        with self.state_lock:
            self.current_target_deg += delta_deg
            target = self.current_target_deg
        self._push_position()
        self._trigger_motion()
        with print_lock:
            print(f"\n[Steer] Target angle set to {target:.2f} deg")

    def set_position(self, value_deg: float) -> None:
        with self.state_lock:
            self.current_target_deg = value_deg
        self._push_position()
        self._trigger_motion()
        with print_lock:
            print(f"\n[Steer] Target angle forced to {value_deg:.2f} deg")

    def adjust_offset(self, node_id: int, delta_deg: float) -> None:
        if node_id not in self.nodes:
            return
        with self.state_lock:
            self.angle_offsets_deg[node_id] += delta_deg
            self._write_back_to_profile(node_id, self.angle_offsets_deg[node_id])
        self._push_position()
        self._trigger_motion()
        with print_lock:
            offset = self.angle_offsets_deg[node_id]
            print(f"\n[Calib] Node {node_id} offset adjusted to {offset:.3f} deg ({self.active_profile})")

    # ----- Legacy single-file save/load -------------------------------------
    def save_offsets(self) -> None:
        with self.state_lock:
            data_to_save = {str(k): v for k, v in self.angle_offsets_deg.items()}
        try:
            with open(STEER_OFFSET_FILENAME, "w") as f:
                json.dump(data_to_save, f, indent=4)
            with print_lock:
                print(f"\n[Calib] Offsets saved to {STEER_OFFSET_FILENAME} (current profile only)")
        except Exception as e:
            with print_lock:
                print(f"\n[Error] Failed to save offsets: {e}")

    def load_offsets(self) -> None:
        try:
            with open(STEER_OFFSET_FILENAME, "r") as f:
                loaded_data = json.load(f)
            loaded_offsets = {int(k): v for k, v in loaded_data.items()}
            with self.state_lock:
                for node_id in self.nodes:
                    if node_id in loaded_offsets:
                        self.angle_offsets_deg[node_id] = loaded_offsets[node_id]
                        self._write_back_to_profile(node_id, loaded_offsets[node_id])
            with print_lock:
                print(f"\n[Calib] Offsets loaded from {STEER_OFFSET_FILENAME} -> applied to current profile ({self.active_profile})")
            self._push_position()
            self._trigger_motion()
            self.print_offsets()
        except FileNotFoundError:
            with print_lock:
                print(f"\n[Info] Offset file not found. Using current values.")
        except Exception as e:
            with print_lock:
                print(f"\n[Error] Failed to load offsets: {e}")

    def _import_legacy_into_forward(self) -> None:
        try:
            with open(STEER_OFFSET_FILENAME, "r") as f:
                loaded_data = json.load(f)
            loaded_offsets = {int(k): v for k, v in loaded_data.items()}
            with self.state_lock:
                for n in self.nodes:
                    if n in loaded_offsets:
                        self.offsets_forward_deg[n] = loaded_offsets[n]
            with print_lock:
                print(f"\n[Calib] Imported legacy {STEER_OFFSET_FILENAME} into FORWARD profile")
        except FileNotFoundError:
            pass

    # ----- Profiles save/load -----------------------------------------------
    def save_profiles(self) -> None:
        try:
            with self.state_lock:
                data = {
                    "active_profile": self.active_profile,
                    "mode": "auto" if self.profile_mode_auto else "manual",
                    "profiles": {
                        "forward": {str(k): self.offsets_forward_deg.get(k, 0.0) for k in self.nodes},
                        "reverse": {str(k): self.offsets_reverse_deg.get(k, 0.0) for k in self.nodes},
                        "neutral": {str(k): self.offsets_neutral_deg.get(k, 0.0) for k in self.nodes},
                    },
                }
            with open(PROFILE_JSON, "w") as f:
                json.dump(data, f, indent=4)
            with print_lock:
                print(f"\n[Calib] Profiles saved to {PROFILE_JSON}")
        except Exception as e:
            with print_lock:
                print(f"\n[Error] Failed to save profiles: {e}")

    def load_profiles(self) -> bool:
        try:
            with open(PROFILE_JSON, "r") as f:
                data = json.load(f)
            profs = data.get("profiles", {})
            fwd = {int(k): float(v) for k, v in profs.get("forward", {}).items()}
            rev = {int(k): float(v) for k, v in profs.get("reverse", {}).items()}
            neu = {int(k): float(v) for k, v in profs.get("neutral", {}).items()}
            with self.state_lock:
                for n in self.nodes:
                    self.offsets_forward_deg[n] = fwd.get(n, DEFAULT_FORWARD_OFFSETS.get(n, 0.0))
                    self.offsets_reverse_deg[n] = rev.get(n, DEFAULT_REVERSE_OFFSETS.get(n, 0.0))
                    self.offsets_neutral_deg[n] = neu.get(n, DEFAULT_NEUTRAL_OFFSETS.get(n, 0.0))
                self.active_profile = data.get("active_profile", "forward")
                self.profile_mode_auto = (data.get("mode", "auto") == "auto")
                self._sync_active_into_runtime()
            with print_lock:
                print(f"\n[Calib] Profiles loaded from {PROFILE_JSON} (active: {self.active_profile}, mode: {'Auto' if self.profile_mode_auto else 'Manual'})")
            return True
        except FileNotFoundError:
            with print_lock:
                print(f"\n[Info] Profile file not found. Using defaults.")
            return False
        except Exception as e:
            with print_lock:
                print(f"\n[Error] Failed to load profiles: {e}")
            return False

    # ----- Profiles runtime helpers -----------------------------------------
    def _sync_active_into_runtime(self) -> None:
        if self.active_profile == "forward":
            src = self.offsets_forward_deg
        elif self.active_profile == "reverse":
            src = self.offsets_reverse_deg
        else:
            src = self.offsets_neutral_deg
        with self.state_lock:
            for n in self.nodes:
                self.angle_offsets_deg[n] = src.get(n, 0.0)

    def _write_back_to_profile(self, node_id: int, value: float) -> None:
        if self.active_profile == "forward":
            self.offsets_forward_deg[node_id] = value
        elif self.active_profile == "reverse":
            self.offsets_reverse_deg[node_id] = value
        else:
            self.offsets_neutral_deg[node_id] = value

    def set_profile(self, name: str, reason: str = "") -> None:
        if name not in ("forward", "reverse", "neutral"):
            return
        with self.state_lock:
            if self.active_profile == name:
                return
            self.active_profile = name
            self._sync_active_into_runtime()
        self._push_position()
        self._trigger_motion()
        with print_lock:
            msg = f"\n[Calib] Profile -> {self.active_profile.upper()}"
            if reason:
                msg += f" ({reason})"
            print(msg)

    def set_mode_auto(self, enable: bool) -> None:
        with self.state_lock:
            self.profile_mode_auto = enable
        with print_lock:
            print(f"\n[Calib] Profile mode: {'Auto' if enable else 'Manual'}")

    # ----- Telemetry & prints ------------------------------------------------
    def print_offsets(self) -> None:
        with self.state_lock, print_lock:
            print("\n[Calib] Current steer offsets (deg):")
            for node in sorted(self.nodes):
                off = self.angle_offsets_deg[node]
                print(f"  Node {node}: {off:.3f} ({self.active_profile})")
            print("\n[Calib] FORWARD profile:")
            for node in sorted(self.nodes):
                print(f"  Node {node}: {self.offsets_forward_deg[node]:.3f}")
            print("\n[Calib] REVERSE profile:")
            for node in sorted(self.nodes):
                print(f"  Node {node}: {self.offsets_reverse_deg[node]:.3f}")
            print("\n[Calib] NEUTRAL profile:")
            for node in sorted(self.nodes):
                print(f"  Node {node}: {self.offsets_neutral_deg[node]:.3f}")

    def adjust_velocity(self, delta_dps: float) -> None:
        with self.state_lock:
            self.profile_velocity_dps = max(
                STEER_MIN_VEL_DPS, self.profile_velocity_dps + delta_dps
            )
            velocity = self.profile_velocity_dps
        self._push_profile_velocity()
        with print_lock:
            print(f"\n[Steer] Profile velocity set to {velocity:.2f} deg/s")

    def adjust_accel(self, delta_dps2: float) -> None:
        with self.state_lock:
            self.profile_accel_dps2 = max(
                STEER_MIN_ACCEL_DPS2, self.profile_accel_dps2 + delta_dps2
            )
            accel = self.profile_accel_dps2
        self._push_profile_accel()
        with print_lock:
            print(f"\n[Steer] Profile accel set to {accel:.2f} deg/s²")

    def fetch_status(self) -> Dict[str, object]:
        with self.state_lock:
            tgt_deg = self.current_target_deg
            vel_dps = self.profile_velocity_dps
            accel_dps2 = self.profile_accel_dps2
            offsets = self.angle_offsets_deg.copy()
            prof = self.active_profile
            auto = self.profile_mode_auto

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
                with print_lock:
                    print(f"\n[Steer] Read failed on node {node}: {exc}")

        return {
            "target_deg": tgt_deg,
            "profile_vel_dps": vel_dps,
            "profile_accel_dps2": accel_dps2,
            "actual_angle_deg": actual_angle,
            "actual_velocity_dps": actual_velocity,
            "offsets_deg": offsets,
            "profile": prof,
            "profile_mode_auto": auto,
        }

    def print_profile_accel(self) -> None:
        status = self.fetch_status()
        with print_lock:
            print("\n[Steer] Profile accel per node (deg/s²):")
            pulses = dps2_to_pps2(status["profile_accel_dps2"])
            for node in self.nodes:
                print(f"  Node {node}: {status['profile_accel_dps2']:.2f} deg/s² ({pulses} pps²)")

    def print_profile_velocity(self) -> None:
        status = self.fetch_status()
        with print_lock:
            print("\n[Steer] Profile velocity per node (deg/s):")
            pulses = dps_to_pps(status["profile_vel_dps"])
            for node in self.nodes:
                print(f"  Node {node}: {status['profile_vel_dps']:.2f} deg/s ({pulses} pps)")

    def print_actual_position(self) -> None:
        status = self.fetch_status()
        with print_lock:
            print("\n[Steer] Actual position/velocity:")
            for node in self.nodes:
                angle = status["actual_angle_deg"].get(node)
                vel = status["actual_velocity_dps"].get(node)
                angle_txt = f"{angle:.2f}" if angle is not None else "N/A"
                vel_txt = f"{vel:.2f}" if vel is not None else "N/A"
                print(f"  Node {node}: angle {angle_txt} deg, vel {vel_txt} deg/s")

    def print_snapshot(self) -> None:
        with print_lock:
            print("\n[Steer] Snapshot (raw pulses & calibrated angle):")
        for node, bus_name in self._for_each_node():
            try:
                raw_internal = send_sdo_read(bus_name, node, OD_INTERNAL_POSITION, 0, STEER_READ_TIMEOUT)
                raw_feedback = send_sdo_read(bus_name, node, OD_ACTUAL_POSITION, 0, STEER_READ_TIMEOUT)
                angle_raw = pulses_to_deg(raw_feedback)
                with self.state_lock:
                    offset = self.angle_offsets_deg.get(node, 0.0)
                calibrated_angle = angle_raw - offset
                with print_lock:
                    print(
                        f"  Node {node}: internal {raw_internal:,} pulses | "
                        f"feedback {raw_feedback:,} pulses"
                    )
                    print(
                        f"            angle {angle_raw:+.3f}° raw | "
                        f"calibrated {calibrated_angle:+.3f}°"
                    )
            except Exception as exc:
                with print_lock:
                    print(f"  Node {node}: snapshot read failed ({exc})")

    def get_offset(self, node_id: int) -> float:
        with self.state_lock:
            return self.angle_offsets_deg.get(node_id, 0.0)

    def apply_absolute_targets(self, targets_deg: Dict[int, float], label: str = "") -> None:
        pending_nodes = {node for node in targets_deg if node in self.nodes}
        if not pending_nodes:
            return
        for node, bus_name in self._for_each_node():
            if node not in pending_nodes:
                continue
            desired = targets_deg[node]
            with self.state_lock:
                offset = self.angle_offsets_deg.get(node, 0.0)
            pulses = deg_to_pulses(desired + offset)
            send_sdo_write(bus_name, node, OD_TARGET_POSITION, 0, pulses, 4)
        self._trigger_motion()
        if label:
            with print_lock:
                info = " ".join(
                    f"{node}:{targets_deg[node]:+.2f}°" for node in sorted(pending_nodes)
                )
                print(f"\n[Steer] {label} -> {info}")

# --- ZeroRadiusHelper (unchanged logic, minor comments) ---------------------
class ZeroRadiusHelper:
    def __init__(
        self,
        drive_nodes: List[int],
        steer_nodes: List[int],
        drive_controller: Optional[DriveController],
        steer_controller: Optional[SteerController],
    ) -> None:
        self.drive_nodes = set(drive_nodes)
        self.steer_nodes = set(steer_nodes)
        self.drive_controller = drive_controller
        self.steer_controller = steer_controller
        self.available = (
            ZERO_TURN_DRIVE_NODES.issubset(self.drive_nodes)
            and ZERO_TURN_STEER_NODES.issubset(self.steer_nodes)
            and steer_controller is not None
        )

        self.step_angle_deg = ZERO_TURN_STEP_ANGLE_DEG
        self.step_duration = ZERO_TURN_STEP_DURATION_S
        self.cumulative_deg = 0.0
        self.motion_lock = threading.Lock()
        self.step_in_progress = False

        self.steer_targets_deg_by_node: Dict[int, float] = {}
        self.drive_velocity_pps_by_node: Dict[int, int] = {}
        self.hardware_inverted_nodes: Set[int] = set()

        if self.available:
            angles_by_wheel = self._calculate_zero_turn_angles()
            self.steer_targets_deg_by_node = {
                ZERO_TURN_STEER_NODE_BY_WHEEL[wheel]: angle
                for wheel, angle in angles_by_wheel.items()
            }
            self.drive_velocity_pps_by_node = self._precompute_drive_velocity()
            if self.drive_controller:
                self.hardware_inverted_nodes = (
                    set(self.drive_controller.nodes) & DRIVE_NODES_TO_INVERT
                )

    def _calculate_zero_turn_angles(self) -> Dict[str, float]:
        coords = {
            "FL": (ZERO_TURN_WHEEL_BASE_M / 2.0, ZERO_TURN_TRACK_FRONT_M / 2.0),
            "FR": (ZERO_TURN_WHEEL_BASE_M / 2.0, -ZERO_TURN_TRACK_FRONT_M / 2.0),
            "RL": (-ZERO_TURN_WHEEL_BASE_M / 2.0, ZERO_TURN_TRACK_REAR_M / 2.0),
            "RR": (-ZERO_TURN_WHEEL_BASE_M / 2.0, -ZERO_TURN_TRACK_REAR_M / 2.0),
        }
        angles: Dict[str, float] = {}
        for wheel, (x, y) in coords.items():
            radial = math.atan2(y, x)
            angle_deg = math.degrees(radial) + 90.0
            while angle_deg > 180.0:
                angle_deg -= 360.0
            while angle_deg <= -180.0:
                angle_deg += 360.0
            angles[wheel] = angle_deg
        return angles

    def _precompute_drive_velocity(self) -> Dict[int, int]:
        theta = math.radians(self.step_angle_deg)
        velocity_pps: Dict[int, int] = {}
        for wheel, node_id in ZERO_TURN_DRIVE_NODE_BY_WHEEL.items():
            if wheel.startswith("F"):
                lateral = ZERO_TURN_TRACK_FRONT_M / 2.0
            else:
                lateral = ZERO_TURN_TRACK_REAR_M / 2.0
            longitudinal = ZERO_TURN_WHEEL_BASE_M / 2.0
            if wheel in ("RL", "RR"):
                longitudinal *= -1.0
            radius = math.hypot(longitudinal, lateral)
            arc_length = radius * theta
            wheel_revs = arc_length / WHEEL_CIRCUMFERENCE_M
            motor_revs = wheel_revs * DRIVE_REDUCTION_RATIO
            pulses = motor_revs * FEED_CONSTANT
            velocity = int(round(pulses / self.step_duration))
            velocity_pps[node_id] = velocity
        return velocity_pps

    def _validate(self) -> bool:
        if not self.available:
            with print_lock:
                print("\n[ZeroTurn] Full 4-wheel configuration required; skipping command.")
            return False
        return True

    def align_steer(self) -> None:
        if not self._validate() or not self.steer_controller:
            return
        if self.drive_controller:
            self.drive_controller.set_speed(0.0)
        self.steer_controller.apply_absolute_targets(
            self.steer_targets_deg_by_node,
            label="Zero-turn alignment",
        )
        with self.motion_lock:
            self.cumulative_deg = 0.0
        with print_lock:
            print(
                f"[ZeroTurn] Allow ~{ZERO_TURN_STEER_SETTLE_S:.1f}s for steering to settle before rotating."
            )
            wheel_info = " ".join(
                f"{wheel}:{self.steer_targets_deg_by_node[ZERO_TURN_STEER_NODE_BY_WHEEL[wheel]]:+.2f}°"
                for wheel in ("FL", "FR", "RL", "RR")
            )
            print(f"[ZeroTurn] Targets: {wheel_info}")

    def restore_straight(self) -> None:
        if not self.steer_controller:
            return
        if self.drive_controller:
            self.drive_controller.set_speed(0.0)
        self.steer_controller.set_position(0.0)
        with self.motion_lock:
            self.cumulative_deg = 0.0
        with print_lock:
            print("[ZeroTurn] Steering returned to 0°.")

    def request_step(self, direction: int) -> None:
        if direction not in (-1, 1):
            return
        if not self._validate():
            return
        with self.motion_lock:
            if self.step_in_progress:
                with print_lock:
                    print("\n[ZeroTurn] Step already in progress; command ignored.")
                return
            self.step_in_progress = True
        threading.Thread(target=self._run_step, args=(direction,), daemon=True).start()

    def _apply_drive_velocities(self, direction: int) -> None:
        for node_id, base_velocity in self.drive_velocity_pps_by_node.items():
            if node_id not in self.drive_nodes:
                continue
            velocity = base_velocity
            if (
                node_id in DRIVE_NODES_TO_INVERT
                and node_id not in self.hardware_inverted_nodes
            ):
                velocity *= -1
            bus_name = DRIVE_NODE_CONFIG[node_id]
            send_sdo_write(bus_name, node_id, OD_TARGET_VELOCITY, 0, direction * velocity, 4)

    def _stop_drives(self) -> None:
        for node_id in ZERO_TURN_DRIVE_NODES:
            if node_id not in self.drive_nodes:
                continue
            bus_name = DRIVE_NODE_CONFIG[node_id]
            send_sdo_write(bus_name, node_id, OD_TARGET_VELOCITY, 0, 0, 4)

    def _run_step(self, direction: int) -> None:
        try:
            if self.drive_controller:
                self.drive_controller.set_speed(0.0)
            self._apply_drive_velocities(direction)
            time.sleep(self.step_duration)
        finally:
            self._stop_drives()
            with self.motion_lock:
                self.step_in_progress = False
        self.cumulative_deg += direction * self.step_angle_deg
        with print_lock:
            print(f"\n[ZeroTurn] Cumulative rotation {self.cumulative_deg:+6.2f}°")

# --- Keyboard handling -------------------------------------------------------
drive_controller: Optional[DriveController] = None
steer_controller: Optional[SteerController] = None
selected_steer_node: Optional[int] = None
zero_radius_helper: Optional[ZeroRadiusHelper] = None
active_drive_nodes: List[int] = []
active_steer_nodes: List[int] = []

def on_press(key: keyboard.Key) -> bool:
    global is_running, selected_steer_node, current_offset_step_deg
    try:
        char = key.char.lower()
    except AttributeError:
        if key == keyboard.Key.esc:
            is_running = False
            return False

        # 오프셋 트림: ←/→
        if steer_controller and selected_steer_node:
            if key == keyboard.Key.left:
                steer_controller.adjust_offset(selected_steer_node, -current_offset_step_deg)
            elif key == keyboard.Key.right:
                steer_controller.adjust_offset(selected_steer_node, current_offset_step_deg)

        # F1/F2/F3: 오프셋 스텝
        if key == keyboard.Key.f1:
            current_offset_step_deg = STEER_OFFSET_STEP_DEG
            with print_lock:
                print(f"\n[Calib] Offset step set to {current_offset_step_deg:.3f} deg (F1)")
        elif key == keyboard.Key.f2:
            current_offset_step_deg = STEER_OFFSET_STEP_DEG1
            with print_lock:
                print(f"\n[Calib] Offset step set to {current_offset_step_deg:.3f} deg (F2)")
        elif key == keyboard.Key.f3:
            current_offset_step_deg = STEER_OFFSET_STEP_DEG2
            with print_lock:
                print(f"\n[Calib] Offset step set to {current_offset_step_deg:.3f} deg (F3)")
        # F4: Auto/Manual 토글
        elif key == keyboard.Key.f4 and steer_controller:
            steer_controller.set_mode_auto(not steer_controller.profile_mode_auto)
        # F5/F6: 프로파일 파일 일괄 저장/로드
        elif key == keyboard.Key.f5 and steer_controller:
            steer_controller.save_profiles()
        elif key == keyboard.Key.f6 and steer_controller:
            steer_controller.load_profiles()
        return True

    # --- Drive keys ---
    if char == "i" and drive_controller:
        drive_controller.adjust_speed(DRIVE_SPEED_STEP_MPS)
    elif char == "k" and drive_controller:
        drive_controller.adjust_speed(-DRIVE_SPEED_STEP_MPS)
    elif char == "m" and drive_controller:
        drive_controller.set_speed(0.0)
    elif char == "u" and drive_controller:
        drive_controller.adjust_accel(DRIVE_ACCEL_STEP_PPS2)
    elif char == "j" and drive_controller:
        drive_controller.adjust_accel(-DRIVE_ACCEL_STEP_PPS2)
    elif char == "t" and drive_controller:
        drive_controller.print_actual_state()

    # --- Steer angle commands ---
    elif char == "q" and steer_controller:
        steer_controller.adjust_position(STEER_DEG_STEP)
    elif char == "e" and steer_controller:
        steer_controller.adjust_position(-STEER_DEG_STEP)
    elif char == "w" and steer_controller:
        steer_controller.set_position(0.0)
    elif char == "a" and steer_controller:
        steer_controller.set_position(-90.0)
    elif char == "d" and steer_controller:
        steer_controller.set_position(90.0)
    elif char == "r" and steer_controller:
        steer_controller.print_snapshot()

    # --- Zero-turn ---
    elif char == "z" and zero_radius_helper:
        zero_radius_helper.align_steer()
    elif char == "x" and zero_radius_helper:
        zero_radius_helper.restore_straight()
    elif char == "," and zero_radius_helper:
        zero_radius_helper.request_step(-1)
    elif char == "." and zero_radius_helper:
        zero_radius_helper.request_step(+1)

    # --- Help ---
    elif char == "?":
        print_help(active_drive_nodes, active_steer_nodes)

    # --- Calibration select & files ---
    elif char.isdigit() and steer_controller:
        node_map = {1: 2, 2: 4, 3: 6, 4: 8}  # 1:FL, 2:FR, 3:RL, 4:RR
        node_id = node_map.get(int(char))
        if node_id and node_id in steer_controller.nodes:
            selected_steer_node = node_id
            with print_lock:
                print(f"\n[Calib] Steer node {selected_steer_node} selected for calibration.")

    elif char == "p" and steer_controller:
        steer_controller.print_offsets()

    elif char == "s" and steer_controller:
        steer_controller.save_offsets()   # 레거시: 현재 프로파일만 저장

    elif char == "l" and steer_controller:
        steer_controller.load_offsets()   # 레거시: 현재 프로파일로 로드

    # --- Profile manual select ---
    elif char == "f" and steer_controller:
        steer_controller.set_mode_auto(False)
        steer_controller.set_profile("forward", reason="manual key [f]")
    elif char == "o" and steer_controller:
        steer_controller.set_mode_auto(False)
        steer_controller.set_profile("reverse", reason="manual key [o]")
    elif char == "n" and steer_controller:
        steer_controller.set_mode_auto(False)
        steer_controller.set_profile("neutral", reason="manual key [n]")

    return True

# --- Status formatting -------------------------------------------------------
def format_drive_status(status: Dict[str, object]) -> str:
    if not status:
        return "Drive: (disabled)"
    speed = status["target_speed_mps"]
    accel = status["target_accel_mps2"]
    vel_parts = []
    for node, value in status["actual_velocity_mps"].items():
        vel_parts.append(f"{node}:{value:.2f}" if value is not None else f"{node}:N/A")
    pos_parts = []
    for node, value in status["actual_position_m"].items():
        pos_parts.append(f"{node}:{value:.2f}" if value is not None else f"{node}:N/A")
    return (
        f"Drive tgt {speed:6.2f} m/s accel {accel:5.2f} m/s² | "
        f"vel {{ {' '.join(vel_parts)} }} m/s | "
        f"pos {{ {' '.join(pos_parts)} }} m"
    )

def format_steer_status(status: Dict[str, object]) -> str:
    if not status:
        return "Steer: (disabled)"
    tgt = status["target_deg"]
    vel = status["profile_vel_dps"]
    angle_parts = []
    for node, value in status["actual_angle_deg"].items():
        angle_parts.append(f"{node}:{value:.1f}" if value is not None else f"{node}:N/A")

    calib_str = ""
    if selected_steer_node is not None:
        offset = status["offsets_deg"].get(selected_steer_node, 0.0)
        calib_str = f" | Calib Node:{selected_steer_node} Offset:{offset:+.3f} Step:{current_offset_step_deg:.3f}"

    zero_turn_str = ""
    if zero_radius_helper and zero_radius_helper.available:
        with zero_radius_helper.motion_lock:
            yaw = zero_radius_helper.cumulative_deg
        zero_turn_str = f" | ZeroTurn {yaw:+4.1f}°"

    prof = status.get("profile", "forward")
    mode = "Auto" if status.get("profile_mode_auto", True) else "Manual"
    prof_str = f" | Prof:{prof} [{mode}]"

    return (
        f"Steer tgt {tgt:6.2f}° vel {vel:5.1f}°/s | "
        f"angle {{ {' '.join(angle_parts)} }}°{calib_str}{zero_turn_str}{prof_str}"
    )

# --- Help text ---------------------------------------------------------------
def parse_node_argument(arg: str, config: Dict[int, str]) -> List[int]:
    arg = arg.strip().lower()
    if arg in ("all", ""):
        return sorted(config.keys())
    if arg in ("none", "off"):
        return []
    nodes = []
    for part in arg.split(","):
        value = int(part.strip())
        if value not in config:
            raise ValueError(f"Node {value} not in supported set {sorted(config.keys())}")
        nodes.append(value)
    return sorted(nodes)

def _build_key_guide(drive_nodes: List[int], steer_nodes: List[int]) -> str:
    drive_label = ", ".join(str(node) for node in drive_nodes) if drive_nodes else "none"
    steer_label = ", ".join(str(node) for node in steer_nodes) if steer_nodes else "none"
    accel_step_k = DRIVE_ACCEL_STEP_PPS2 / 1000
    lines: List[str] = []
    lines.append("\n" + "=" * 74)
    lines.append(" KEYBOARD QUICK GUIDE  (press [?] anytime) ".center(74))
    lines.append("=" * 74)
    lines.append(f"Active drive nodes : {drive_label}")
    lines.append(f"Active steer nodes : {steer_label}")
    lines.append("")
    lines.append("Left hand → Drive speed & accel")
    lines.append(
        f"  [u] +{accel_step_k:.0f}k pps²   [i] +{DRIVE_SPEED_STEP_MPS:.2f} m/s   [m] zero speed"
    )
    lines.append(
        f"  [j] -{accel_step_k:.0f}k pps²   [k] -{DRIVE_SPEED_STEP_MPS:.2f} m/s   [t] drive snapshot"
    )
    lines.append("")
    lines.append("Right hand → Steering angle presets")
    lines.append(
        f"  [q] steer -{STEER_DEG_STEP}°   [w] steer 0°   [e] steer +{STEER_DEG_STEP}°"
    )
    lines.append("  [a] steer -90°        [d] steer +90°")
    lines.append("")
    lines.append("Calibration & wheel selection")
    lines.append("  [1][2][3][4] select wheel → FL / FR / RL / RR")
    lines.append("  F1/F2/F3   offset step 0.001° / 0.010° / 0.100°")
    lines.append("  <- / ->    trim selected wheel by current step")
    lines.append("  p print offsets   s save offsets (legacy)   l load offsets (legacy)")
    lines.append("")
    lines.append("Profile mode & files")
    lines.append("  [f] FORWARD (Manual)   [o] REVERSE (Manual)   [n] NEUTRAL (Manual)")
    lines.append("  [F4] toggle Auto/Manual (Auto: +v=forward, -v=reverse, |v|<=eps=neutral)")
    lines.append("  [F5] save all profiles → steer_offset_profiles.json")
    lines.append("  [F6] load all profiles ← steer_offset_profiles.json")
    lines.append("")
    lines.append("Diagnostics")
    lines.append("  r steer snapshot -> 0x6063/0x6064 pulses + calibrated angle")
    lines.append("")
    lines.append("Wheel layout reference")
    lines.append("        Front ^")
    lines.append("      [1]   [2]")
    lines.append("      [3]   [4]")
    lines.append("        Rear")
    lines.append("")
    lines.append("Zero-radius rotation cluster")
    lines.append("  z align wheels    x restore straight")
    lines.append(
        f"  , rotate -{ZERO_TURN_STEP_ANGLE_DEG:.1f}° (CW)   . rotate +{ZERO_TURN_STEP_ANGLE_DEG:.1f}° (CCW)"
    )
    lines.append("")
    lines.append("General")
    lines.append("  ESC exit controller")
    lines.append("  [?] show this quick guide")
    lines.append("=" * 74)
    return "\n".join(lines)

def print_help(drive_nodes: List[int], steer_nodes: List[int]) -> None:
    padded_status_print("")
    guide = _build_key_guide(drive_nodes, steer_nodes)
    with print_lock:
        print(guide)

# --- Main loop ---------------------------------------------------------------
def main() -> None:
    global drive_controller, steer_controller, is_running, selected_steer_node, zero_radius_helper, active_drive_nodes, active_steer_nodes

    parser = argparse.ArgumentParser(
        description="Combined keyboard controller for drive and steer motors."
    )
    parser.add_argument("--drive", default="all", help="Drive nodes (e.g. all, 1,3,5)")
    parser.add_argument("--steer", default="all", help="Steer nodes (e.g. all, 2,4)")
    args = parser.parse_args()

    drive_nodes = parse_node_argument(args.drive, DRIVE_NODE_CONFIG)
    steer_nodes = parse_node_argument(args.steer, STEER_NODE_CONFIG)
    if not drive_nodes and not steer_nodes:
        print("No nodes selected. Use --drive or --steer to pick targets.")
        return

    bus_names = set(DRIVE_NODE_CONFIG[node] for node in drive_nodes)
    bus_names.update(STEER_NODE_CONFIG[node] for node in steer_nodes)
    ensure_buses(bus_names)

    active_drive_nodes = list(drive_nodes)
    active_steer_nodes = list(steer_nodes)

    if drive_nodes:
        drive_controller = DriveController(drive_nodes)
        drive_controller.setup()
    if steer_nodes:
        steer_controller = SteerController(steer_nodes)
        steer_controller.setup()
        if steer_nodes:
            selected_steer_node = steer_nodes[0]

    zero_radius_helper = ZeroRadiusHelper(
        drive_nodes, steer_nodes, drive_controller, steer_controller
    )
    if zero_radius_helper.available:
        with print_lock:
            print(
                "\n[ZeroTurn] Ready. Use [z] to align, [x] to restore, [,]/[.] for ±"
                f"{ZERO_TURN_STEP_ANGLE_DEG:.1f}° yaw steps."
            )
    else:
        if drive_nodes and steer_nodes:
            with print_lock:
                print(
                    "\n[ZeroTurn] Zero-radius helpers disabled (need drive 1,3,5,7 and steer 2,4,6,8)."
                )

    print_help(drive_nodes, steer_nodes)

    listener = keyboard.Listener(on_press=on_press)
    listener.start()

    try:
        while is_running:
            drive_status = drive_controller.fetch_status() if drive_controller else {}
            # ✨ 자동 프로파일 전환
            if drive_controller and steer_controller and steer_controller.profile_mode_auto:
                spd = drive_status.get("target_speed_mps", 0.0)
                desired = None
                if spd > AUTO_SWITCH_SPEED_EPS:
                    desired = "forward"
                elif spd < -AUTO_SWITCH_SPEED_EPS:
                    desired = "reverse"
                else:
                    desired = "neutral"
                if desired and desired != steer_controller.active_profile:
                    steer_controller.set_profile(desired, reason="auto by speed sign / near-stop")

            steer_status = steer_controller.fetch_status() if steer_controller else {}
            status_line = f"{format_drive_status(drive_status)} || {format_steer_status(steer_status)}"
            padded_status_print(status_line)
            time.sleep(0.5)
    except KeyboardInterrupt:
        is_running = False
    finally:
        listener.stop()
        padded_status_print("")
        with print_lock:
            print("\n\nShutting down controllers...")
        if drive_controller:
            drive_controller.emergency_stop()
        time.sleep(0.5)
        shutdown_buses()
        with print_lock:
            print("All CAN buses closed. Bye!")

if __name__ == "__main__":
    main()
