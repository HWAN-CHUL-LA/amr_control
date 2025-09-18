#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import math
import struct
import sys
import threading
import time
from typing import Dict, Iterable, List, Optional

import can
from pynput import keyboard

# --- Drive constants ---------------------------------------------------------
WHEEL_RADIUS_M = 0.075
WHEEL_CIRCUMFERENCE_M = 2.0 * math.pi * WHEEL_RADIUS_M
DRIVE_REDUCTION_RATIO = 20.0
FEED_CONSTANT = 10000.0

DRIVE_NODE_CONFIG: Dict[int, str] = {
    1: "can1",
    3: "can0",
    5: "can1",
    7: "can0",
}

OD_CONTROL_WORD = 0x6040
OD_MODES_OF_OPERATION = 0x6060
OD_TARGET_VELOCITY = 0x60FF
OD_TARGET_POSITION = 0x607A
OD_PROFILE_VELOCITY = 0x6081
OD_PROFILE_ACCELERATION = 0x6083
OD_PROFILE_DECELERATION = 0x6084
OD_ACTUAL_POSITION = 0x6064
OD_ACTUAL_VELOCITY = 0x606C

DRIVE_SPEED_STEP_MPS = 0.05
DRIVE_ACCEL_STEP_PPS2 = 10000
DRIVE_MIN_ACCEL_PPS2 = 1000
DRIVE_READ_TIMEOUT = 0.5

# --- Steer constants ---------------------------------------------------------
TOTAL_REDUCTION_RATIO = 122.5
USER_PULSES_PER_REVOLUTION = 18000.0
PULSE_PER_DEGREE = (USER_PULSES_PER_REVOLUTION * TOTAL_REDUCTION_RATIO) / 360.0

STEER_NODE_CONFIG: Dict[int, str] = {
    2: "can1",
    4: "can0",
    6: "can1",
    8: "can0",
}

STEER_DEG_STEP = 1.0
STEER_VEL_STEP_DPS = 5.0
STEER_ACCEL_STEP_DPS2 = 20.0
STEER_MIN_VEL_DPS = 1.0
STEER_MIN_ACCEL_DPS2 = 5.0
STEER_READ_TIMEOUT = 0.5

# --- Shared CAN infrastructure -----------------------------------------------
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
        cmd, fmt = 0x2F, "<b"
    elif size == 2:
        cmd, fmt = 0x2B, "<H"
    elif size == 4:
        cmd, fmt = 0x23, "<i"
    else:
        raise ValueError("Unsupported SDO size (must be 1/2/4).")

    cob_id = 0x600 + node_id
    index_bytes = struct.pack("<H", index)
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

# --- Drive helpers -----------------------------------------------------------
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

# --- Steer helpers -----------------------------------------------------------
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

# --- Steer controller --------------------------------------------------------
class SteerController:
    def __init__(self, nodes: List[int]):
        self.nodes = nodes
        self.node_to_bus = {node: STEER_NODE_CONFIG[node] for node in nodes}
        self.current_target_deg = 0.0
        self.profile_velocity_dps = 30.0
        self.profile_accel_dps2 = 80.0
        self.state_lock = threading.Lock()

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
        pulses = deg_to_pulses(self.current_target_deg)
        for node, bus_name in self._for_each_node():
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
        with print_lock:
            print(f"\n[Steer] Target angle set to {target:.2f} deg")

    def set_position(self, value_deg: float) -> None:
        with self.state_lock:
            self.current_target_deg = value_deg
        self._push_position()
        self._trigger_motion()
        with print_lock:
            print(f"\n[Steer] Target angle forced to {value_deg:.2f} deg")

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

# --- Keyboard handling -------------------------------------------------------
drive_controller: Optional[DriveController] = None
steer_controller: Optional[SteerController] = None

def on_press(key: keyboard.Key) -> bool:
    global is_running
    try:
        char = key.char.lower()
    except AttributeError:
        if key == keyboard.Key.esc:
            is_running = False
            return False
        return True

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
    elif char == "q" and steer_controller:
        steer_controller.adjust_position(STEER_DEG_STEP)
    elif char == "e" and steer_controller:
        steer_controller.adjust_position(-STEER_DEG_STEP)
    elif char == "w" and steer_controller:
        steer_controller.set_position(0.0)
    elif char == "v" and steer_controller:
        steer_controller.adjust_velocity(-STEER_VEL_STEP_DPS)
    elif char == "b" and steer_controller:
        steer_controller.adjust_velocity(STEER_VEL_STEP_DPS)
    elif char == "g" and steer_controller:
        steer_controller.adjust_accel(-STEER_ACCEL_STEP_DPS2)
    elif char == "h" and steer_controller:
        steer_controller.adjust_accel(STEER_ACCEL_STEP_DPS2)
    elif char == "f" and steer_controller:
        steer_controller.print_profile_accel()
    elif char == "c" and steer_controller:
        steer_controller.print_profile_velocity()
    elif char == "r" and steer_controller:
        steer_controller.print_actual_position()
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
    accel = status["profile_accel_dps2"]
    angle_parts = []
    for node, value in status["actual_angle_deg"].items():
        angle_parts.append(f"{node}:{value:.1f}" if value is not None else f"{node}:N/A")
    vel_parts = []
    for node, value in status["actual_velocity_dps"].items():
        vel_parts.append(f"{node}:{value:.1f}" if value is not None else f"{node}:N/A")
    return (
        f"Steer tgt {tgt:6.2f}° vel {vel:5.1f}°/s accel {accel:5.1f}°/s² | "
        f"angle {{ {' '.join(angle_parts)} }}° | "
        f"vel {{ {' '.join(vel_parts)} }}°/s"
    )

# --- Main --------------------------------------------------------------------
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

def print_help(drive_nodes: List[int], steer_nodes: List[int]) -> None:
    with print_lock:
        print("\n" + "=" * 80)
        print("Keyboard control ready.")
        if drive_nodes:
            print(f"Drive nodes: {drive_nodes}")
        if steer_nodes:
            print(f"Steer nodes: {steer_nodes}")
        print("\nDrive keys:")
        print("  [i] speed +0.05 m/s   [k] speed -0.05 m/s   [m] zero speed")
        print("  [u] accel +10k pps²   [j] accel -10k pps²   [t] print drive actuals")
        print("\nSteer keys:")
        print("  [q] -1°   [e] +1°   [w] back to 0°")
        print("  [v] velocity -5°/s   [b] velocity +5°/s")
        print("  [g] accel -20°/s²    [h] accel +20°/s²")
        print("  [f] print profile accel   [c] print profile velocity   [r] print actual angle")
        print("\n[Esc] exits while keeping the last commands.")
        print("=" * 80)

def main() -> None:
    global drive_controller, steer_controller, is_running

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

    if drive_nodes:
        drive_controller = DriveController(drive_nodes)
        drive_controller.setup()
    if steer_nodes:
        steer_controller = SteerController(steer_nodes)
        steer_controller.setup()

    print_help(drive_nodes, steer_nodes)

    listener = keyboard.Listener(on_press=on_press)
    listener.start()

    try:
        while is_running:
            drive_status = drive_controller.fetch_status() if drive_controller else {}
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
