#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Keyboard-driven zero-radius rotation control for the 4-wheel AMR."""

import math
import struct
import threading
import time
from dataclasses import dataclass
from typing import Dict, Tuple

import can
from pynput import keyboard


@dataclass
class RobotConfig:
    wheel_base: float = 1.1211306
    track_front: float = 0.8415904
    track_rear: float = 0.6965428
    wheel_radius: float = 0.075  # metre
    steering_gear_ratio: float = 122.5
    driving_gear_ratio: float = 20.0
    steer_feed_constant: int = 18000  # pulses per motor rev (steer axis)
    drive_feed_constant: int = 10000  # pulses per motor rev (drive axis)
    drive_inverted_nodes: Tuple[int, ...] = (1, 5)  # compensate motor wiring
    step_angle_deg: float = 0.05
    step_duration: float = 0.4
    steer_settle_time: float = 2.0
    can_bitrate: int = 500000


CAN_CHANNELS = {"bus0": "can0", "bus1": "can1"}
OD_CONTROL_WORD = 0x6040
OD_MODES_OF_OPERATION = 0x6060
OD_TARGET_POSITION = 0x607A
OD_TARGET_VELOCITY = 0x60FF
OD_PROFILE_VELOCITY = 0x6081
OD_PROFILE_ACCELERATION = 0x6083
OD_PROFILE_DECELERATION = 0x6084

STEER_NODE_BY_WHEEL = {"FL": 2, "FR": 4, "RL": 6, "RR": 8}
DRIVE_NODE_BY_WHEEL = {"FL": 1, "FR": 3, "RL": 5, "RR": 7}
NODE_TO_BUS = {1: "bus1", 2: "bus1", 3: "bus0", 4: "bus0",
               5: "bus1", 6: "bus1", 7: "bus0", 8: "bus0"}


class ZeroRadiusController:
    def __init__(self, config: RobotConfig) -> None:
        self.config = config
        self.buses: Dict[str, can.interface.Bus] = {}
        self.step_in_progress = False
        self.motion_lock = threading.Lock()
        self.cumulative_deg = 0.0

        self.steer_angles_deg = self._calculate_zero_turn_angles()
        self.steer_target_pulses = self._angles_to_pulses(self.steer_angles_deg)
        self.drive_step_pulses, self.drive_velocity_pps = self._precompute_drive_profile()

    # ------------------------------------------------------------------
    # Kinematics helpers
    def _calculate_zero_turn_angles(self) -> Dict[str, float]:
        coords = {
            "FL": (self.config.wheel_base / 2.0, self.config.track_front / 2.0),
            "FR": (self.config.wheel_base / 2.0, -self.config.track_front / 2.0),
            "RL": (-self.config.wheel_base / 2.0, self.config.track_rear / 2.0),
            "RR": (-self.config.wheel_base / 2.0, -self.config.track_rear / 2.0),
        }
        angles: Dict[str, float] = {}
        for name, (x, y) in coords.items():
            radial = math.atan2(y, x)
            angle_deg = math.degrees(radial) + 90.0
            while angle_deg > 180.0:
                angle_deg -= 360.0
            while angle_deg <= -180.0:
                angle_deg += 360.0
            angles[name] = angle_deg
        return angles

    def _angles_to_pulses(self, angles_deg: Dict[str, float]) -> Dict[str, int]:
        pulses_per_deg = (self.config.steer_feed_constant * self.config.steering_gear_ratio) / 360.0
        return {
            wheel: int(round(angle_deg * pulses_per_deg))
            for wheel, angle_deg in angles_deg.items()
        }

    def _precompute_drive_profile(self) -> Tuple[Dict[str, int], Dict[str, int]]:
        theta = math.radians(self.config.step_angle_deg)
        drive_step: Dict[str, int] = {}
        drive_velocity: Dict[str, int] = {}
        for wheel, node_id in DRIVE_NODE_BY_WHEEL.items():
            if wheel.startswith("F"):
                lateral = self.config.track_front / 2.0
            else:
                lateral = self.config.track_rear / 2.0
            longitudinal = self.config.wheel_base / 2.0
            if wheel in ("RL", "RR"):
                longitudinal *= -1.0
            radius = math.hypot(longitudinal, lateral)
            arc_length = radius * theta
            wheel_revs = arc_length / (2.0 * math.pi * self.config.wheel_radius)
            motor_revs = wheel_revs * self.config.driving_gear_ratio
            pulses = motor_revs * self.config.drive_feed_constant
            drive_step[wheel] = int(round(pulses))
            velocity_pps = pulses / self.config.step_duration
            drive_velocity[wheel] = int(round(velocity_pps))
        return drive_step, drive_velocity

    # ------------------------------------------------------------------
    # CAN helpers
    def _open_buses(self) -> None:
        for name, channel in CAN_CHANNELS.items():
            self.buses[name] = can.interface.Bus(
                channel=channel,
                bustype="socketcan",
                bitrate=self.config.can_bitrate,
            )

    def _shutdown_buses(self) -> None:
        for bus in self.buses.values():
            try:
                bus.shutdown()
            except can.CanError:
                pass
        self.buses.clear()

    def _bus_for_node(self, node_id: int) -> can.interface.Bus:
        bus_name = NODE_TO_BUS[node_id]
        return self.buses[bus_name]

    def _send_nmt(self, command: int, target_bus: str) -> None:
        msg = can.Message(arbitration_id=0x000, data=[command, 0x00], is_extended_id=False)
        self.buses[target_bus].send(msg)
        time.sleep(0.01)

    def _send_sdo(self, node_id: int, index: int, sub_index: int, value: int, size: int) -> None:
        cob_id = 0x600 + node_id
        if size == 1:
            command, packed = 0x2F, struct.pack("<b", value)
        elif size == 2:
            command, packed = 0x2B, struct.pack("<h", value)
        elif size == 4:
            command, packed = 0x23, struct.pack("<i", value)
        else:
            raise ValueError("Unsupported SDO size (1, 2, 4).")
        index_bytes = struct.pack("<H", index)
        payload = bytes([command, index_bytes[0], index_bytes[1], sub_index]) + packed
        payload = payload.ljust(8, b"\x00")
        msg = can.Message(arbitration_id=cob_id, data=payload, is_extended_id=False)
        self._bus_for_node(node_id).send(msg)
        time.sleep(0.01)

    # ------------------------------------------------------------------
    # Initialisation sequence
    def initialise(self) -> None:
        self._open_buses()
        print("CAN buses opened.")
        for bus_name in CAN_CHANNELS:
            self._send_nmt(0x81, bus_name)  # reset
        time.sleep(1.0)
        for bus_name in CAN_CHANNELS:
            self._send_nmt(0x01, bus_name)  # start
        print("NMT state: OPERATIONAL.")

        for wheel, node_id in STEER_NODE_BY_WHEEL.items():
            self._send_sdo(node_id, OD_MODES_OF_OPERATION, 0, 1, 1)
            self._send_sdo(node_id, OD_PROFILE_VELOCITY, 0, 150000, 4)
            self._send_sdo(node_id, OD_PROFILE_ACCELERATION, 0, 150000, 4)
            self._send_sdo(node_id, OD_PROFILE_DECELERATION, 0, 150000, 4)
            print(f"Steer node {wheel} -> mode=PP")
        for wheel, node_id in DRIVE_NODE_BY_WHEEL.items():
            self._send_sdo(node_id, OD_MODES_OF_OPERATION, 0, 3, 1)
            self._send_sdo(node_id, OD_PROFILE_ACCELERATION, 0, 80000, 4)
            self._send_sdo(node_id, OD_PROFILE_DECELERATION, 0, 80000, 4)
            print(f"Drive node {wheel} -> mode=PV")

        for node_id in list(STEER_NODE_BY_WHEEL.values()) + list(DRIVE_NODE_BY_WHEEL.values()):
            for state in (0x06, 0x07, 0x0F):
                self._send_sdo(node_id, OD_CONTROL_WORD, 0, state, 2)
        print("All servos enabled.")

        print("Target steering angles (deg / pulses):")
        for wheel in ("FL", "FR", "RL", "RR"):
            angle = self.steer_angles_deg[wheel]
            pulses = self.steer_target_pulses[wheel]
            node_id = STEER_NODE_BY_WHEEL[wheel]
            self._send_sdo(node_id, OD_TARGET_POSITION, 0, pulses, 4)
            self._send_sdo(node_id, OD_CONTROL_WORD, 0, 0x1F, 2)  # start move
            print(f"  {wheel}: {angle:8.3f} deg -> {pulses:8d} pulses")
        print(f"Waiting {self.config.steer_settle_time:.1f}s for steering to settle...")
        time.sleep(self.config.steer_settle_time)
        print("Steering modules aligned for zero-radius rotation.")

        print("Drive step (pulses / pulses/s for 1 deg):")
        for wheel in ("FL", "FR", "RL", "RR"):
            step = self.drive_step_pulses[wheel]
            vel = self.drive_velocity_pps[wheel]
            print(f"  {wheel}: {step:6d} pulses, {vel:6d} pulses/s")

    # ------------------------------------------------------------------
    # Motion commands
    def _apply_drive_velocities(self, direction: int) -> None:
        for wheel, node_id in DRIVE_NODE_BY_WHEEL.items():
            base_velocity = self.drive_velocity_pps[wheel]
            if node_id in self.config.drive_inverted_nodes:
                base_velocity *= -1
            command_velocity = int(direction * base_velocity)
            self._send_sdo(node_id, OD_TARGET_VELOCITY, 0, command_velocity, 4)

    def _stop_drives(self) -> None:
        for node_id in DRIVE_NODE_BY_WHEEL.values():
            self._send_sdo(node_id, OD_TARGET_VELOCITY, 0, 0, 4)

    def request_step(self, direction: int) -> None:
        with self.motion_lock:
            if self.step_in_progress:
                print("Busy: previous step still running.")
                return
            self.step_in_progress = True
        threading.Thread(target=self._run_step, args=(direction,), daemon=True).start()

    def _run_step(self, direction: int) -> None:
        try:
            self._apply_drive_velocities(direction)
            time.sleep(self.config.step_duration)
        finally:
            self._stop_drives()
            with self.motion_lock:
                self.step_in_progress = False
        self.cumulative_deg += direction * self.config.step_angle_deg
        print(f"Total yaw: {self.cumulative_deg:+7.2f} deg", end="\r")

    # ------------------------------------------------------------------
    # Keyboard integration
    def handle_key(self, key) -> None:
        try:
            if key.char == 'e':
                self.request_step(+1)
            elif key.char == 'q':
                self.request_step(-1)
            elif key.char == 'x':
                print("\nExit requested (x).")
                return False
        except AttributeError:
            if key == keyboard.Key.esc:
                print("\nExit requested (ESC).")
                return False
        return None

    # ------------------------------------------------------------------
    def shutdown(self) -> None:
        print("\nStopping drives and disabling servos...")
        self._stop_drives()
        for node_id in list(STEER_NODE_BY_WHEEL.values()) + list(DRIVE_NODE_BY_WHEEL.values()):
            try:
                self._send_sdo(node_id, OD_CONTROL_WORD, 0, 0x06, 2)
            except can.CanError:
                pass
        self._shutdown_buses()
        print("Shutdown complete.")


def main() -> None:
    controller = ZeroRadiusController(RobotConfig())
    try:
        controller.initialise()
        print("\nControls: [q] CW -1 deg, [e] CCW +1 deg, [x]/[ESC] exit")
        with keyboard.Listener(on_press=controller.handle_key) as listener:
            listener.join()
    except KeyboardInterrupt:
        print("\nInterrupted by user.")
    finally:
        controller.shutdown()


if __name__ == "__main__":
    main()
