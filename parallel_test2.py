#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import logging
import sys
import threading
import time
from typing import Dict, List, Optional

from pynput import keyboard

from control_lib.controllers import (
    DRIVE_ACCEL_STEP_PPS2,
    DRIVE_NODE_CONFIG,
    DRIVE_SPEED_STEP_MPS,
    STEER_ACCEL_STEP_DPS2,
    STEER_DEG_STEP,
    STEER_NODE_CONFIG,
    STEER_OFFSET_STEP_DEG,
    STEER_VEL_STEP_DPS,
)
from control_lib.service import AmrControlService, ControllerConfig


logging.basicConfig(level=logging.INFO)

print_lock = threading.Lock()
status_len_lock = threading.Lock()
last_status_len = 0
is_running = True


def padded_status_print(text: str) -> None:
    global last_status_len
    with status_len_lock:
        pad = " " * max(0, last_status_len - len(text))
        msg = "\r" + text + pad
        with print_lock:
            sys.stdout.write(msg)
            sys.stdout.flush()
        last_status_len = len(text)


def controller_status_callback(message: str) -> None:
    """Thread-safe print hook for controller log messages."""
    padded_status_print("")
    with print_lock:
        print("\n" + message)
        sys.stdout.flush()

# --- Keyboard handling (✨ 보정 키 추가) ---
control_service: Optional[AmrControlService] = None
# ✨ 보정을 위해 현재 선택된 조향 노드를 저장하는 변수
selected_steer_node: Optional[int] = None


def on_press(key: keyboard.Key) -> bool:
    global is_running, selected_steer_node, control_service
    service = control_service
    try:
        char = key.char.lower()
    except AttributeError:
        if key == keyboard.Key.esc:
            is_running = False
            return False
        if service and service.has_steer() and selected_steer_node:
            if key == keyboard.Key.left:
                service.adjust_steer_offset(selected_steer_node, -STEER_OFFSET_STEP_DEG)
            elif key == keyboard.Key.right:
                service.adjust_steer_offset(selected_steer_node, STEER_OFFSET_STEP_DEG)
        return True

    if not service:
        return True

    if char == "i" and service.has_drive():
        service.adjust_drive_speed(DRIVE_SPEED_STEP_MPS)
    elif char == "k" and service.has_drive():
        service.adjust_drive_speed(-DRIVE_SPEED_STEP_MPS)
    elif char == "m" and service.has_drive():
        service.set_drive_speed(0.0)
    elif char == "u" and service.has_drive():
        service.adjust_drive_accel(DRIVE_ACCEL_STEP_PPS2)
    elif char == "j" and service.has_drive():
        service.adjust_drive_accel(-DRIVE_ACCEL_STEP_PPS2)
    elif char == "t" and service.has_drive():
        service.print_drive_actuals()
    elif char == "q" and service.has_steer():
        service.adjust_steer_angle(STEER_DEG_STEP)
    elif char == "e" and service.has_steer():
        service.adjust_steer_angle(-STEER_DEG_STEP)
    elif char == "a" and service.has_steer():
        service.adjust_steer_angle(90.0)
    elif char == "d" and service.has_steer():
        service.adjust_steer_angle(-90.0)
    elif char == "w" and service.has_steer():
        service.set_steer_angle(0.0)
    elif char == "v" and service.has_steer():
        service.adjust_steer_velocity(-STEER_VEL_STEP_DPS)
    elif char == "b" and service.has_steer():
        service.adjust_steer_velocity(STEER_VEL_STEP_DPS)
    elif char == "g" and service.has_steer():
        service.adjust_steer_accel(-STEER_ACCEL_STEP_DPS2)
    elif char == "h" and service.has_steer():
        service.adjust_steer_accel(STEER_ACCEL_STEP_DPS2)
    elif char == "r" and service.has_steer():
        service.print_steer_actuals()
    elif char.isdigit() and service.has_steer():
        node_map = {1: 2, 2: 4, 3: 6, 4: 8}
        node_id = node_map.get(int(char))
        if node_id and node_id in service.steer_nodes():
            selected_steer_node = node_id
            controller_status_callback(
                f"[Calib] Steer node {selected_steer_node} selected for calibration."
            )
    elif char == "p" and service.has_steer():
        service.print_steer_offsets()
    elif char == "s" and service.has_steer():
        service.save_offsets()
    elif char == "l" and service.has_steer():
        service.load_offsets()
    return True




# --- Status formatting (✨ 보정 상태 표시 추가) ---
def format_drive_status(status: Dict[str, object]) -> str:
    # ... (변경 없음)
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
    
    # ✨ 보정 상태를 위한 문자열 생성
    calib_str = ""
    if selected_steer_node is not None:
        offset = status["offsets_deg"].get(selected_steer_node, 0.0)
        calib_str = f" | Calib Node:{selected_steer_node} Offset:{offset:+.3f}"

    return (
        f"Steer tgt {tgt:6.2f}° vel {vel:5.1f}°/s | "
        f"angle {{ {' '.join(angle_parts)} }}°{calib_str}"
    )

# --- Main (✨ 보정 관련 초기화 추가) ---
def parse_node_argument(arg: str, config: Dict[int, str]) -> List[int]:
    # ... (변경 없음)
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
        print("  [q] angle -1°         [e] angle +1°         [w] back to 0°")
        print("  [v] velocity -5°/s    [b] velocity +5°/s")
        print("  [g] accel -20°/s²     [h] accel +20°/s²")
        print("  [r] print actual angle")
        # ✨ 보정 관련 키 설명 추가
        print("\nCalibration keys:")
        print("  [1-4] Select wheel (1:FL, 2:FR, 3:RL, 4:RR)")
        print("  [<-] selected offset -0.05°   [->] selected offset +0.05°")
        print("  [p] print all offsets")
        print("  [s] save offsets to file     [l] load offsets from file")
        
        print("\n[Esc] exits while keeping the last commands.")
        print("=" * 80)


def main() -> None:
    global control_service, is_running, selected_steer_node

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

    config = ControllerConfig(drive_nodes=drive_nodes, steer_nodes=steer_nodes)
    control_service = AmrControlService(config, status_hook=controller_status_callback)
    control_service.start()

    steer_nodes_active = control_service.steer_nodes()
    if steer_nodes_active:
        selected_steer_node = steer_nodes_active[0]

    print_help(drive_nodes, steer_nodes)

    listener = keyboard.Listener(on_press=on_press)
    listener.start()

    try:
        while is_running:
            drive_status = control_service.drive_status() if control_service else {}
            steer_status = control_service.steer_status() if control_service else {}
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
        if control_service:
            control_service.shutdown()
            time.sleep(0.5)
            control_service = None
        with print_lock:
            print("All CAN buses closed. Bye!")





if __name__ == "__main__":
    main()