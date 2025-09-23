#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import can
import time
import argparse

# --- 전체 조향 노드 설정 ---
ALL_STEER_NODES_CONFIG = {
    2: {'bus': 'can1', 'id': 2},
    6: {'bus': 'can1', 'id': 6},
    4: {'bus': 'can0', 'id': 4},
    8: {'bus': 'can0', 'id': 8},
}

# --- CANopen 명령어 페이로드 정의 ---
CMD_SET_HOMING_METHOD_35 = [0x2F, 0x98, 0x60, 0x00, 0x23, 0x00, 0x00, 0x00]
CMD_SERVO_ON = [0x2B, 0x40, 0x60, 0x00, 0x0F, 0x00, 0x00, 0x00]
CMD_SET_HOMING_MODE = [0x2F, 0x60, 0x60, 0x00, 0x06, 0x00, 0x00, 0x00]
CMD_START_HOMING = [0x2B, 0x40, 0x60, 0x00, 0x1F, 0x00, 0x00, 0x00]

# ✨ NMT 명령어 정보 추가
NMT_COB_ID = 0x000
CMD_ENTER_PRE_OP = 0x80


def send_sdo_command(bus, node_id, description, data_payload):
    """지정된 노드에 SDO 메시지를 전송하고 로그를 출력하는 함수"""
    print(description)
    cob_id = 0x600 + node_id
    msg = can.Message(arbitration_id=cob_id, data=data_payload, is_extended_id=False)
    try:
        bus.send(msg)
        print(f"  -> OK: Node {node_id} on {bus.channel_info}")
    except can.CanError as e:
        print(f"  -> FAILED to send to Node {node_id}: {e}")
    time.sleep(0.1)

# ✨ NMT 명령어를 보내는 전용 함수 추가
def send_nmt_command(bus, command, node_id):
    """지정된 노드에 NMT 명령을 전송하는 함수"""
    data_payload = [command, node_id]
    msg = can.Message(arbitration_id=NMT_COB_ID, data=data_payload, is_extended_id=False)
    try:
        bus.send(msg)
        print(f"  -> OK: Sent NMT command 0x{command:02X} to Node {node_id}")
    except can.CanError as e:
        print(f"  -> FAILED to send NMT command to Node {node_id}: {e}")
    time.sleep(0.05) # 상태 변경을 위한 짧은 대기


def main():
    """메인 실행 함수"""
    parser = argparse.ArgumentParser(description="CANopen Homing script for specified steer motors.")
    parser.add_argument('nodes', nargs='+', help="Node IDs to home (e.g., 2 4 8). Use 'all' to home all configured nodes.")
    args = parser.parse_args()

    nodes_to_home = []
    if 'all' in [node.lower() for node in args.nodes]:
        nodes_to_home = list(ALL_STEER_NODES_CONFIG.values())
        print(f"Homing Target: ALL nodes -> {[node['id'] for node in nodes_to_home]}")
    else:
        try:
            target_ids = [int(node_id) for node_id in args.nodes]
            valid_ids = []
            for node_id in target_ids:
                if node_id not in ALL_STEER_NODES_CONFIG:
                    print(f"Warning: Node ID {node_id} is not a valid steer motor. Skipping.")
                else:
                    nodes_to_home.append(ALL_STEER_NODES_CONFIG[node_id])
                    valid_ids.append(node_id)
            if not valid_ids:
                print("No valid nodes selected. Exiting.")
                return
            print(f"Homing Targets: Nodes -> {valid_ids}")
        except ValueError:
            print("Error: Invalid node ID provided. Please provide integer IDs or 'all'.")
            return
            
    buses = {}
    try:
        bus_names = set(node['bus'] for node in nodes_to_home)
        print("\nInitializing CAN buses...")
        for name in bus_names:
            buses[name] = can.interface.Bus(channel=name, interface="socketcan")
            print(f"- Bus '{name}' is up.")

        # ✨ 버스 초기화 후 Pre-Operational 모드 진입 명령 전송
        print("\nSetting all target nodes to Pre-Operational Mode...")
        for node_info in nodes_to_home:
            bus_name = node_info['bus']
            node_id = node_info['id']
            bus = buses[bus_name]
            send_nmt_command(bus, CMD_ENTER_PRE_OP, node_id)

        print("\nStarting Homing Sequence...")
        for node_info in nodes_to_home:
            bus_name = node_info['bus']
            node_id = node_info['id']
            bus = buses[bus_name]

            print(f"\n--- Homing Node {node_id} on {bus_name} ---")
            
            send_sdo_command(bus, node_id, "1. Setting Homing Method to 35...", CMD_SET_HOMING_METHOD_35)
            send_sdo_command(bus, node_id, "2. Setting Servo-ON (Operation Enable)...", CMD_SERVO_ON)
            send_sdo_command(bus, node_id, "3. Switching to Homing Mode...", CMD_SET_HOMING_MODE)
            send_sdo_command(bus, node_id, "4. Starting Homing operation...", CMD_START_HOMING)

        print("\n--- Homing sequence sent to all selected nodes successfully. ---")

    except Exception as e:
        print(f"\nAn error occurred: {e}")
    finally:
        for name, bus in buses.items():
            if bus:
                bus.shutdown()
                print(f"\nBus '{name}' shut down.")

if __name__ == "__main__":
    main()