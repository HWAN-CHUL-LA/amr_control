#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
모든 조향 모터를 지정된 절대 각도로 동시에 회전시키는 스크립트.
"""

import can
import struct
import time
import sys

# --- AMR 모터 및 제어 상수 설정 ---
TOTAL_REDUCTION_RATIO = 122.5
USER_PULSES_PER_REVOLUTION = 18000.0

# 1도당 필요한 펄스 값 자동 계산
PULSE_PER_DEGREE = (USER_PULSES_PER_REVOLUTION * TOTAL_REDUCTION_RATIO) / 360.0

# CANopen 객체 주소 (Object Dictionary)
OD_CONTROL_WORD = 0x6040
OD_MODES_OF_OPERATION = 0x6060
OD_TARGET_POSITION = 0x607A

# 제어할 조향 모터 목록 (버스, 노드 ID)
STEER_MOTORS = [
    {'bus': 'can1', 'id': 2},  # Front-Left
    {'bus': 'can0', 'id': 4},  # Front-Right
    {'bus': 'can1', 'id': 6},  # Rear-Left
    {'bus': 'can0', 'id': 8}   # Rear-Right
]

def send_sdo_write(bus: can.interface.Bus, node_id: int, index: int, sub_index: int, value: int, size: int):
    """지정된 크기의 데이터로 SDO Write 명령을 생성하고 전송하는 범용 함수"""
    cob_id = 0x600 + node_id

    if size == 1:
        cmd, fmt = 0x2F, '<b'
    elif size == 2:
        cmd, fmt = 0x2B, '<h'
    elif size == 4:
        cmd, fmt = 0x23, '<i'
    else:
        raise ValueError("Unsupported data size. Must be 1, 2, or 4.")

    index_bytes = struct.pack('<H', index)
    data_bytes = struct.pack(fmt, value)
    
    payload = [cmd, index_bytes[0], index_bytes[1], sub_index] + list(data_bytes)
    payload.extend([0] * (8 - len(payload)))

    msg = can.Message(arbitration_id=cob_id, data=payload, is_extended_id=False)
    
    try:
        bus.send(msg)
    except can.CanError as e:
        print(f"Message to Node {node_id} NOT sent: {e}")

def main():
    if len(sys.argv) != 2:
        print("사용법: python3 rotate_all_steer.py [DEGREES]")
        print("예시: python3 rotate_all_steer.py 90.5")
        return

    target_degrees = float(sys.argv[1])
    target_pulses = int(target_degrees * PULSE_PER_DEGREE)

    buses = {}
    try:
        # 필요한 모든 CAN 버스 초기화
        print("\nInitializing CAN buses...")
        bus_names = set(motor['bus'] for motor in STEER_MOTORS)
        for name in bus_names:
            buses[name] = can.interface.Bus(channel=name, bustype='socketcan')
            print(f" -> CAN bus on '{name}' initialized.")

        print(f"\nTarget: {target_degrees} degrees -> {target_pulses} pulses")
        
        # --- 모든 모터 동시 제어 시작 ---

        print("\nStep 1: Setting all motors to Profile Position Mode...")
        for motor in STEER_MOTORS:
            send_sdo_write(buses[motor['bus']], motor['id'], OD_MODES_OF_OPERATION, 0, 1, 1)
        time.sleep(0.1)

        print("\nStep 2: Enabling all motors (Servo ON)...")
        control_words = {'ready': 0x06, 'on': 0x07, 'enable': 0x0F}
        for state in ['ready', 'on', 'enable']:
            print(f" -> Sending '{state}' command to all motors...")
            for motor in STEER_MOTORS:
                send_sdo_write(buses[motor['bus']], motor['id'], OD_CONTROL_WORD, 0, control_words[state], 2)
            time.sleep(0.1)

        print("\nStep 3: Sending target position to all motors...")
        for motor in STEER_MOTORS:
            send_sdo_write(buses[motor['bus']], motor['id'], OD_TARGET_POSITION, 0, target_pulses, 4)
        time.sleep(0.1)

        print("\nStep 4: Triggering motion on all motors...")
        # New Set-Point (bit 4) 활성화
        for motor in STEER_MOTORS:
            send_sdo_write(buses[motor['bus']], motor['id'], OD_CONTROL_WORD, 0, 0x1F, 2)
        time.sleep(0.05)
        # New Set-Point 비활성화 (다음 명령을 위해)
        for motor in STEER_MOTORS:
            send_sdo_write(buses[motor['bus']], motor['id'], OD_CONTROL_WORD, 0, 0x0F, 2)
        
        print("\nAll motion commands sent successfully!")

    except Exception as e:
        print(f"\nAn error occurred: {e}")
    finally:
        for name, bus in buses.items():
            if bus:
                bus.shutdown()
                print(f"\nCAN bus on '{name}' shut down.")

if __name__ == "__main__":
    main()