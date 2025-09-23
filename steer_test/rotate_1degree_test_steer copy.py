#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import can
import struct
import time
import sys
from pynput import keyboard

# --- 상수 설정 ---
TOTAL_REDUCTION_RATIO = 122.5
USER_PULSES_PER_REVOLUTION = 18000.0
PULSE_PER_DEGREE = (USER_PULSES_PER_REVOLUTION * TOTAL_REDUCTION_RATIO) / 360.0

# CANopen 객체 주소
OD_CONTROL_WORD = 0x6040
OD_MODES_OF_OPERATION = 0x6060
OD_TARGET_POSITION = 0x607A
OD_PROFILE_VELOCITY = 0x6081
OD_PROFILE_ACCELERATION = 0x6083
OD_PROFILE_DECELERATION = 0x6084

STEER_MOTORS = [
    {'bus': 'can1', 'id': 2}, {'bus': 'can0', 'id': 4},
    {'bus': 'can1', 'id': 6}, {'bus': 'can0', 'id': 8}
]

# --- 실시간 제어를 위한 전역 변수 ---
buses = {}
target_nodes = []
is_running = True
# 제어 상태 변수
current_target_degrees = 0.0
current_profile_velocity = 150000  # 중간값에서 시작
current_profile_accel = 150000     # 중간값에서 시작

# 키 입력당 변경 단위
VELOCITY_UNIT = 25000
ACCEL_UNIT = 25000
DEGREE_UNIT = 1.0

# --- CAN 통신 함수 ---
def send_sdo_write(bus, node_id, index, sub_index, value, size):
    cob_id = 0x600 + node_id
    if size == 1: cmd, fmt = 0x2F, '<b'
    elif size == 2: cmd, fmt = 0x2B, '<h'
    elif size == 4: cmd, fmt = 0x23, '<i'
    else: raise ValueError("Unsupported data size.")
    
    data_bytes = struct.pack(fmt, value)
    payload = [cmd, index & 0xFF, index >> 8, sub_index] + list(data_bytes)
    payload.extend([0] * (8 - len(payload)))
    msg = can.Message(arbitration_id=cob_id, data=payload, is_extended_id=False)
    bus.send(msg)

def read_sdo_value(bus, node_id, index, sub_index):
    """SDO 읽기 요청을 보내고 응답을 받아 값을 반환하는 함수"""
    cob_id_req = 0x600 + node_id
    cob_id_resp = 0x580 + node_id
    
    # 읽기 요청 전송
    payload = [0x40, index & 0xFF, index >> 8, sub_index, 0, 0, 0, 0]
    msg = can.Message(arbitration_id=cob_id_req, data=payload, is_extended_id=False)
    bus.send(msg)

    # 응답 기다리기
    timeout = time.time() + 0.5 # 0.5초 타임아웃
    while time.time() < timeout:
        response = bus.recv(0.1)
        if response and response.arbitration_id == cob_id_resp:
            # 응답 데이터 파싱 (4바이트 정수 값으로 가정)
            value = struct.unpack('<i', response.data[4:8])[0]
            return value
    return None # 타임아웃

# --- 제어 로직 함수 ---
def command_all_nodes(command_type, value):
    for node_id in target_nodes:
        bus = buses[NODE_CONFIG[node_id]]
        if command_type == 'position':
            target_pulses = int(value * PULSE_PER_DEGREE)
            send_sdo_write(bus, node_id, OD_TARGET_POSITION, 0, target_pulses, 4)
        elif command_type == 'velocity':
            send_sdo_write(bus, node_id, OD_PROFILE_VELOCITY, 0, int(value), 4)
        elif command_type == 'accel':
            send_sdo_write(bus, node_id, OD_PROFILE_ACCELERATION, 0, int(value), 4)
            send_sdo_write(bus, node_id, OD_PROFILE_DECELERATION, 0, int(value), 4)

def trigger_motion():
    """모든 노드의 위치 이동을 시작시키는 함수"""
    for node_id in target_nodes:
        bus = buses[NODE_CONFIG[node_id]]
        send_sdo_write(bus, node_id, OD_CONTROL_WORD, 0, 0x1F, 2)
    time.sleep(0.05)
    for node_id in target_nodes:
        bus = buses[NODE_CONFIG[node_id]]
        send_sdo_write(bus, node_id, OD_CONTROL_WORD, 0, 0x0F, 2)

# --- 키보드 입력 처리 ---
def on_press(key):
    global current_target_degrees, current_profile_velocity, current_profile_accel, is_running

    try:
        char = key.char
        # 위치 제어
        if char == 'q':
            current_target_degrees -= DEGREE_UNIT
            command_all_nodes('position', current_target_degrees)
            trigger_motion()
        elif char == 'e':
            current_target_degrees += DEGREE_UNIT
            command_all_nodes('position', current_target_degrees)
            trigger_motion()
        elif char == 'w':
            current_target_degrees = 0.0
            command_all_nodes('position', current_target_degrees)
            trigger_motion()
        # 가감속 제어
        elif char == 'h':
            current_profile_accel += ACCEL_UNIT
            command_all_nodes('accel', current_profile_accel)
        elif char == 'g':
            current_profile_accel = max(1000, current_profile_accel - ACCEL_UNIT)
            command_all_nodes('accel', current_profile_accel)
        # 이동 속도 제어
        elif char == 'b':
            current_profile_velocity += VELOCITY_UNIT
            command_all_nodes('velocity', current_profile_velocity)
        elif char == 'v':
            current_profile_velocity = max(1000, current_profile_velocity - VELOCITY_UNIT)
            command_all_nodes('velocity', current_profile_velocity)
        # 값 읽기
        elif char == 'f':
            print("\n[가감속도 읽기]")
            for node_id in target_nodes:
                bus = buses[NODE_CONFIG[node_id]]
                val = read_sdo_value(bus, node_id, OD_PROFILE_ACCELERATION, 0)
                print(f"  Node {node_id}: {val if val is not None else '응답 없음'}")
        elif char == 'c':
            print("\n[이동 속도 읽기]")
            for node_id in target_nodes:
                bus = buses[NODE_CONFIG[node_id]]
                val = read_sdo_value(bus, node_id, OD_PROFILE_VELOCITY, 0)
                print(f"  Node {node_id}: {val if val is not None else '응답 없음'}")

    except AttributeError:
        if key == keyboard.Key.esc:
            is_running = False
            return False # 리스너 종료

# --- 메인 실행 함수 ---
def main():
    global target_nodes, NODE_CONFIG
    # NODE_CONFIG를 id 기준으로 딕셔너리로 변환하여 사용하기 쉽게 만듦
    NODE_CONFIG = {motor['id']: motor['bus'] for motor in STEER_MOTORS}

    # (이하 코드는 이전과 유사하게 제어 대상 설정)
    if len(sys.argv) != 2:
        print("사용법: python3 keyboard_steer_control.py [NODE_ID | 'all']")
        return
    node_arg = sys.argv[1]
    if node_arg.lower() == 'all':
        target_nodes = list(NODE_CONFIG.keys())
    else:
        target_nodes = [int(node_arg)]

    try:
        # CAN 버스 및 모터 초기화
        bus_names = set(NODE_CONFIG[node] for node in target_nodes)
        for name in bus_names:
            buses[name] = can.interface.Bus(channel=name, bustype='socketcan')
        
        for node_id in target_nodes:
            bus = buses[NODE_CONFIG[node_id]]
            # 1. 위치 모드(1)로 설정
            send_sdo_write(bus, node_id, OD_MODES_OF_OPERATION, 0, 1, 1)
            time.sleep(0.05)
            # 2. Servo ON
            for state_val in [0x06, 0x07, 0x0F]:
                send_sdo_write(bus, node_id, OD_CONTROL_WORD, 0, state_val, 2)
                time.sleep(0.05)

        # 초기 설정값 전송
        command_all_nodes('velocity', current_profile_velocity)
        command_all_nodes('accel', current_profile_accel)
        command_all_nodes('position', current_target_degrees)
        trigger_motion()

        # 키보드 리스너 시작 및 상태 표시 루프
        listener = keyboard.Listener(on_press=on_press)
        listener.start()
        
        print("\n" + "="*50)
        print("실시간 조향 제어를 시작합니다. (목표 각도 기준)")
        print(f"제어 대상: {target_nodes}")
        print("[q]: -1도 | [e]: +1도 | [w]: 0도(원점)")
        print("[h/g]: 가감속↑/↓ | [b/v]: 속도↑/↓")
        print("[f]: 가감속 읽기 | [c]: 속도 읽기 | [Esc]: 종료")
        print("="*50)

        while is_running:
            status_line = (
                f"\r >> 목표각도: {current_target_degrees:6.1f}° | "
                f"속도설정: {current_profile_velocity:7d} | "
                f"가감속설정: {current_profile_accel:7d}  "
            )
            print(status_line, end="")
            time.sleep(0.1)

    except Exception as e:
        print(f"\n[오류] 프로그램 실행 중 에러: {e}")
    finally:
        if buses:
            print("\n\n프로그램을 종료합니다. 모터를 원점으로 이동합니다...")
            command_all_nodes('position', 0.0)
            trigger_motion()
            time.sleep(2) # 원점 이동 시간 대기
            for bus in buses.values():
                bus.shutdown()
            print("모든 CAN 버스가 종료되었습니다.")

if __name__ == "__main__":
    main()