import can
import struct
import time
import sys
import math
import os
from pynput import keyboard

# --- AMR 모터 및 제어 상수 설정 ---
WHEEL_RADIUS_M = 0.075
WHEEL_CIRCUMFERENCE_M = 2 * math.pi * WHEEL_RADIUS_M
DRIVE_REDUCTION_RATIO = 20.0
FEED_CONSTANT = 10000.0

NODE_CONFIG = {
    1: 'can1',
    3: 'can0',
    5: 'can1',
    7: 'can0',
}

OD_CONTROL_WORD = 0x6040
OD_MODES_OF_OPERATION = 0x6060
OD_TARGET_VELOCITY = 0x60FF
OD_PROFILE_ACCELERATION = 0x6083
OD_PROFILE_DECELERATION = 0x6084

# --- 실시간 제어를 위한 전역 변수 ---
buses = {}
target_nodes = []
current_speed_mps = 0.0
current_accel_pps2 = 50000 # 초기 가감속도 값
is_running = True

# 속도 및 가감속도 변경 단위
SPEED_UNIT_MPS = 0.05  # w, x 키를 누를 때마다 변경될 속도 (m/s)
ACCEL_UNIT_PPS2 = 10000 # q, z 키를 누를 때마다 변경될 가감속도 (pulse/s^2)

# --- CANopen 함수들 (이전과 동일) ---
def mps_to_pulse_per_second(speed_mps: float) -> int:
    wheel_rps = speed_mps / WHEEL_CIRCUMFERENCE_M
    motor_rps = wheel_rps * DRIVE_REDUCTION_RATIO
    pulse_per_second = motor_rps * FEED_CONSTANT
    return int(pulse_per_second)

def send_sdo_write(bus: can.interface.Bus, node_id: int, index: int, sub_index: int, value: int, size: int):
    cob_id = 0x600 + node_id
    if size == 1: cmd, fmt = 0x2F, '<B'
    elif size == 2: cmd, fmt = 0x2B, '<H'
    elif size == 4: cmd, fmt = 0x23, '<i'
    else: raise ValueError("Unsupported data size")
    index_bytes = struct.pack('<H', index)
    data_bytes = struct.pack(fmt, value)
    payload = [cmd, index_bytes[0], index_bytes[1], sub_index] + list(data_bytes)
    payload.extend([0] * (8 - len(payload)))
    msg = can.Message(arbitration_id=cob_id, data=payload, is_extended_id=False)
    bus.send(msg)
    time.sleep(0.01)

def setup_velocity_mode(bus: can.interface.Bus, node_id: int):
    print(f" -> Node {node_id}: Servo On 및 속도 제어 모드 설정...")
    send_sdo_write(bus, node_id, OD_CONTROL_WORD, 0, 0x06, 2)
    send_sdo_write(bus, node_id, OD_CONTROL_WORD, 0, 0x07, 2)
    send_sdo_write(bus, node_id, OD_MODES_OF_OPERATION, 0, 3, 1)
    send_sdo_write(bus, node_id, OD_CONTROL_WORD, 0, 0x0F, 2)

# --- ✨ 모든 노드에 명령을 보내는 통합 함수 ---
def command_all_nodes(command_type, value):
    """모든 대상 노드에 특정 명령(속도 또는 가감속)을 전송합니다."""
    for node_id in target_nodes:
        bus = buses[NODE_CONFIG[node_id]]
        if command_type == 'speed':
            target_pulse = mps_to_pulse_per_second(value)
            send_sdo_write(bus, node_id, OD_TARGET_VELOCITY, 0, target_pulse, 4)
        elif command_type == 'accel':
            send_sdo_write(bus, node_id, OD_PROFILE_ACCELERATION, 0, int(value), 4)
            send_sdo_write(bus, node_id, OD_PROFILE_DECELERATION, 0, int(value), 4)

# --- ✨ 키보드 입력 처리 함수 ---
def on_press(key):
    global current_speed_mps, current_accel_pps2, is_running

    try:
        char = key.char
        # 속도 제어
        if char == 'w':
            current_speed_mps += SPEED_UNIT_MPS
            command_all_nodes('speed', current_speed_mps)
        elif char == 'x':
            current_speed_mps -= SPEED_UNIT_MPS
            command_all_nodes('speed', current_speed_mps)
        elif char == 's':
            current_speed_mps = 0.0
            command_all_nodes('speed', current_speed_mps)
        # 가감속도 제어
        elif char == 'q':
            current_accel_pps2 += ACCEL_UNIT_PPS2
            command_all_nodes('accel', current_accel_pps2)
        elif char == 'z':
            current_accel_pps2 -= ACCEL_UNIT_PPS2
            if current_accel_pps2 < 1000: # 최소값 제한
                current_accel_pps2 = 1000
            command_all_nodes('accel', current_accel_pps2)
            
    except AttributeError:
        # 특수 키(Esc 등) 처리
        if key == keyboard.Key.esc:
            is_running = False
            return False # 리스너 종료

# --- ✨ 메인 실행 함수 ---
def main():
    if len(sys.argv) != 2:
        print("사용법: python3 keyboard_control.py [NODE_ID | 'all']")
        return
    
    global target_nodes
    node_arg = sys.argv[1]
    if node_arg.lower() == 'all':
        target_nodes = list(NODE_CONFIG.keys())
    else:
        target_nodes = [int(node_arg)]

    try:
        # CAN 버스 초기화
        unique_bus_names = set(NODE_CONFIG[node] for node in target_nodes)
        for bus_name in unique_bus_names:
            buses[bus_name] = can.interface.Bus(channel=bus_name, bustype='socketcan')
        
        # 모터 초기 설정
        for node_id in target_nodes:
            bus = buses[NODE_CONFIG[node_id]]
            setup_velocity_mode(bus, node_id)
        command_all_nodes('accel', current_accel_pps2) # 초기 가감속도 전송
        
        # 키보드 리스너 시작
        listener = keyboard.Listener(on_press=on_press)
        listener.start()

        # 상태 표시 및 메인 루프
        print("\n" + "="*50)
        print("실시간 키보드 제어를 시작합니다.")
        print(f"제어 대상: {target_nodes}")
        print(" [w]: 속도 증가 | [x]: 속도 감소 | [s]: 즉시 정지")
        print(" [q]: 반응 속도↑ (가감속 증가) | [z]: 반응 속도↓ (가감속 감소)")
        print(" [Esc]: 프로그램 종료")
        print("="*50)

        while is_running:
            # 실시간으로 현재 상태를 터미널에 출력
            # \r은 커서를 줄의 맨 앞으로 이동시켜 덮어쓰는 효과를 줌
            print(f"\r >> 현재 속도: {current_speed_mps:6.2f} m/s | 현재 가감속: {current_accel_pps2:7d}  ", end="")
            time.sleep(0.1)

    except Exception as e:
        print(f"\n[오류] 프로그램 실행 중 에러: {e}")
    finally:
        # 프로그램 종료 처리
        if buses:
            print("\n\n프로그램을 종료합니다. 모터를 정지합니다...")
            command_all_nodes('speed', 0.0)
            time.sleep(1)
            for bus in buses.values():
                bus.shutdown()
            print("모든 CAN 버스가 종료되었습니다.")

if __name__ == "__main__":
    main()