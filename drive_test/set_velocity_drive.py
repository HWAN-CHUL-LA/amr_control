import can
import struct
import time
import sys
import math

# --- AMR 모터 및 제어 상수 설정 ---
# 제공된 로봇 사양을 기반으로 합니다.
WHEEL_RADIUS_M = 0.075  # 바퀴 반지름 (미터)
WHEEL_CIRCUMFERENCE_M = 2 * math.pi * WHEEL_RADIUS_M  # 바퀴 원주 (미터)
DRIVE_REDUCTION_RATIO = 20.0  # 주행모터 감속비
# 드라이버의 1회전 당 명령 펄스 기본값. (필요시 18000 등으로 변경)
FEED_CONSTANT = 10000.0

# 제어할 주행 모터 노드 목록
NODE_CONFIG = {
    1: 'can1',
    3: 'can0',
    5: 'can1',
    7: 'can0',
}

# CANopen 객체 주소 (Object Dictionary)
OD_CONTROL_WORD = 0x6040
OD_MODES_OF_OPERATION = 0x6060
OD_TARGET_VELOCITY = 0x60FF

def mps_to_pulse_per_second(speed_mps: float) -> int:
    """미터/초(m/s) 속도를 드라이버 명령 단위인 펄스/초(pulse/s)로 변환합니다."""
    # 1. m/s -> 바퀴 RPS (초당 회전 수)
    wheel_rps = speed_mps / WHEEL_CIRCUMFERENCE_M
    # 2. 바퀴 RPS -> 모터 RPS
    motor_rps = wheel_rps * DRIVE_REDUCTION_RATIO
    # 3. 모터 RPS -> 명령 pulse/s
    pulse_per_second = motor_rps * FEED_CONSTANT
    return int(pulse_per_second)

def send_sdo_write(bus: can.interface.Bus, node_id: int, index: int, sub_index: int, value: int, size: int):
    """지정된 크기의 데이터로 SDO Write 명령을 생성하고 전송하는 범용 함수"""
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

def setup_velocity_mode(bus: can.interface.Bus, node_id: int):
    """지정된 노드를 Servo On 및 Profile Velocity 모드로 설정합니다."""
    print(f" -> Node {node_id}: Servo On 및 속도 제어 모드 설정 중...")
    send_sdo_write(bus, node_id, OD_CONTROL_WORD, 0, 0x06, 2) # Ready to Switch On
    time.sleep(0.05)
    send_sdo_write(bus, node_id, OD_CONTROL_WORD, 0, 0x07, 2) # Switched On
    time.sleep(0.05)
    send_sdo_write(bus, node_id, OD_MODES_OF_OPERATION, 0, 3, 1) # Profile Velocity Mode
    time.sleep(0.05)
    send_sdo_write(bus, node_id, OD_CONTROL_WORD, 0, 0x0F, 2) # Operation Enable
    time.sleep(0.05)

def main():
    if len(sys.argv) != 3:
        print("사용법: python3 drive_motor.py [NODE_ID 또는 'all'] [속도(m/s)]")
        print("예시 1: python3 drive_motor.py 3 0.5   (3번 노드를 0.5 m/s로 구동)")
        print("예시 2: python3 drive_motor.py all -0.2  (모든 노드를 -0.2 m/s로 구동)")
        return

    target_node_str = sys.argv[1]
    target_speed_mps = float(sys.argv[2])
    
    buses = {}
    try:
        unique_bus_names = set(NODE_CONFIG.values())
        for bus_name in unique_bus_names:
            buses[bus_name] = can.interface.Bus(channel=bus_name, bustype='socketcan')
        
        target_nodes = []
        if target_node_str.lower() == 'all':
            target_nodes = list(NODE_CONFIG.keys())
        else:
            target_nodes = [int(target_node_str)]

        # 1. 목표 속도를 pulse/s로 변환
        target_pulse_per_second = mps_to_pulse_per_second(target_speed_mps)
        print("\n" + "="*50)
        print(f"목표 속도: {target_speed_mps} m/s  ->  명령 값: {target_pulse_per_second} pulse/s")
        print("="*50)

        # 2. 대상 노드에 명령 전송
        for node_id in target_nodes:
            if node_id not in NODE_CONFIG:
                print(f"[경고] Node {node_id}는 설정에 없습니다. 건너뜁니다.")
                continue
            
            bus_name = NODE_CONFIG[node_id]
            bus = buses[bus_name]
            
            # 2-1. 서보 온 및 속도 모드 설정
            setup_velocity_mode(bus, node_id)

            # 2-2. 목표 속도 값 전송
            print(f" -> Node {node_id}: 목표 속도 {target_pulse_per_second} pulse/s 전송...")
            send_sdo_write(bus, node_id, OD_TARGET_VELOCITY, 0, target_pulse_per_second, 4)
            print(f" -> Node {node_id}: 명령 전송 완료.")

    except Exception as e:
        print(f"\n에러 발생: {e}")
    finally:
        for bus in buses.values():
            bus.shutdown()
        print("\n모든 CAN 버스가 종료되었습니다.")

if __name__ == "__main__":
    main()