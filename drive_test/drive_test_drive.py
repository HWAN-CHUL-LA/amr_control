import can
import struct
import time
import sys
import math

# --- AMR 모터 및 제어 상수 설정 (이전과 동일) ---
WHEEL_RADIUS_M = 0.075
WHEEL_CIRCUMFERENCE_M = 2 * math.pi * WHEEL_RADIUS_M
DRIVE_REDUCTION_RATIO = 20.0
FEED_CONSTANT = 10000.0

# 제어할 주행 모터 노드와 연결된 CAN 버스 이름
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
OD_PROFILE_ACCELERATION = 0x6083
OD_PROFILE_DECELERATION = 0x6084

# --- 함수 정의 (이전과 동일) ---
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
    print(f" -> Node {node_id}: Servo On 및 속도 제어 모드 설정 중...")
    send_sdo_write(bus, node_id, OD_CONTROL_WORD, 0, 0x06, 2)
    send_sdo_write(bus, node_id, OD_CONTROL_WORD, 0, 0x07, 2)
    send_sdo_write(bus, node_id, OD_MODES_OF_OPERATION, 0, 3, 1)
    send_sdo_write(bus, node_id, OD_CONTROL_WORD, 0, 0x0F, 2)
    print(f" -> Node {node_id}: Operation Enable 상태 완료.")

def set_motor_speed(bus: can.interface.Bus, node_id: int, speed_mps: float):
    target_pulse = mps_to_pulse_per_second(speed_mps)
    print(f" -> Node {node_id}: 목표 속도 {speed_mps:.3f} m/s ({target_pulse} pulse/s) 전송.")
    send_sdo_write(bus, node_id, OD_TARGET_VELOCITY, 0, target_pulse, 4)

def set_acceleration(bus: can.interface.Bus, node_id: int, accel_val: int):
    print(f" -> Node {node_id}: 가감속도 {accel_val} pulse/s^2 설정.")
    send_sdo_write(bus, node_id, OD_PROFILE_ACCELERATION, 0, accel_val, 4)
    send_sdo_write(bus, node_id, OD_PROFILE_DECELERATION, 0, accel_val, 4)

# --- ✨ 여기가 수정된 main 함수 ---
def main():
    if len(sys.argv) != 2:
        print("사용법: python3 interactive_motor_all.py [NODE_ID | 'all']")
        print("예시 1 (단일 노드): python3 interactive_motor_all.py 3")
        print("예시 2 (모든 노드): python3 interactive_motor_all.py all")
        return

    # 1. 'all' 또는 단일 노드 ID를 처리하여 제어 대상 목록 생성
    node_arg = sys.argv[1]
    target_nodes = []
    if node_arg.lower() == 'all':
        target_nodes = list(NODE_CONFIG.keys())
        print(f"** 제어 대상: 모든 노드 {target_nodes} **")
    else:
        try:
            node_id = int(node_arg)
            if node_id not in NODE_CONFIG:
                print(f"[오류] 노드 ID {node_id}가 설정 파일에 없습니다.")
                return
            target_nodes = [node_id]
            print(f"** 제어 대상: 단일 노드 {target_nodes} **")
        except ValueError:
            print(f"[오류] 유효하지 않은 노드 ID입니다: '{node_arg}'")
            return

    buses = {}
    try:
        # 2. 필요한 모든 CAN 버스 초기화
        unique_bus_names = set(NODE_CONFIG.values())
        for bus_name in unique_bus_names:
            buses[bus_name] = can.interface.Bus(channel=bus_name, bustype='socketcan')
            print(f" -> CAN 버스 '{bus_name}'가 활성화되었습니다.")
        
        # 3. 모든 제어 대상 노드에 대해 초기 설정 진행
        for node_id in target_nodes:
            bus = buses[NODE_CONFIG[node_id]] # 해당 노드에 맞는 버스 선택
            setup_velocity_mode(bus, node_id)
            set_acceleration(bus, node_id, 50000) # 기본 가감속도 설정
        
        print("\n" + "="*50)
        print("실시간 모터 제어를 시작합니다.")
        print("원하는 속도를 m/s 단위로 입력 후 Enter를 누르세요.")
        print("프로그램을 종료하려면 'q'를 입력하세요.")
        print("="*50)

        # 4. 사용자 입력을 기다리는 무한 루프
        while True:
            user_input = input(f"모든 대상 노드 목표 속도 (m/s) 입력 ('q' 입력 시 종료): ")

            if user_input.lower() == 'q':
                print("종료 명령을 수신했습니다.")
                break

            try:
                speed_mps = float(user_input)
                # 모든 제어 대상 노드에 속도 명령 전송
                for node_id in target_nodes:
                    bus = buses[NODE_CONFIG[node_id]]
                    set_motor_speed(bus, node_id, speed_mps)
            except ValueError:
                print("[경고] 잘못된 입력입니다. 숫자를 입력하거나 'q'로 종료하세요.")

    except KeyboardInterrupt:
        print("\n[알림] Ctrl+C로 프로그램을 강제 종료합니다.")
    except Exception as e:
        print(f"\n[오류] 프로그램 실행 중 에러가 발생했습니다: {e}")
    finally:
        # 5. 프로그램 종료 시 모든 노드를 정지하고 모든 버스 연결 해제
        if buses:
            print("\n프로그램을 종료하기 전 모든 모터를 안전하게 정지합니다...")
            for node_id in target_nodes:
                bus = buses[NODE_CONFIG[node_id]]
                set_motor_speed(bus, node_id, 0.0)
            
            time.sleep(1)
            
            for bus_name, bus in buses.items():
                bus.shutdown()
                print(f" -> CAN 버스 '{bus_name}'가 종료되었습니다.")

if __name__ == "__main__":
    main()