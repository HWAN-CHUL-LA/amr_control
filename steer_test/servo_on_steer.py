import can
import time
import struct

# --- 설정 값 ---
# 제어할 조향 모터의 노드 ID와 CAN 버스를 정의합니다.
STEERING_NODE_CONFIG = {
    2: 'can1',
    6: 'can1',
    4: 'can0',
    8: 'can0',
}

# CANopen 객체 주소
OD_CONTROL_WORD = 0x6040

def send_control_word(bus: can.interface.Bus, node_id: int, value: int):
    """지정된 노드의 Controlword에 2바이트 값을 전송합니다."""
    cob_id = 0x600 + node_id
    
    # SDO 2바이트 쓰기 명령어(2Bh), Index(le), Sub-index, 데이터(le)
    index_bytes = struct.pack('<H', OD_CONTROL_WORD)
    data_bytes = struct.pack('<H', value)
    
    payload = [0x2B, index_bytes[0], index_bytes[1], 0x00] + list(data_bytes)
    payload.extend([0] * (8 - len(payload)))

    msg = can.Message(arbitration_id=cob_id, data=payload, is_extended_id=False)
    
    try:
        bus.send(msg)
    except can.CanError as e:
        print(f" -> Node {node_id} ({bus.channel_info}): 명령 전송 실패: {e}")

def sequence_servo_on(bus: can.interface.Bus, node_id: int):
    """지정된 노드에 Servo On 시퀀스(3단계)를 전송합니다."""
    print(f" -> Node {node_id} ({bus.channel_info}): Servo On 시퀀스 시작...")
    
    # 1단계: Ready to Switch On 상태로 전환
    send_control_word(bus, node_id, 0x0006)
    time.sleep(0.05)
    
    # 2단계: Switched On 상태로 전환
    send_control_word(bus, node_id, 0x0007)
    time.sleep(0.05)

    # 3단계: Operation Enable 상태로 전환 (최종 Servo On)
    send_control_word(bus, node_id, 0x000F)
    print(f" -> Node {node_id} ({bus.channel_info}): Servo On 완료.")


def main():
    """
    설정된 모든 조향 모터에 Servo On 명령을 전송합니다.
    """
    buses = {}
    try:
        # 1. 필요한 모든 CAN 버스를 중복 없이 초기화합니다.
        unique_bus_names = set(STEERING_NODE_CONFIG.values())
        for bus_name in unique_bus_names:
            try:
                buses[bus_name] = can.interface.Bus(channel=bus_name, bustype='socketcan')
                print(f"CAN 버스 '{bus_name}'가 성공적으로 초기화되었습니다.")
            except can.CanError as e:
                print(f"CAN 버스 '{bus_name}' 초기화 실패: {e}")
                return

        print("\n" + "="*50)
        print("모든 조향 모터에 Servo On 명령을 전송합니다...")
        print("="*50)

        # 2. 설정된 모든 조향 노드에 순차적으로 명령을 전송합니다.
        for node_id, bus_name in STEERING_NODE_CONFIG.items():
            bus = buses[bus_name]
            sequence_servo_on(bus, node_id)
            time.sleep(0.1) # 다음 노드로 넘어가기 전 딜레이
        
        print("\n모든 Servo On 명령 전송이 완료되었습니다.")

    finally:
        # 3. 사용한 모든 버스를 안전하게 종료합니다.
        for bus_name, bus in buses.items():
            if bus:
                bus.shutdown()
                print(f"CAN 버스 '{bus_name}'가 종료되었습니다.")

if __name__ == "__main__":
    main()