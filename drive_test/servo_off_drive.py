import can
import time
import struct

# --- 설정 값 ---
# 제어할 조향 모터의 노드 ID와 CAN 버스를 정의합니다.
STEERING_NODE_CONFIG = {
    1: 'can1',
    5: 'can1',
    3: 'can0',
    7: 'can0',
}

def send_servo_off(bus: can.interface.Bus, node_id: int):
    """지정된 노드에 Servo Off (Shutdown) 명령을 전송합니다."""
    cob_id = 0x600 + node_id
    
    # Controlword(6040h)에 Shutdown(0x0006) 명령 전송
    # 0x2B: 2-byte write command
    # 0x40, 0x60: Index 6040h (little-endian)
    # 0x00: Sub-index 0
    # 0x06, 0x00: Value 0x0006 (little-endian)
    payload = [0x2B, 0x40, 0x60, 0x00, 0x06, 0x00]
    
    # 페이로드가 8바이트가 되도록 0으로 채웁니다.
    payload.extend([0] * (8 - len(payload)))

    msg = can.Message(
        arbitration_id=cob_id,
        data=payload,
        is_extended_id=False
    )
    
    try:
        bus.send(msg)
        print(f" -> Node {node_id} ({bus.channel_info}): Servo Off 명령 전송 완료.")
    except can.CanError as e:
        print(f" -> Node {node_id} ({bus.channel_info}): 명령 전송 실패: {e}")

def main():
    """
    설정된 모든 조향 모터에 Servo Off 명령을 전송합니다.
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
        print("모든 조향 모터에 Servo Off 명령을 전송합니다...")
        print("="*50)

        # 2. 설정된 모든 조향 노드에 순차적으로 명령을 전송합니다.
        for node_id, bus_name in STEERING_NODE_CONFIG.items():
            bus = buses[bus_name]
            send_servo_off(bus, node_id)
            time.sleep(0.05)
        
        print("\n모든 Servo Off 명령 전송이 완료되었습니다.")

    finally:
        # 3. 사용한 모든 버스를 안전하게 종료합니다.
        for bus_name, bus in buses.items():
            if bus:
                bus.shutdown()
                print(f"CAN 버스 '{bus_name}'가 종료되었습니다.")

if __name__ == "__main__":
    main()
