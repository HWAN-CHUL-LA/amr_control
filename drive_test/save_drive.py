import can
import time
import struct

# --- 설정 값 ---
# 각 노드 ID가 어떤 CAN 버스에 연결되어 있는지 정의합니다.
NODE_CONFIG = {
    1: 'can1',
    3: 'can0',
    5: 'can1',
    7: 'can0',
}

def send_save_command(bus: can.interface.Bus, node_id: int):
    """지정된 노드에 'Save All Parameters' 명령을 전송합니다."""
    cob_id = 0x600 + node_id
    
    # 'save' 명령어 페이로드: 
    # Index 1010h, Sub-index 1, Data 'save' (0x65766173)
    # 0x23: 4-byte write command
    # 0x10, 0x10: Index 1010h (little-endian)
    # 0x01: Sub-index 1
    # 0x73, 0x61, 0x76, 0x65: 's', 'a', 'v', 'e' ASCII (little-endian)
    payload = [0x23, 0x10, 0x10, 0x01, 0x73, 0x61, 0x76, 0x65]
    
    msg = can.Message(
        arbitration_id=cob_id,
        data=payload,
        is_extended_id=False
    )
    
    try:
        bus.send(msg)
        print(f" -> Node {node_id} ({bus.channel_info}): Save 명령 전송 완료.")
    except can.CanError as e:
        print(f" -> Node {node_id} ({bus.channel_info}): 명령 전송 실패: {e}")

def main():
    """
    설정된 모든 노드에 파라미터 저장(Save) 명령을 전송합니다.
    """
    buses = {}
    try:
        # 1. 필요한 모든 CAN 버스를 중복 없이 초기화합니다.
        unique_bus_names = set(NODE_CONFIG.values())
        for bus_name in unique_bus_names:
            try:
                buses[bus_name] = can.interface.Bus(channel=bus_name, bustype='socketcan')
                print(f"CAN 버스 '{bus_name}'가 성공적으로 초기화되었습니다.")
            except can.CanError as e:
                print(f"CAN 버스 '{bus_name}' 초기화 실패: {e}")
                return

        print("\n" + "="*50)
        print("모든 노드에 파라미터 저장(Save) 명령을 전송합니다...")
        print("="*50)

        # 2. 설정된 모든 노드에 순차적으로 Save 명령을 전송합니다.
        for node_id, bus_name in NODE_CONFIG.items():
            bus = buses[bus_name]
            send_save_command(bus, node_id)
            time.sleep(0.05)
        
        print("\n모든 Save 명령 전송이 완료되었습니다.")
        print("이제 드라이버의 전원을 재시작하면 변경된 설정이 유지됩니다.")

    finally:
        # 3. 사용한 모든 버스를 안전하게 종료합니다.
        for bus_name, bus in buses.items():
            if bus:
                bus.shutdown()
                print(f"CAN 버스 '{bus_name}'가 종료되었습니다.")

if __name__ == "__main__":
    main()
