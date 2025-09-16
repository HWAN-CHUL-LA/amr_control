import can
import time

# --- 설정 값 ---
# 각 노드 ID가 어떤 CAN 버스에 연결되어 있는지 정의합니다.
NODE_CONFIG = {
    2: 'can1',
    4: 'can0',
    6: 'can1',
    8: 'can0',
}

def send_sdo_write_2byte(bus: can.interface.Bus, node_id: int, index: int, value: int):
    """
    지정된 2바이트 값으로 SDO Write 명령을 생성하고 전송합니다.
    """
    cob_id = 0x600 + node_id
    
    # SDO 2바이트 쓰기 명령어(2Bh), Index(le), Sub-index, 데이터(le)
    index_bytes = struct.pack('<H', index)
    data_bytes = struct.pack('<H', value)
    
    payload = [0x2B, index_bytes[0], index_bytes[1], 0x00] + list(data_bytes)
    payload.extend([0] * (8 - len(payload)))

    msg = can.Message(arbitration_id=cob_id, data=payload, is_extended_id=False)
    
    try:
        bus.send(msg)
        print(f" -> Node {node_id} ({bus.channel_info}): E-STOP 명령 전송 완료.")
    except can.CanError as e:
        print(f" -> Node {node_id} ({bus.channel_info}): 명령 전송 실패: {e}")

def main():
    """
    설정된 모든 노드에 E-STOP (Quick Stop) 명령을 전송합니다.
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
        print("모든 노드에 E-STOP (Quick Stop) 명령을 전송합니다...")
        print("="*50)

        # Controlword 객체 주소 및 Quick Stop 명령어
        OD_CONTROL_WORD = 0x6040
        CMD_QUICK_STOP = 0x0002

        # 2. 설정된 모든 노드에 순차적으로 명령을 전송합니다.
        for node_id, bus_name in NODE_CONFIG.items():
            bus = buses[bus_name]
            send_sdo_write_2byte(bus, node_id, OD_CONTROL_WORD, CMD_QUICK_STOP)
            time.sleep(0.05)
        
        print("\n모든 E-STOP 명령 전송이 완료되었습니다.")

    finally:
        # 3. 사용한 모든 버스를 안전하게 종료합니다.
        for bus_name, bus in buses.items():
            if bus:
                bus.shutdown()
                print(f"CAN 버스 '{bus_name}'가 종료되었습니다.")

if __name__ == "__main__":
    # 이 코드는 struct 라이브러리가 필요합니다.
    import struct
    main()