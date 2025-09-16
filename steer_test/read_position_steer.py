import can
import struct
import time

# --- 설정 값 ---
# 각 노드 ID가 어떤 CAN 버스에 연결되어 있는지 정의합니다.
NODE_CONFIG = {
    2: 'can1',
    4: 'can0',
    6: 'can1',
    8: 'can0',
}

# 읽어올 CANopen 객체 주소
OD_INTERNAL_POS = 0x6063  # 내부 실제 위치 (Raw)
OD_FEEDBACK_POS = 0x6064  # 피드백 위치 (명령 단위)

def read_sdo(bus: can.interface.Bus, node_id: int, index: int, sub_index: int = 0) -> int | None:
    """
    지정된 객체의 값을 읽기 위해 SDO Read 요청을 보내고 응답을 기다립니다.
    응답이 오면 값을 10진수 정수로 변환하여 반환하고, 없으면 None을 반환합니다.
    """
    cob_id_request = 0x600 + node_id
    cob_id_response = 0x580 + node_id

    # SDO Read 요청 페이로드: 40h (읽기명령) + Index(le) + SubIndex + 4bytes(dummy)
    index_bytes = struct.pack('<H', index)
    payload = [0x40, index_bytes[0], index_bytes[1], sub_index, 0, 0, 0, 0]

    # 메시지 생성 및 전송
    msg = can.Message(arbitration_id=cob_id_request, data=payload, is_extended_id=False)
    
    try:
        bus.send(msg)
    except can.CanError as e:
        print(f"  [Error] Node {node_id}로 메시지 전송 실패: {e}")
        return None

    # 응답 메시지 기다리기 (1초 타임아웃)
    for _ in range(10): # 0.1초 간격으로 10번 시도
        response = bus.recv(timeout=0.1)
        if response and response.arbitration_id == cob_id_response:
            # 응답 데이터의 마지막 4바이트를 little-endian signed integer로 변환
            if len(response.data) == 8:
                value_bytes = response.data[4:8]
                value = struct.unpack('<i', value_bytes)[0]
                return value
    
    # 타임아웃
    return None

def main():
    buses = {}
    try:
        # 설정된 모든 CAN 버스 초기화
        unique_bus_names = set(NODE_CONFIG.values())
        for bus_name in unique_bus_names:
            try:
                buses[bus_name] = can.interface.Bus(channel=bus_name, bustype='socketcan')
                print(f"CAN 버스 '{bus_name}'가 성공적으로 초기화되었습니다.")
            except can.CanError as e:
                print(f"CAN 버스 '{bus_name}' 초기화 실패: {e}")
                return
        
        print("\n" + "="*40)
        print("각 노드의 현재 위치 값을 읽습니다...")
        print("="*40)

        # 설정된 모든 노드에 대해 순차적으로 값 읽기
        for node_id, bus_name in NODE_CONFIG.items():
            bus = buses.get(bus_name)
            if not bus:
                print(f"\n--- [ Node {node_id} ] ---")
                print(f"  [Error] '{bus_name}' 버스를 찾을 수 없습니다.")
                continue

            print(f"\n--- [ Node {node_id} on {bus_name} ] ---")
            
            # 0x6063 값 읽기
            internal_pos = read_sdo(bus, node_id, OD_INTERNAL_POS)
            if internal_pos is not None:
                print(f"  - 내부 실제 위치 (0x6063): {internal_pos:,} pulses")
                print("    (의미: 전자 기어비가 적용되기 전의 순수 엔코더 값입니다.)")
            else:
                print(f"  - 내부 실제 위치 (0x6063): 응답 없음.")

            time.sleep(0.05) # 다음 명령 전송 전 짧은 딜레이

            # 0x6064 값 읽기
            feedback_pos = read_sdo(bus, node_id, OD_FEEDBACK_POS)
            if feedback_pos is not None:
                print(f"  - 피드백 위치 (0x6064): {feedback_pos:,} pulses")
                print("    (의미: 전자 기어비가 적용된 후의 값으로, 실제 위치 명령에 사용되는 단위입니다.)")
            else:
                print(f"  - 피드백 위치 (0x6064): 응답 없음.")

    finally:
        for bus_name, bus in buses.items():
            if bus:
                bus.shutdown()
                print(f"\nCAN 버스 '{bus_name}'가 종료되었습니다.")

if __name__ == "__main__":
    main()