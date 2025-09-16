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

def main():
    """
    설정된 모든 노드에 NMT Start (Enter Operational) 명령을 전송합니다.
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
        print("모든 노드를 NMT Start (Operational 상태)로 전환합니다...")
        print("="*50)

        # NMT 명령어 정보
        NMT_COB_ID = 0x000
        CMD_START_NODE = 0x01

        # 2. 설정된 모든 노드에 순차적으로 NMT 명령을 전송합니다.
        for node_id, bus_name in NODE_CONFIG.items():
            bus = buses[bus_name]
            
            # 데이터 페이로드: [명령어, 노드ID]
            data_payload = [CMD_START_NODE, node_id]

            msg = can.Message(
                arbitration_id=NMT_COB_ID,
                data=data_payload,
                is_extended_id=False
            )

            try:
                bus.send(msg)
                print(f" -> Node {node_id} ({bus_name})에 Start 명령 전송 완료.")
            except can.CanError as e:
                print(f" -> Node {node_id} ({bus_name})에 명령 전송 실패: {e}")
            
            time.sleep(0.05)
        
        print("\n모든 노드에 Start 명령 전송이 완료되었습니다.")

    finally:
        # 3. 사용한 모든 버스를 안전하게 종료합니다.
        for bus_name, bus in buses.items():
            if bus:
                bus.shutdown()
                print(f"CAN 버스 '{bus_name}'가 종료되었습니다.")

if __name__ == "__main__":
    main()