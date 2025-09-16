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

# 보낼 데이터 페이로드 (Feed Constant 6092h, sub 1을 18000으로 설정)
# 18000 (10진수) -> 4650 (16진수) -> [0x50, 0x46, 0x00, 0x00] (리틀 엔디안)
PAYLOAD = [0x23, 0x92, 0x60, 0x01, 0x50, 0x46, 0x00, 0x00]

def main():
    """
    설정된 모든 노드에 Feed Constant 변경 명령을 전송합니다.
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
                return  # 하나의 버스라도 실패하면 프로그램을 종료합니다.

        print("\n" + "="*50)
        print("모든 노드에 Feed Constant(18000) 설정 명령을 전송합니다...")
        print("="*50)

        # 2. 설정된 모든 노드에 순차적으로 CAN 메시지를 전송합니다.
        for node_id, bus_name in NODE_CONFIG.items():
            bus = buses[bus_name]
            cob_id = 0x600 + node_id

            msg = can.Message(
                arbitration_id=cob_id,
                data=PAYLOAD,
                is_extended_id=False
            )

            try:
                bus.send(msg)
                print(f" -> Node {node_id} ({bus_name})에 명령 전송 완료.")
            except can.CanError as e:
                print(f" -> Node {node_id} ({bus_name})에 명령 전송 실패: {e}")
            
            # 버스 부하를 줄이기 위한 짧은 딜레이
            time.sleep(0.05)
        
        print("\n모든 명령 전송이 완료되었습니다.")
        print("⚠️ 설정을 영구 저장하려면 'Save' 명령어를 보내고 전원을 재시작하세요.")

    finally:
        # 3. 사용한 모든 버스를 안전하게 종료합니다.
        for bus_name, bus in buses.items():
            if bus:
                bus.shutdown()
                print(f"CAN 버스 '{bus_name}'가 종료되었습니다.")

if __name__ == "__main__":
    main()