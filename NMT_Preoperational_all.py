# CANopen의 NMT(Network Management) 명령어이며, Node ID x 인 드라이버를 'Pre-Operational' (사전 작동) 상태로 전환하라는 함수
# **'Pre-Operational'**은 모터가 실제 구동(회전)을 시작하기 전의 준비 및 설정 단계입니다.

# 이 상태에서 드라이버는 다음과 같은 작업을 할 수 있습니다.


# SDO 통신: 파라미터를 읽거나 쓰는 등의 설정 변경이 가능합니다. 


# SYNC, EMCY 메시지: 동기화 신호를 받거나 에러 메시지를 보낼 수 있습니다. 

# 하지만 이 상태에서는 위치나 속도 명령과 같은 실시간 데이터(PDO)를 주고받으며 모터를 구동할 수는 없습니다.

import can
import time

# --- 설정 값 ---
# 각 노드 ID가 어떤 CAN 버스에 연결되어 있는지 정의합니다.
NODE_CONFIG = {
    2: 'can1',
    4: 'can0',
    6: 'can1',
    8: 'can0',
    1: 'can1',
    3: 'can0',
    5: 'can1',
    7: 'can0',
}

def main():
    """
    설정된 모든 노드에 'Enter Pre-Operational' NMT 명령을 전송합니다.
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
        print("모든 노드를 Pre-Operational 상태로 전환합니다...")
        print("="*50)

        # NMT 명령어 정보
        NMT_COB_ID = 0x000
        CMD_ENTER_PRE_OP = 0x80

        # 2. 설정된 모든 노드에 순차적으로 NMT 명령을 전송합니다.
        for node_id, bus_name in NODE_CONFIG.items():
            bus = buses[bus_name]
            
            # 데이터 페이로드: [명령어, 노드ID]
            data_payload = [CMD_ENTER_PRE_OP, node_id]

            msg = can.Message(
                arbitration_id=NMT_COB_ID,
                data=data_payload,
                is_extended_id=False
            )

            try:
                bus.send(msg)
                print(f" -> Node {node_id} ({bus_name})에 Pre-Operational 전환 명령 전송 완료.")
            except can.CanError as e:
                print(f" -> Node {node_id} ({bus_name})에 명령 전송 실패: {e}")
            
            time.sleep(0.05)
        
        print("\n모든 명령 전송이 완료되었습니다.")

    finally:
        # 3. 사용한 모든 버스를 안전하게 종료합니다.
        for bus_name, bus in buses.items():
            if bus:
                bus.shutdown()
                print(f"CAN 버스 '{bus_name}'가 종료되었습니다.")

if __name__ == "__main__":
    main()