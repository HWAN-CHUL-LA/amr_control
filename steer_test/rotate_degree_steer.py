# 10진수를 입력하면 그 수만큼의 pulse 를 가한다. 
# 입력받은 10진수는 자동으로 16진수 - little Endian 값으로 변환하여 socketcan 형식으로 메세지를 던진다. 

import can
import struct
import time
import sys

# --- AMR 모터 및 제어 상수 설정 ---
# 이 값들은 사용자의 AMR 사양에 맞게 설정되었습니다.
TOTAL_REDUCTION_RATIO = 122.5
FEED_CONSTANT = 18000.0  # float으로 정의하여 계산 정확도 향상

# 1도당 필요한 펄스 값 자동 계산
PULSE_PER_DEGREE = (FEED_CONSTANT * TOTAL_REDUCTION_RATIO) / 360.0

# CANopen 객체 주소 (Object Dictionary)
OD_CONTROL_WORD = 0x6040
OD_MODES_OF_OPERATION = 0x6060
OD_TARGET_POSITION = 0x607A

def send_sdo_write(bus: can.interface.Bus, node_id: int, index: int, sub_index: int, value: int, size: int):
    """
    지정된 크기의 데이터로 SDO Write 명령을 생성하고 전송하는 범용 함수
    size: 데이터 크기 (1, 2, 4 바이트)
    """
    cob_id = 0x600 + node_id

    # 데이터 크기에 따른 SDO 명령어와 struct 포맷 선택
    if size == 1:
        cmd = 0x2F
        fmt = '<B' # Unsigned 1-byte
    elif size == 2:
        cmd = 0x2B
        fmt = '<H' # Unsigned 2-byte
    elif size == 4:
        cmd = 0x23
        fmt = '<i' # Signed 4-byte
    else:
        raise ValueError("Unsupported data size. Must be 1, 2, or 4.")

    # 데이터 페이로드 생성
    index_bytes = struct.pack('<H', index)
    data_bytes = struct.pack(fmt, value)
    
    payload = [cmd, index_bytes[0], index_bytes[1], sub_index] + list(data_bytes)
    # 페이로드가 8바이트 미만일 경우 0으로 채움
    payload.extend([0] * (8 - len(payload)))

    msg = can.Message(arbitration_id=cob_id, data=payload, is_extended_id=False)
    
    try:
        bus.send(msg)
        print(f" -> Sent to Node {node_id}: Index={hex(index)}, Value={value}, Size={size}bytes")
    except can.CanError as e:
        print(f"Message NOT sent: {e}")

def main():
    if len(sys.argv) != 4:
        print("사용법: python3 your_script_name.py [CAN_BUS] [NODE_ID] [DEGREES]")
        print("예시: python3 move_motor.py can0 4 90.5")
        return

    # --- 사용자 입력 값 ---
    can_interface = sys.argv[1]
    node_id = int(sys.argv[2])
    target_degrees = float(sys.argv[3])
    # --------------------

    bus = None
    try:
        # CAN 버스 초기화
        bus = can.interface.Bus(channel=can_interface, bustype='socketcan')
        print(f"\nCAN bus on '{can_interface}' initialized.")

        # 1. Servo On 및 Profile Position 모드 설정
        print("\nStep 1: Setting up motor (Servo On & Position Mode)...")
        # Shutdown (Switch On Disabled -> Ready to Switch On)
        send_sdo_write(bus, node_id, OD_CONTROL_WORD, 0, 0x06, 2)
        time.sleep(0.1)
        # Switch On (Ready to Switch On -> Switched On)
        send_sdo_write(bus, node_id, OD_CONTROL_WORD, 0, 0x07, 2)
        time.sleep(0.1)
        # Set Profile Position Mode
        send_sdo_write(bus, node_id, OD_MODES_OF_OPERATION, 0, 1, 1)
        time.sleep(0.1)
        # Enable Operation (Switched On -> Operation Enable)
        send_sdo_write(bus, node_id, OD_CONTROL_WORD, 0, 0x0F, 2)
        time.sleep(0.1)
        print("Motor setup complete.")

        # 2. 목표 각도를 펄스 값으로 변환
        target_pulses = int(target_degrees * PULSE_PER_DEGREE)
        
        print(f"\nStep 2: Converting angle to pulses...")
        print(f"  -> Target: {target_degrees} degrees")
        print(f"  -> Calculated Pulses: {target_pulses}")

        # 3. 목표 위치(펄스) 전송
        print("\nStep 3: Sending target position...")
        send_sdo_write(bus, node_id, OD_TARGET_POSITION, 0, target_pulses, 4)
        time.sleep(0.1)

        # 4. 모션 실행 트리거
        print("\nStep 4: Triggering motion...")
        # New Set-Point (bit 4) 활성화
        send_sdo_write(bus, node_id, OD_CONTROL_WORD, 0, 0x1F, 2)
        time.sleep(0.05)
        # New Set-Point 비활성화 (다음 명령을 위해)
        send_sdo_write(bus, node_id, OD_CONTROL_WORD, 0, 0x0F, 2)
        print("\nMotion command sent successfully!")

    except can.CanError as e:
        print(f"\nAn error occurred with the CAN bus: {e}")
    except ValueError as e:
        print(f"\nAn error occurred: {e}")
    finally:
        if bus:
            bus.shutdown()
            print("\nCAN bus shut down.")

if __name__ == "__main__":
    main()