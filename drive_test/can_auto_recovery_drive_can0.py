# 명령어 : sudo python3 can_auto_recovery.py


import can
import time
import subprocess
import os

# --- 설정 ---
CAN_INTERFACE = 'can0'  # 감시할 CAN 인터페이스
# -------------

def restart_can_interface(interface: str):
    """지정된 CAN 인터페이스를 재시작합니다."""
    print(f"🚨 CAN Bus-Off 감지! '{interface}' 인터페이스를 재시작합니다...")
    try:
        # 인터페이스를 내렸다가 다시 올립니다.
        subprocess.run(['sudo', 'ip', 'link', 'set', interface, 'down'], check=True)
        time.sleep(0.1)
        subprocess.run(['sudo', 'ip', 'link', 'set', interface, 'up'], check=True)
        print(f"✅ '{interface}' 인터페이스가 성공적으로 재시작되었습니다.")
    except subprocess.CalledProcessError as e:
        print(f"❌ 인터페이스 재시작 실패: {e}")
    except FileNotFoundError:
        print("❌ 'ip' 명령어를 찾을 수 없습니다. 리눅스 환경에서 실행해주세요.")

def main():
    """CAN 버스를 감시하며 Bus-Off 에러 발생 시 자동 복구를 수행합니다."""
    print(f"'{CAN_INTERFACE}' 인터페이스 감시를 시작합니다. (Ctrl+C로 종료)")
    bus = None
    
    # 스크립트가 sudo 권한으로 실행되었는지 확인
    if os.geteuid() != 0:
        print("\n⚠️ 경고: 이 스크립트는 'sudo' 권한으로 실행해야 합니다.")
        print("sudo python3 can_auto_recovery.py")
        return

    while True:
        try:
            # 에러 프레임도 수신하도록 버스를 초기화합니다.
            bus = can.interface.Bus(channel=CAN_INTERFACE, bustype='socketcan')

            for msg in bus:
                if msg.is_error_frame:
                    # arbitration_id에 에러 종류가 비트마스크로 담겨 옵니다.
                    error_code = msg.arbitration_id
                    if error_code & can.CAN_ERR_BUSOFF:
                        # Bus-Off 에러가 감지되면 버스를 종료하고 재시작
                        bus.shutdown()
                        restart_can_interface(CAN_INTERFACE)
                        # 재시작 후 다시 루프를 시작하기 위해 break
                        break
        
        except can.CanError as e:
            print(f"CAN 에러 발생: {e}. 5초 후 재시도합니다...")
            if bus:
                bus.shutdown()
            time.sleep(5)
        except KeyboardInterrupt:
            print("\n프로그램을 종료합니다.")
            break
        finally:
            if bus:
                bus.shutdown()

if __name__ == "__main__":
    main()
