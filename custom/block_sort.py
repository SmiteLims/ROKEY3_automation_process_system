import rclpy
import DR_init
import time
from DR_common2 import posx, posj

ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 30, 30
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL
ON, OFF = 1, 0

# 감지 위치 정의
nocontact_z = 350.0
position_lst = [
    posx(198.21,-58.48,nocontact_z,147.25,-179.99,145.78),
    posx(249.95,-52.92,nocontact_z,11.30,-179.55,9.85),
    posx(303.75,-50.58,nocontact_z,3.91,-179.05,2.20),
    posx(204.68,-107.52,nocontact_z,6.47,-178.94,4.53),
    posx(257.29,-105.63,nocontact_z,3.56,-178.59,1.44),
    posx(311.57,-102.36,nocontact_z,179.50,177.77,176.98),
    posx(214.47,-159.75,nocontact_z,171.63,177.15,169.30),
    posx(267.19,-156.88,nocontact_z,173.03,176.69,170.36),
    posx(320.45,-155.79,nocontact_z,174.63,176.23,171.84)
]

# 팔레트 위치 (TODO: 실제 좌표 입력)
short_pallets = [posx(194.06, -212.82, 272.11, 136.89, 180, 135.3), posx(249.04, -208.6, 271.62, 131.21, 180, 129.54), posx(299.15, -204.28, 271.21, 125.49, 180, 123.58)]
mid_pallets   = [posx(198.61, -261.26, 270.73, 123.41, 179.99, 121.48), posx(253.2, -259.33, 270.37, 121.97, 179.99, 119.38), posx(301.92, -253.32, 270.1, 147.87, -180, 144.85)]
tall_pallets  = [posx(203.49, -311.72, 270.01, 116.34, 179.99, 110.69), posx(254.49, -307.87, 269.7, 159.93, -180, 153.76), posx(305.63, -304.16, 269.28, 144.86, -180, 138.41)]
block_to_down = posx(0, 0, -100, 0, 0, 0)  # 상대적 하강

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("rokey_sorter", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    try:
        from DSR_ROBOT2 import (
            set_tool, set_tcp, movej, movel,
            set_digital_output, get_current_posx,
            set_ref_coord, task_compliance_ctrl, set_stiffnessx,
            set_desired_force, release_force, release_compliance_ctrl,
            check_force_condition, mwait,
            DR_AXIS_Z, DR_BASE, DR_MV_MOD_REL
        )
    except ImportError as e:
        print(f"Error importing DSR_ROBOT2: {e}")
        return

    def grip():
        set_digital_output(1, ON)
        set_digital_output(2, OFF)

    def release():
        set_digital_output(2, ON)
        set_digital_output(1, OFF)

    def check_and_grab():
        set_ref_coord(0)
        task_compliance_ctrl()
        set_stiffnessx([3000.0]*3 + [200.0]*3)
        set_desired_force([0.0, 0.0, -30.0, 0.0, 0.0, 0.0], [0, 0, 1, 0, 0, 0])

        while True:
            if check_force_condition(axis=DR_AXIS_Z, max=25):
                release()
                print(1)
                time.sleep(0.2)
                print(2)
                z = get_current_posx()[0]
                break
        print(3)
        z[2] += 5.0
        print(4)
        time.sleep(0.2)
        print(5)
        movel(z, vel=150, acc=300, ref=DR_BASE)
        print(6)
        grip()  
        print(7) 
        release_force()
        release_compliance_ctrl()

        if height == 0:
            print("블럭 없음: 집기 생략")
            return 0

        # ✅ 측정 직후 살짝 더 내려가서 잡기
        release()
        time.sleep(0.2)
        print("⬇️ 블럭 위치에서 하강 및 그리퍼 동작")
        movel(block_to_down, vel=VELOCITY, acc=ACC, mod=DR_MV_MOD_REL)
        time.sleep(0.2)
        grip()
        time.sleep(0.2)

        # ✅ 다시 위로 복귀
        movel(posx(0, 0, 100, 0, 0, 0), vel=VELOCITY, acc=ACC, mod=DR_MV_MOD_REL)
        time.sleep(0.2)

        return height


    def place_block(place_pos):
        movel(place_pos, vel=VELOCITY, acc=ACC)
        time.sleep(0.2)
        release()
        time.sleep(0.2)
        movel(place_pos, vel=VELOCITY, acc=ACC)

    set_tool("Tool Weight_2FG")
    set_tcp("2FG_TCP")
    JReady = posj([0.0, 0.0, 90.0, 0.0, 90.0, 0.0])
    movej(JReady, vel=80, acc=80)
    release()
    grip()

    short_blocks, mid_blocks, tall_blocks = [], [], []

    print("=== 블럭 감지 및 그 자리에서 집기 시작 ===")
    for idx, pos in enumerate(position_lst):
        print(f"[{idx+1}/9] 감지 위치 이동")
        movel(pos, vel=VELOCITY, acc=ACC, ref=DR_BASE)
        height = check_and_grab()


        if height == 1:
            short_blocks.append(pos)
        elif height == 2:
            mid_blocks.append(pos)
        elif height == 3:
            tall_blocks.append(pos)

        movel(pos, vel=VELOCITY, acc=ACC, ref=DR_BASE)  # 다시 원위치

    print("=== 분류 완료: short/mid/tall ===")
    print(f"short: {len(short_blocks)}, mid: {len(mid_blocks)}, tall: {len(tall_blocks)}")

    def sort_and_place(block_list, pallet_list, label):
        for i in range(min(len(block_list), 3)):
            movej(JReady, vel=30, acc=30)
            movel(block_list[i], vel=VELOCITY, acc=ACC)
            place_block(pallet_list[i])
            print(f"{label} 블럭 {i+1} 정렬 완료")

    sort_and_place(short_blocks, short_pallets, "short")
    sort_and_place(mid_blocks, mid_pallets, "mid")
    sort_and_place(tall_blocks, tall_pallets, "tall")

    print("✅ 모든 블럭 정렬 완료")
    rclpy.shutdown()

if __name__ == "__main__":
    main()
