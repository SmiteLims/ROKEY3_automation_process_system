import rclpy
import DR_init
import time
import sys
from DR_common2 import posx, posj

ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 100, 100
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL
ON, OFF = 1, 0

# 블럭 감지 위치
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

# 상대적으로 아래로 이동
block_to_down = posx(0, 0, -10, 0, 0, 0)

# 팔레트 위치 (TODO: 실제 좌표 입력)
short_pallets = [posx(194.06, -212.82, 272.11, 136.89, 180, 135.3), posx(249.04, -208.6, 271.62, 131.21, 180, 129.54), posx(299.15, -204.28, 271.21, 125.49, 180, 123.58)]
mid_pallets   = [posx(198.61, -261.26, 270.73, 123.41, 179.99, 121.48), posx(253.2, -259.33, 270.37, 121.97, 179.99, 119.38), posx(301.92, -253.32, 270.1, 147.87, -180, 144.85)]
tall_pallets  = [posx(203.49, -311.72, 270.01, 116.34, 179.99, 110.69), posx(254.49, -307.87, 269.7, 159.93, -180, 153.76), posx(305.63, -304.16, 269.28, 144.86, -180, 138.41)]

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("rokey_block_sorter", namespace=ROBOT_ID)
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
        print(f"DSR_ROBOT2 import 오류: {e}")
        return

    def grip():
        set_digital_output(1, ON)
        set_digital_output(2, OFF)

    def release():
        set_digital_output(1, OFF)
        set_digital_output(2, ON)

    def check_and_grab():
        set_ref_coord(0)
        task_compliance_ctrl()
        set_stiffnessx([3000.0]*3 + [200.0]*3)
        set_desired_force([0.0, 0.0, -30.0, 0.0, 0.0, 0.0], [0, 0, 1, 0, 0, 0])

        while True:
            if check_force_condition(axis=DR_AXIS_Z, max=25):
                z = get_current_posx()[0][2]
                print(f"측정된 높이 Z: {z}")
                if z >= 290:
                    code = 3
                elif z >= 280:
                    code = 2
                elif z >= 270:
                    code = 1
                else:
                    code = 0
                break

        release_force()
        release_compliance_ctrl()
        release()
        mwait(0.2)
        movel(block_to_down, vel=VELOCITY, acc=ACC, mod=DR_MV_MOD_REL)
        mwait(0.2)
        grip()
        mwait(0.2)
        return code

    def pick_and_place(pick_pos, place_pos):
        movej(JReady, vel=30, acc=30)
        movel(pick_pos, vel=VELOCITY, acc=ACC)
        mwait(0.2)
        movel(place_pos, vel=VELOCITY, acc=ACC)
        mwait(0.2)
        release()
        mwait(0.2)
        movel(place_pos, vel=VELOCITY, acc=ACC)

    set_tool("Tool Weight_2FG")
    set_tcp("2FG_TCP")
    JReady = posj([0.0, 0.0, 90.0, 0.0, 90.0, 0.0])
    movej(JReady, vel=80, acc=80)
    release()

    short_blocks, mid_blocks, tall_blocks = [], [], []

    for idx, pos in enumerate(position_lst):
        print(f"[{idx+1}/9] 위치 감지 중...")
        movel(pos, vel=VELOCITY, acc=ACC)
        height_code = check_and_grab()

        if height_code == 1:
            short_blocks.append((idx, pos))
        elif height_code == 2:
            mid_blocks.append((idx, pos))
        elif height_code == 3:
            tall_blocks.append((idx, pos))
        else:
            print(f"{idx+1}번: 블럭 없음 (스킵)")
        movel(pos, vel=VELOCITY, acc=ACC)

    print(f"감지 완료: short={len(short_blocks)}, mid={len(mid_blocks)}, tall={len(tall_blocks)}")

    for i in range(3):
        if i < len(short_blocks):
            _, pick_pos = short_blocks[i]
            pick_and_place(pick_pos, short_pallets[i])
        if i < len(mid_blocks):
            _, pick_pos = mid_blocks[i]
            pick_and_place(pick_pos, mid_pallets[i])
        if i < len(tall_blocks):
            _, pick_pos = tall_blocks[i]
            pick_and_place(pick_pos, tall_pallets[i])

    print("✅ 모든 블럭 분류 완료")
    rclpy.shutdown()

if __name__ == "__main__":
    main()
