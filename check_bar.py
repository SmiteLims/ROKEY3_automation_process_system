import rclpy
import DR_init
import time
from copy import deepcopy
from DR_common2 import posx

# for single robot
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 30, 30

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

ON, OFF = 1, 0

# 메인 함수
def main(args=None):
    build_list = []
    rclpy.init(args=args)
    node = rclpy.create_node("rokey_move", namespace=ROBOT_ID)

    DR_init.__dsr__node = node
    # DRL 라이브러리 import
    try:
        from DSR_ROBOT2 import (
            set_tool,
            set_tcp,
            movej,
            movel,
            set_digital_output,
            get_digital_input,
            get_current_posx,
            trans,
            set_ref_coord,
            task_compliance_ctrl,
            set_stiffnessx,
            set_desired_force,
            release_force,
            release_compliance_ctrl,
            check_force_condition,
            DR_AXIS_Z,
        )

        from DR_common2 import posx, posj

    except ImportError as e:
        print(f"Error importing DSR_ROBOT2 : {e}")
        return
    
    # 함수 정의
    def wait_digital_input(sig_num):
        while not get_digital_input(sig_num):
            time.sleep(0.5)
            print(f"Wait for digital input: {sig_num}")
            pass

    def graps():
        set_digital_output(1,ON)
        set_digital_output(2,OFF)

    def release():
        set_digital_output(2,ON)
        set_digital_output(1,OFF)

    def check_bar():
        set_ref_coord(0)
        task_compliance_ctrl()
        set_stiffnessx([3000.00, 3000.00, 3000.00, 200.00, 200.00, 200.00],time=0.0)
        set_desired_force([0.00, 0.00, -30.00, 0.00, 0.00, 0.00],[0,0,1,0,0,0],time=0.0)
        # 긴거: 3, 중간거: 2, 짧은거: 1, 없는거 0 
        while True:
            if check_force_condition(axis=DR_AXIS_Z, min=25, ref=None):
                x = get_current_posx()
                print(str(x))
                if x[2] >= 64:
                    build_list.append(3)
                elif x[2] >= 54:
                    build_list.append(2)
                elif x[2] >= 44:
                    build_list.append(1)
                else:
                    build_list.append(0)
                break         
        release_force(time=0.0)
        release_compliance_ctrl()

    set_tool("Tool Weight_2FG")
    set_tcp("2FG_TCP")
    
    # 포즈 생성
    JReady = posj([0.0, 0.0, 90.0, 0.0, 90.0, 0.0])
    pos_1_LEFT_UP = posx(196.42, -61.65, 38.82, 156.57, -178.87, 150.08)
    pos_1_LEFT_DOWN = posx(206.5, -163.96, 38.31, 127.41, -178.77, 121.08)
    pos_1_RIGHT_UP = posx(297.92, -51.09, 38.21, 131.14, -178.96, 124.32)
    pos_1_RIGHT_DOWN = posx(308.1, -153.93, 38.48, 108.29, -178.96, 101.89)

    pos_APPROACH_OFFSET = posx(0, 0, 100, 0, 0, 0)

    direction = 1 # 지그제그, 스네이크 설정
    row = 3
    column = 3
    stack = 1
    thickness = 0
    point_offset = [0,0,0]

    total_count = row * column * stack

    # 로봇 위치 초기화
    release()
    movej(JReady, vel = 80, acc = 80)
    graps()
     # 시작
    print('측정 시작')
    for idx in range(0,total_count):
        # get_pattern_point 찾아 주세요
        Pallet_Pose1 = get_pattern_point(pos_1_LEFT_UP, pos_1_RIGHT_UP, pos_1_LEFT_DOWN, pos_1_RIGHT_DOWN, idx, direction, row, column, stack, thickness, point_offset) 
        # 로봇 이동 위치 생성
        Approach_Pose1 = trans(Pallet_Pose1, pos_APPROACH_OFFSET)       
        # 팔레트 순서대로 접근
        print(f"Moving to approach pose: {Approach_Pose1}")
        movel(Approach_Pose1, vel=150, acc=300)
        # 팔레트 위치 측정
        check_bar()    
    # 측정 결과 출력
    print(build_list)

    rclpy.shutdown()

if __name__ == "__main__":
    main()