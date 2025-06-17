import rclpy
import DR_init
import time
from copy import deepcopy
from DR_common2 import posx
import sys
import ast

# for single robot
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 30, 30

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

ON, OFF = 1, 0

# 좌표정의

nocontact_z = 350.0
blueprint_xy1 = posx(198.21,-58.48,nocontact_z,147.25,-179.99,145.78)
blueprint_xy2 = posx(249.95,-52.92,nocontact_z,11.30,-179.55,9.85)
blueprint_xy3 = posx(303.75,-50.58,nocontact_z,3.91,-179.05,2.20)
blueprint_xy4 = posx(204.68,-107.52,nocontact_z,6.47,-178.94,4.53)
blueprint_xy5 = posx(257.29,-105.63,nocontact_z,3.56,-178.59,1.44)
blueprint_xy6 = posx(311.57,-102.36,nocontact_z,179.50,177.77,176.98)
blueprint_xy7 = posx(214.47,-159.75,nocontact_z,171.63,177.15,169.30)
blueprint_xy8 = posx(267.19,-156.88,nocontact_z,173.03,176.69,170.36)
blueprint_xy9 = posx(320.45,-155.79,nocontact_z,174.63,176.23,171.84)
position_lst = [blueprint_xy1,
                        blueprint_xy2,
                        blueprint_xy3,
                        blueprint_xy4,
                        blueprint_xy5,
                        blueprint_xy6,
                        blueprint_xy7,
                        blueprint_xy8,
                        blueprint_xy9]


# location : lego
block_short = posx(379.32, 192.52, 233.65+100, 98.77, 179.74, 65.97)
block_long = posx(294.65, 15.92, 271.62+100, 112.58, -180, 24.53)

# location : row 1, column 1 (1)
step_1 = posx(402.29, 109.11, nocontact_z, 11.76, -180, -169.14)
# location : row 1, column 2 (2)
step_2 = posx(481.74, 108.88, nocontact_z, 21.18, 180, -159.4)
# location : row 1, column 3 (3)
step_3 = posx(562.35, 106.4, nocontact_z, 8.67, -179.99, -171.79)

# location : row 2, column 1 (4)
step_4 = posx(401.63, -1.98, nocontact_z, 10.15, 180, -170.17)
# location : row 2, column 2 (5)
step_5 = posx(481.21, -3.48, nocontact_z, 10.73, -179.99, -169.53)
# location : row 2, column 3 (6)
step_6 = posx(559.87, -4.97, nocontact_z, 11.31, 180, -168.84)

# location : row 3, column 1 (7)
step_7 = posx(400.03, -113.39, nocontact_z, 95.99, -180, -83.73)
# location : row 3, column 2 (8)
step_8 = posx(479.09, -115.1, nocontact_z, 155.72, -179.95, -24.07)
# location : row 3, column 3 (9)
step_9 = posx(558.16, -115.92, nocontact_z, 158.13, -179.89, -22.01)
# difference
# Just to go up
block_to_down = posx(0,0,-100,0,0,0)



with open('/home/rokey/ros_ws/building_text/build_list.txt', 'r', encoding='utf-8') as file:
    build_list_str = file.read()
    build_list = ast.literal_eval(build_list_str)

    print(f'저장된 설계도 리스트{build_list}')

with open('/home/rokey/ros_ws/building_text/blue_print.txt', 'r', encoding='utf-8') as file:
    i = int(file.read())
    print(f"설계도 진행 상황{i}")

with open('/home/rokey/ros_ws/building_text/construct_index.txt', 'r', encoding='utf-8') as file:
    index = int(file.read())
    print(f'건축 진행 상황{index}')


def main(args=None):
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
            mwait,
            DR_MV_MOD_REL,
            DR_FC_MOD_REL,
            DR_AXIS_Z,
            DR_BASE
        )

        from DR_common2 import posx, posj

    except ImportError as e:
        print(f"Error importing DSR_ROBOT2 : {e}")
        return
    
    def grip():
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
            if check_force_condition(axis=DR_AXIS_Z, max=25):
                x = get_current_posx()[0]
                print('측정 좌표 ',x)
                if x[2] >= 290:
                    build_list.append(3)
                elif x[2] >= 280:
                    build_list.append(2)
                elif x[2] >= 270:
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
    # 로봇 위치 초기화
    release()
    movej(JReady, vel = 80, acc = 80)
    grip()
     # 시작
    print('측정 시작')
    if i >= 9:
        print('설계완료')
        print(build_list)
    else:
        for idx in range(i, len(position_lst)):
            
            print(f"Moving to approach pose: {idx}")
            movel(position_lst[idx], vel=150, acc=300, ref=DR_BASE)
            # 팔레트 위치 측정
            check_bar()
            movel(position_lst[idx], vel=150, acc=300, ref=DR_BASE)
            with open('/home/rokey/ros_ws/building_text/blue_print.txt', 'w', encoding='utf-8') as file:
                file.write(str(idx+1))
            with open('/home/rokey/ros_ws/building_text/build_list.txt', 'w', encoding='utf-8') as file:
                file.write(str(build_list))    
        # 측정 결과 출력
        print('측정 완료')
        print(build_list)


    # 건설 시작
    VELOCITY, ACC = 100, 100
    if index >= 9:
            print('건축완료')
            with open('/home/rokey/ros_ws/building_text/construct_index.txt', 'w', encoding='utf-8') as file:
                file.write(str(0))
            with open('/home/rokey/ros_ws/building_text/blue_print.txt', 'w', encoding='utf-8') as file:
                file.write(str(idx+1))
    else:
        for idx in range(index,len(build_list)):
            print(f'{idx} 건설 시작')
            # initialization
            movej(JReady, vel = 30, acc = 30)
            release()
            mwait(0.5)
            
            # REQUESTED BUILDING CONSTRUCTIO    NS CHECK 
            block_map = {
                0: None, 
                1: block_short,
                2: block_long,
                3: None  # or None, if not needed
            }
            block_for_construction = block_map.get(build_list[idx])
            if block_for_construction is None:
                print("블럭 타입 없음 또는 잘못된 입력")
                continue


            # POSITION CHECK FOR BUILDING CONSTRUCTION
            
            step_map = {
                0: step_1, 1: step_2, 2: step_3,
                3: step_4, 4: step_5, 5: step_6,
                6: step_7, 7: step_8, 8: step_9
            }
            block_to_place = step_map.get(idx)
            if block_to_place is None:
                continue

            # REPEAT UNTIL 4TH FLOOR CONSTRUCTED.
            for floor in range(4):
                print(f'{floor +1}층')
                # GET BLOCKS FOR CONSTRUCTION
                movel(block_for_construction, vel = VELOCITY, acc = ACC, ref=DR_BASE) # move to sourcing site 
                mwait(0.5)
                movel(block_to_down, vel = VELOCITY, acc = ACC, mod=DR_MV_MOD_REL) # move down to pick up blocks
                mwait(0.5)
                grip()
                mwait(0.5)
                movel(block_for_construction, vel = VELOCITY, acc = ACC) # move up
                mwait(0.5)

                # CONSTRUCT
                movel(block_to_place, vel = VELOCITY, acc = ACC)

                task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
                time.sleep(1)
                set_desired_force(fd=[0, 0, -30, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
                time.sleep(1)
                force_condition = check_force_condition(DR_AXIS_Z, max=30)

                while (force_condition == 0): # 힘 제어로 블럭 놓기
                    force_condition = check_force_condition(DR_AXIS_Z, max=30) # 조건 만족하면 0 (ROS2에서)

                release_force()
                release_compliance_ctrl()
                release()
                mwait(0.2)
                movel(block_to_place, vel = VELOCITY, acc = ACC)
                mwait(0.2)
                grip()
                time.sleep(1)
                task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
                time.sleep(1)
                set_desired_force(fd=[0, 0, -50, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
                time.sleep(1)
                force_condition = check_force_condition(DR_AXIS_Z, max=40)
                while (force_condition == 0): # 힘 제어로 블럭 놓기
                    force_condition = check_force_condition(DR_AXIS_Z, max=40) # 조건 만족하면 0 (ROS2에서)
                release_force()
                release_compliance_ctrl()
                mwait(0.2)
                movel(block_to_place, vel = VELOCITY, acc = ACC)
                mwait(0.2)
                release()
            with open('/home/rokey/ros_ws/building_text/construct_index.txt', 'w', encoding='utf-8') as file:
                file.write(str(idx))
            
            print(f"{idx+1} 단계 완료, 다음 단계로 넘어갑니다.")

    rclpy.shutdown()

if __name__ == "__main__":
    main()
