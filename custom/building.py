import rclpy
import DR_init
import time
from copy import deepcopy
from DR_common2 import posx
import sys

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

build_list = []

# location : lego
block_short = posx(379.32, 192.52, 233.65+100, 98.77, 179.74, 65.97)
block_long = posx(294.65, 15.92, 271.62, 112.58, -180, 24.53)

# location : row 1, column 1 (1)
step_1 = posx(402.29, 109.11, 265.47, 11.76, -180, -169.14)
# location : row 1, column 2 (2)
step_2 = posx(481.74, 108.88, 264.7, 21.18, 180, -159.4)
# location : row 1, column 3 (3)
step_3 = posx(562.35, 106.4, 265.4, 8.67, -179.99, -171.79)

# location : row 2, column 1 (4)
step_4 = posx(401.63, -1.98, 265, 10.15, 180, -170.17)
# location : row 2, column 2 (5)
step_5 = posx(481.21, -3.48, 265.03, 10.73, -179.99, -169.53)
# location : row 2, column 3 (6)
step_6 = posx(559.87, -4.97, 265.04, 11.31, 180, -168.84)

# location : row 3, column 1 (7)
step_7 = posx(400.03, -113.39, 265.83, 95.99, -180, -83.73)
# location : row 3, column 2 (8)
step_8 = posx(479.09, -115.1, 263.54, 155.72, -179.95, -24.07)
# location : row 3, column 3 (9)
step_9 = posx(558.16, -115.92, 266.15, 158.13, -179.89, -22.01)

# difference
# Just to go up
block_to_down = posx(0,0,-100,0,0,0)

# etc




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
            amove_periodic,
            mwait,
            DR_AXIS_Z,
            DR_BASE,
            DR_MV_MOD_REL,
            DR_FC_MOD_REL,
            DR_TOOL

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
                print(x)
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
        
    def cement():
        movel(cup_up, vel = VELOCITY, acc = ACC) # 컵 위로
        mwait(0.2)
        movel(cup_down, vel = VELOCITY, acc = ACC) # 내려가기
        mwait(0.2)
        grip()
        mwait(0.2)
        movel(cup_up, vel = VELOCITY, acc = ACC) # 컵 위로
        movej(JReady, vel=VELOCITY, acc=ACC) # 섞는 위치 이동하기
        amove_periodic(amp=example_amp,period=1.0, atime=0.02, repeat=3, ref=DR_TOOL) # 흔들기
        mwait(5)
        movel(block_to_place, vel = VELOCITY, acc = ACC) # 붓는 장소 이동
        movej(posj([0,0,0,0,45,0]),vel=VELOCITY, acc=ACC, ref=DR_MV_MOD_REL) # 기울이기
        # movel() # 붓기
        movel(cup_up, vel = VELOCITY, acc = ACC) # 컵 위로
        mwait(0.2)
        movel(cup_down, vel = VELOCITY, acc = ACC) # 내려가기
        mwait(0.2)
        release()

    set_tool("Tool Weight_2FG")
    set_tcp("2FG_TCP")
    
    # 포즈 생성
    JReady = posj([0.0, 0.0, 90.0, 0.0, 90.0, 0.0])
    # 로봇 위치 초기화
    release()
    movej(JReady, vel = 80, acc = 80)
    print('start check blue print')
    grip()
     # 시작
    print('측정 시작')
    for idx, check_list in enumerate(position_lst):
        
        print(f"Moving to approach pose: {idx}")
        movel(check_list, vel=150, acc=300, ref=DR_BASE)
        # 팔레트 위치 측정
        check_bar()    
        movel(check_list, vel=150, acc=300, ref=DR_BASE)
    # 측정 결과 출력
    print('측정 완료')
    print(build_list)


    # 건설 시작
    VELOCITY, ACC = 100, 100
    for idx,buildings in enumerate(build_list):
    
        # initialization
        movej(JReady, vel = 30, acc = 30)
        release()
        mwait(0.5)
        
        # REQUESTED BUILDING CONSTRUCTIONS CHECK 
        if buildings == 0:
            print(f"{idx} 단계에 요청받은 건설 의뢰가 존재하지 않아 다음 단계로 넘어갑니다.")
            idx += 1
            continue

        elif buildings == 1:
            block_for_construction = block_short

        elif buildings == 2:
            block_for_construction = block_long

        elif buildings == 3:
            pass

        else:
            print("Error exists!")
            sys.exit()

        # POSITION CHECK FOR BUILDING CONSTRUCTION
        idx+=1
        if idx == 1 :
            block_to_place = step_1
        elif idx == 2 :
            block_to_place = step_2
        elif idx == 3 :
            block_to_place = step_3
        elif idx == 4 :
            block_to_place = step_4
        elif idx == 5 :
            block_to_place = step_5
        elif idx == 6 :
            block_to_place = step_6
        elif idx == 7 :
            block_to_place = step_7
        elif idx == 8 :
            block_to_place = step_8
        elif idx == 9 :
            block_to_place = step_9
        else:
            print("Error exists!")
            sys.exit()

        # REPEAT UNTIL 4TH FLOOR CONSTRUCTED.
        for floor in range(4):

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

            JReady = posj([0, 0, 90, 0, 90, 0])
            example_amp = [100.0, 100.0, 0.0, 0.0, 0.0, 30.0]
            cup_up = posx(468.11, 240.77, 375.27, 166.06, -179.88, 142.05)
            cup_down = posx(467.59, 243.72, 302.02, 45.37, -179.7, 21.17)

            # Cement Performance
            if floor == 0:
                cement()

            print(f"{idx} 단계 완료, 다음 단계로 넘어갑니다.")

    rclpy.shutdown()

if __name__ == "__main__":
    main()
