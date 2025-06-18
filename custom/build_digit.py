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

height = 100

# location : lego
# coordinate=   posx(X      ,Y          ,Z              ,RX     ,RY     ,RZ     )
block_short =   posx(379.32 ,192.52     ,233.65+height  ,98.77  ,179.74 ,65.97  )
block_long  =   posx(294.65 ,15.92      ,271.62+height  ,112.58 ,-180   ,24.53  )

# location : row 1, column 1 (1)
step_1      =   posx(402.29 ,109.11     ,265.47+height  ,11.76  ,-180   ,-169.14)
# location : row 1, column 2 (2)
step_2      =   posx(481.74 ,108.88     ,264.7+height   ,21.18  ,180    ,-159.4 )
# location : row 1, column 3 (3)
step_3      =   posx(562.35 ,106.4      ,265.4+height   ,8.67   ,-179.99,-171.79)

# location : row 2, column 1 (4)
step_4      =   posx(401.63 ,-1.98      ,265+height     ,10.15  ,180    ,-170.17)
# location : row 2, column 2 (5)
step_5      =   posx(481.21 ,-3.48      ,265.03+height  ,10.73  ,-179.99,-169.53)
# location : row 2, column 3 (6)
step_6      =   posx(559.87 ,-4.97      ,265.04+height  ,11.31  ,180    ,-168.84)

# location : row 3, column 1 (7)
step_7      =   posx(400.03 ,-113.39    ,265.83+height  ,95.99  ,-180   ,-83.73 )
step_7_2    =   posx(398.95 ,-146.82    ,264.12+height  ,158.15 ,179.99 ,-20.71 )
step_7_3    =   posx(398.08 ,-162.29    ,266.43+height  ,158.9  ,-180   ,-20.4  )
# location : row 3, column 2 (8)
step_8      =   posx(479.09 ,-115.1     ,263.54+height  ,155.72 ,-179.95,-24.07 )
step_8_2    =   posx(477.79 ,-146.42    ,267.02+height  ,169.59 ,-180   ,-9.28  )
step_8_3    =   posx(478.44 ,-163.88    ,265.16+height  ,2.79   ,-180   ,-176.94)
# location : row 3, column 3 (9)
step_9      =   posx(558.16 ,-115.92    ,266.15+height  ,158.13 ,-179.89,-22.01 )
step_9_2    =   posx(558.19 ,-148.61    ,267.12+height  ,165.73 ,180    ,-13.31 )
step_9_3    =   posx(556.44 ,-163.38    ,264.77+height  ,171.27 ,-180   ,-8.71  )

# difference
# Just to go up
block_to_down = posx(0,0,-height,0,0,0)
block_for_spread = posx(0,0,30,0,0,0)

step_for_2  =   posx(-1     ,-33        ,0              ,0      ,0      ,0      )
step_for_3  =   posx(-1     ,-48        ,0              ,0      ,0      ,0      )
options = 0


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
            get_digital_output,
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
            DR_AXIS_Z,
            DR_BASE,
            DR_MV_MOD_REL,
            DR_FC_MOD_REL

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

        # REQUESTED BUILDING CONSTRUCTIONS CHECK 
        if buildings == 0:
            print(f"{idx} 단계에 요청받은 건설 의뢰가 존재하지 않아 다음 단계로 넘어갑니다.")
            continue

        elif buildings == 1 or buildings == 2:
            block_for_construction = block_short if buildings == 1 else block_long
            print(f"{idx}번 위치에 {buildings}타입 4층 탑 건설 시작.")


            # REPEAT UNTIL 4TH FLOOR CONSTRUCTED.
            for floor in range(4):

                # GET BLOCKS FOR CONSTRUCTION
                movel(block_for_construction, vel = VELOCITY, acc = ACC, ref=DR_BASE) # move to sourcing site 
                mwait(0.5)
                movel(block_to_down, vel = VELOCITY, acc = ACC, mod=DR_MV_MOD_REL) # move down to pick up blocks
                mwait(0.5)
                grip()
                mwait(0.5)

                dummy = posx([395, -8, 240, 90, -120, 90])

                ##################################################################################
                while True:
                    di1 = get_digital_input(1)
                    di2 = get_digital_input(2)
                    di3 = get_digital_input(3)

                    # 1. 그립 실패: [0,1,0]
                    if [di1, di2, di3] == [1,0,0]:
                        release()
                        user_input = input("❌ 잡기 실패. 다시 시도하려면 'start' 입력: ")
                        if user_input == "start":
                            movel(block_for_construction, vel=VELOCITY, acc=ACC, ref=DR_BASE)
                            mwait(0.5)
                            grip()
                            mwait(0.5)
                            movel([0, 0, 100, 0, 0, 0], vel=VELOCITY, acc=ACC, ref=DR_BASE, mod=DR_MV_MOD_REL)
                            continue
                        else:
                            print("⛔ 사용자 중단")
                            break

                    # 2. 비정상 블록 감지: [0,0,1]
                    elif [di1, di2, di3] == [0,0,1]:
                        movel(dummy, vel=VELOCITY, acc=ACC, ref=DR_BASE)
                        release()
                        user_input = input("⚠️ 비정상 블록 감지됨. 다시 시도하려면 'start' 입력: ")
                        if user_input == "start":
                            movel(block_for_construction, vel=VELOCITY, acc=ACC, ref=DR_BASE)
                            mwait(0.5)
                            grip()
                            mwait(0.5)
                            movel([0, 0, 100, 0, 0, 0], vel=VELOCITY, acc=ACC, ref=DR_BASE, mod=DR_MV_MOD_REL)
                            continue
                        else:
                            print("⛔ 사용자 중단")
                            break

                    # 3. 정상 블록 감지: [0,1,1]
                    elif [di1, di2, di3] == [0,1,1]:
                        print("✅ 정상 블록 감지 완료!")
                        break

                    # 4. 예외 처리: 예상치 못한 조합
                    else:
                        print(f"⚠️ 예외 상태: 다시 측정 중...")
                        mwait(0.2)  # 잠시 대기 후 다시 측정
                        continue

                ##################################################################################

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
                movel(block_for_spread, vel = VELOCITY, acc = ACC)
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


        elif buildings == 3:
            # 층별/블록별 건설 설계도
            _3type_building = [[3, 3], [2, 2, 2], [3, 3], [2, 2, 2]]
            
            # 한 층씩 건설
            for fl, floor_plan in enumerate(_3type_building):
                # 한 층의 블록들을 순서대로 건설
                for block_idx, block_type in enumerate(floor_plan):
                    
                    # 사용할 블록과 옆으로 놓기 위한 상대 좌표 결정
                    if block_type == 2:
                        block_for_construction = block_short
                        relative_offset = step_for_2
                    else:
                        block_for_construction = block_long
                        relative_offset = step_for_3
                    
                    # 블록 집기 동작
                    movel(block_for_construction, vel=VELOCITY, acc=ACC)
                    mwait(0.5)
                    movel(block_to_down, vel=VELOCITY, acc=ACC, mod=DR_MV_MOD_REL)
                    mwait(0.5)
                    grip()
                    mwait(0.5)
                    movel(block_for_construction, vel=VELOCITY, acc=ACC)
                    mwait(0.5)

                    # 블록 놓을 최종 위치 계산
                    final_placement_pose = block_to_place[:]
                    if block_idx > 0:
                        final_placement_pose[0] += relative_offset[0] * block_idx
                        final_placement_pose[1] += relative_offset[1] * block_idx
                    
                    # 계산된 위치에 블록 조립 (두 번 누르기 로직)
                    movel(final_placement_pose, vel=VELOCITY, acc=ACC)
                    task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
                    time.sleep(1)
                    set_desired_force(fd=[0, 0, -30, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
                    time.sleep(1)
                    force_condition = check_force_condition(DR_AXIS_Z, max=30)
                    while (force_condition == 0):
                        force_condition = check_force_condition(DR_AXIS_Z, max=30)
                    release_force()
                    release_compliance_ctrl()
                    release()
                    mwait(0.2)
                    movel(final_placement_pose, vel=VELOCITY, acc=ACC)
                    mwait(0.2)
                    grip()
                    time.sleep(1)
                    task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
                    time.sleep(1)
                    set_desired_force(fd=[0, 0, -50, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
                    time.sleep(1)
                    force_condition = check_force_condition(DR_AXIS_Z, max=40)
                    while (force_condition == 0):
                        force_condition = check_force_condition(DR_AXIS_Z, max=40)
                    release_force()
                    release_compliance_ctrl()
                    mwait(0.2)
                    movel(final_placement_pose, vel=VELOCITY, acc=ACC)
                    mwait(0.2)
                    release()
        print(f"{idx} 단계 완료, 다음 단계로 넘어갑니다.")

    rclpy.shutdown()

if __name__ == "__main__":
    main()
