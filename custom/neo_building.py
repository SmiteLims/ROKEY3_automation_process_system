# Brik Assemble @20250515
# construction_step2_Jay
import rclpy
import DR_init
import time
import sys
import ast

# Configuration for a single robot
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 60, 60
ON, OFF = 1, 0

# Initialize DR_init with robot parameters
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

rclpy.init()
node = rclpy.create_node("brick_assemble_node", namespace=ROBOT_ID)
DR_init.__dsr__node = node

# DSR API import
try:
    from DSR_ROBOT2 import (
        get_digital_input, set_digital_output,
        get_current_posx, trans,set_ref_coord,
        set_tool, set_tcp,
        movej, movel, set_stiffnessx,amove_periodic, 
        wait, mwait,
        task_compliance_ctrl, release_compliance_ctrl,
        set_desired_force, release_force, check_force_condition,
        DR_BASE,DR_MVS_VEL_CONST, DR_TOOL,
        DR_FC_MOD_REL,
        DR_AXIS_Z,
        DR_MV_MOD_REL)
    
    from DR_common2 import posx, posj

except ImportError as e:
    print(f"Error importing DSR_ROBOT2 : {e}")
    exit()

with open('/home/rokey/ros_ws/building_text/build_list.txt', 'r', encoding='utf-8') as file:
    build_list_str = file.read()
    build_list = ast.literal_eval(build_list_str)

    print(f'현재 설계도면: {build_list}')

with open('/home/rokey/ros_ws/building_text/blue_print.txt', 'r', encoding='utf-8') as file:
    i = int(file.read())
    print(f'측정중인 공간: {i}')
# Global variable
##### 좌표 정의 ####
######### 설계도면 좌표 ##########
blueprint_xy1 = posx(194.38, -60.5, 150.0, 152.35, 180, 143.21)     #0
blueprint_xy2 = posx(244.83, -53.04, 150.0, 166.55, 180, 157.29)    #1
blueprint_xy3 = posx(295.89, -45.59, 150.0, 161, -180, 151.96)      #2
blueprint_xy4 = posx(201.67, -111.3, 150.0, 144.15, 180, 135.19)    #3
blueprint_xy5 = posx(252.36, -103.68, 150.0, 141.41, -180, 132.02)  #4
blueprint_xy6 = posx(302.79, -95.66, 150.0, 165.55, 180, 155.94)    #5
blueprint_xy7 = posx(209.64, -161.45, 150.0, 139.01, 180, 129.42)   #6
blueprint_xy8 = posx(259.82, -154.17, 150.0, 153.75, 179.96, 144)   #7
blueprint_xy9 = posx(310.27, -146.5, 150.0, 20.05, -179.96, 10.52)  #8
position_lst = [blueprint_xy1, blueprint_xy2, blueprint_xy3,
                blueprint_xy4, blueprint_xy5, blueprint_xy6,
                blueprint_xy7, blueprint_xy8, blueprint_xy9]
######### 건설좌표 ##########
# 위치 : 홈
pos_home            =   posj(0      ,0          ,90         ,0      ,90     ,0      )

# 높이 값
height = 250

# 레고 좌표
# 좌표               =   posx(X      ,Y          ,Z          ,RX     ,RY     ,RZ     )
block_short         =   posx(299.07, 89.08, 40.93+100, 13.84, 180, 105.56)
block_long          =   posx(299.51, 14.82, 41.22+100, 9.72, 180, 101.8)

# 위치 : 행 1, 열 1 (#1)
step_1              =   posx(401.43, 114.9, 136.12, 69, 179.97, 68.92)
# 위치 : 행 1, 열 2 (#2)
step_2              =   posx(480.75, 114.37, 134.9, 3.87, 180, 4.17)
# 위치 : 행 1, 열 3 (#3)
step_3              =   posx(561.21, 113.93, 134.97, 10.35, 180, 10.71)
# 위치 : 행 2, 열 1 (#4)
step_4              =   posx(400.75, 2.88, 134.68, 178.8, 180, 179.27)
# 위치 : 행 2, 열 2 (#5)
step_5              =   posx(480.54, 2.21, 134.63, 17.65, -180, 18.13)
# 위치 : 행 2, 열 3 (#6)
step_6              =   posx(560.57, 1.85, 135.43, 169.21, -180, 169.69) #5

# 위치 : 행 3, 열 1 (#7)
step_7              =   posx(399.66, -108.28, 136.19, 160.73, -180, 161.28)  #6
# 위치 : 행 3, 열 2 (#8)
step_8              =   posx(479.99, -108.84, 133.95, 167.52, -180, 167.91)  #7
# 위치 : 행 3, 열 3 (#9)
step_9              =  posx(559.68, -109.54, 134.8, 146.96, -180, 147.18)   #8


# Z값 100으로 내려가는 값 (레고블럭 잡기용/상대좌표)
block_to_down       =   posx(0      ,0          ,-100    ,0      ,0      ,0      )
# Z값 30으로 올라가는 값 (Force로 레고블럭 누르기 이전에 사용/상대좌표)
block_for_spread    =   posx(0      ,0          ,30         ,0      ,0      ,0      )
# 레고길이 값 (building 3 전용/상대좌표)
offset_for_short    =   posx(0     ,-31.9        ,0         ,0      ,0      ,0      )
offset_for_long     =   posx(0     ,-47.7        ,0         ,0      ,0      ,0      )

# 변수
# block_for_construction : 잡을 레고 위치 (+height)
# block_to_place : 레고 건축 위치 좌표
# relative_offset : 건축물 type 3용 상대좌표
# block_for_spread : 레고 조립 완벽 장착용


# Function Definition
def grip():
    set_digital_output(1, 1)
    set_digital_output(2, 0)
    time.sleep(0.5)
  
       
def release():
    set_digital_output(1, 0)
    set_digital_output(2, 1)
    time.sleep(0.5)

def check_bar(idx):
    task_compliance_ctrl()
    set_stiffnessx([3000.00, 3000.00, 3000.00, 200.00, 200.00, 200.00],time=0.0)
    set_desired_force([0.00, 0.00, -30.00, 0.00, 0.00, 0.00],[0,0,1,0,0,0],time=0.0)
    # 긴거: 3, 중간거: 2, 짧은거: 1, 없는거 0 
    while True:
        if check_force_condition(axis=DR_AXIS_Z, max=25):
            x = get_current_posx()[0]
            print(f'측정위치 : {x}')
            if x[2] >= 64:
                build_list[idx] = 3
            elif x[2] >= 54:
                build_list[idx] = 2
            elif x[2] >= 44:
                build_list[idx] = 1
            else:
                build_list[idx] = 0
            break         
    release_force(time=0.0)
    release_compliance_ctrl()

def cement():
    # step 1 truck move
    set_ref_coord(DR_BASE) 
    movel(cup_up, vel = VELOCITY, acc = ACC) # 컵 위로 이동
    mwait(0.1)
    release()
    mwait(0.1)
    movel(cup_down, vel = VELOCITY, acc = ACC) # 컵 아래로 이동
    mwait(0.1)
    grip() # 잡은 후 
    mwait(0.1)
    movel(cup_up, vel = VELOCITY, acc = ACC) # 컵 위로

    
    # step 2 mixing
    movej(JReady, vel=VELOCITY, acc=ACC) # 섞는 위치 이동하기
    amove_periodic(amp=example_amp,period=3.0, atime=0.02, repeat=2, ref=DR_TOOL) # 흔들기
    mwait(0.1)
    # movej(JStart, vel=VELOCITY, acc=ACC)


    # step 3 pavement
    movesx(xlist,vel=[100,30],acc=[200,60],vel_opt=DR_MVS_VEL_CONST)
    mwait(0.1)

    # step 4 organization
    movel(cup_up, vel = VELOCITY, acc = ACC) # 컵 위로
    mwait(0.1)
    movel(cup_down, vel = VELOCITY, acc = ACC) # 내려가기
    mwait(0.1)
    release()
    mwait(0.1)
    movel(cup_up, vel = VELOCITY, acc = ACC)
    mwait(0.1)


def main(args=None):

    set_tool("Tool Weight_2FG")
    set_tcp("2FG_TCP")

    if rclpy.ok():

        movej(pos_home, vel = 80, acc = 80)
        release()
        mwait(0.1)
        grip()
        mwait(0.2)
        print('초기위치 설정 완료')
        
        # 시작
        print('측정 시작')
        if i >= 9:
            print('이미 모든 위치 측정 완료')
            print(build_list)
        else:
            for idx in range(i, len(position_lst)):
                
                try:
                    print(f'{idx}위치 측정 시작')
                    movel(position_lst[idx], vel=150, acc=300, ref=DR_BASE)
                    # 팔레트 위치 측정
                    check_bar(idx)
                    movel(position_lst[idx], vel=150, acc=300, ref=DR_BASE)
                    with open('/home/rokey/ros_ws/building_text/blue_print.txt', 'w', encoding='utf-8') as file:
                        file.write(str(idx+1))
                    with open('/home/rokey/ros_ws/building_text/build_list.txt', 'w', encoding='utf-8') as file:
                        file.write(str(build_list))    
                except Exception as e:
                    print(f"[ERROR] {idx} 위치 이동 실패: {e}")
            # 측정 결과 출력
            print('측정 완료')
            print(build_list)
        
        for idx,buildings in enumerate(build_list):
            
            # initialization
            movej(pos_home, vel = 30, acc = 30)
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
            

            # 팔레트 높이로 체크한 빌딩 단계 체크
            if buildings == 0:
                print(f"{idx} 단계에 요청받은 건설 의뢰가 존재하지 않아 다음 단계로 넘어갑니다.")
                continue

            elif buildings == 1 or buildings == 2:
                
                # buildings가 1이면 
                block_for_construction = block_short if buildings == 1 else block_long
                print(f"{idx}번 위치에 {buildings}타입 4층 탑 건설 시작.")

                # 4층 건설까지 반복.
                for floor in range(4):

                    # 레고 블럭 좌표로 이동
                    movel(block_for_construction, vel = VELOCITY, acc = ACC)
                    print("1111")
                    mwait(0.5)
                    movel(block_to_down, vel = VELOCITY, acc = ACC, mod=DR_MV_MOD_REL)
                    print("w222")
                    mwait(0.5)
                    grip()
                    mwait(0.5)
                    dummy = posx([395, -8, 240, 90, -120, 90])

                ##################################################################################
                    while True:
                        di1 = get_digital_input(1)
                        di2 = get_digital_input(2)
                        di3 = get_digital_input(3)
                        print(f'{floor+1}층 진행중')
                        # 1. 그립 실패: [0,1,0]
                        if [di1, di2, di3] == [1,0,0]:
                            release()
                            user_input = input("❌ 잡기 실패. 다시 시도하려면 'start' 입력: ")
                            if user_input == "start":
                                movel(block_for_construction, vel=VELOCITY, acc=ACC, ref=DR_BASE)
                                mwait(0.5)
                                movel(block_to_down, vel=VELOCITY, acc=ACC, ref=DR_BASE, mod=DR_MV_MOD_REL)
                                mwait(0.5)
                                grip()
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
                            print(f"⚠️ 측정중")
                            time.sleep(3)  # 잠시 대기 후 다시 측정
                            continue
                    movel(block_for_construction, vel = VELOCITY, acc = ACC)
                    print("33")
                    mwait(0.5)

                    # 레고블럭 쌓을 위치로 이동
                    movel(block_to_place, vel = VELOCITY, acc = ACC)
                    print("41")

                    # Force 설정
                    task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
                    time.sleep(1)
                    print("5551")
                    set_desired_force(fd=[0, 0, -50, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
                    print("166666")
                    time.sleep(1)
                    force_condition = check_force_condition(DR_AXIS_Z, max=40)
                    print("77777")

                    while (force_condition == 0): # 힘 제어로 블럭 놓기
                        force_condition = check_force_condition(DR_AXIS_Z, max=50) # 조건 만족하면 0 (ROS2에서)
                    print("18888")

                    release_force()
                    release_compliance_ctrl()
                    release()
                    mwait(0.2)
                    movel(block_for_spread, vel = VELOCITY, acc = ACC, mod=DR_MV_MOD_REL)
                    print("19991")
                    mwait(0.2)
                    grip()
                    time.sleep(1)
                    task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
                    time.sleep(1)
                    print("11010101")
                    set_desired_force(fd=[0, 0, -50, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
                    time.sleep(1)
                    force_condition = check_force_condition(DR_AXIS_Z, max=40)
                    while (force_condition == 0): # 힘 제어로 블럭 놓기
                        force_condition = check_force_condition(DR_AXIS_Z, max=50) # 조건 만족하면 0 (ROS2에서)
                    release_force()
                    release_compliance_ctrl()
                    mwait(0.2)
                    movel(block_to_place, vel = VELOCITY, acc = ACC)
                    mwait(0.2)
                    release()


            elif buildings == 3:
                # 층별/블록별 건설 설계도
                print(f"{idx}번 위치에 {buildings}타입 4층 탑 건설 시작.")
                _3type_building = [[3, 3], [2, 2, 2], [3, 3], [2, 2, 2]]
                
                # 한 층씩 건설
                for fl, floor_plan in enumerate(_3type_building):
                    # 한 층의 블록들을 순서대로 건설
                    for block_count, block_type in enumerate(floor_plan):
                        
                        # 사용할 블록과 옆으로 놓기 위한 상대 좌표 결정
                        block_for_construction = block_short if block_type == 2 else block_long
                        relative_offset = offset_for_short if block_type == 2 else offset_for_long
                        
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
                        if block_count > 0:
                            final_placement_pose[0] += relative_offset[0] * block_count
                            final_placement_pose[1] += relative_offset[1] * block_count
                        
                        # 계산된 위치에 블록 조립 (두 번 누르기 로직)
                        movel(final_placement_pose, vel=VELOCITY, acc=ACC)
                        task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
                        time.sleep(1)
                        set_desired_force(fd=[0, 0, -30, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
                        time.sleep(1)
                        force_condition = check_force_condition(DR_AXIS_Z, max=40)
                        while (force_condition == 0):
                            force_condition = check_force_condition(DR_AXIS_Z, max=50)
                        release_force()
                        release_compliance_ctrl()
                        release()
                        mwait(0.2)
                        movel(block_for_spread, vel = VELOCITY, acc = ACC, mod=DR_MV_MOD_REL)
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
