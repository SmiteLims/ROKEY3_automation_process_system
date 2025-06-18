# Brik Assemble @20250515
# construction_step2_Jay
import rclpy
import DR_init
import time
import sys


# Configuration for a single robot
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC, BLEND = 80, 80, 20
ON, OFF = 1, 0
# 힘제어 변수, FIRST는 처음 블럭 장착할 때, SECOND는 장착 확인용
FIRST, SECOND = 40, 50

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
        get_current_posx, trans,
        set_tool, set_tcp,
        movej, movel,
        wait, mwait,
        task_compliance_ctrl, release_compliance_ctrl,
        set_desired_force, release_force, check_force_condition,
        DR_BASE,
        DR_FC_MOD_REL,
        DR_AXIS_Z,
        DR_MV_MOD_REL)
    
    from DR_common2 import posx, posj

except ImportError as e:
    print(f"Error importing DSR_ROBOT2 : {e}")
    exit()

# Global variable
##### 좌표 정의 ####

# 위치 : 홈
pos_home            =   posj(0      ,0          ,90         ,0      ,90     ,0      )

# 높이 값
height = 120

# 레고 좌표
# 좌표               =   posx(X      ,Y          ,Z          ,RX     ,RY     ,RZ     )
block_short         =   posx(299.07 ,89.08      ,41+height  ,13.84  ,180    ,105.56 )
block_long          =   posx(299.51 ,14.82      ,41+height  ,9.72   ,180    ,101.8  )

# 위치 : 행 1, 열 1 (#1)
step_1              =   posx(401.43 ,114.9      ,35+height  ,69     ,180    ,68.92  )
# 위치 : 행 1, 열 2 (#2)
step_2              =   posx(480.75 ,114.37     ,35+height  ,3.87   ,180    ,4.17   )
# 위치 : 행 1, 열 3 (#3)
step_3              =   posx(555.21 ,113.93     ,35+height  ,10.35  ,180    ,10.71  )

# 위치 : 행 2, 열 1 (#4)
step_4              =   posx(400.75 ,2.88       ,35+height  ,178.8  ,180    ,179.27 )
# 위치 : 행 2, 열 2 (#5)
step_5              =   posx(480.54 ,2.21       ,35+height  ,17.65  ,180    ,18.13  ) 
# 위치 : 행 2, 열 3 (#6)
step_6              =   posx(555.57 ,1.85       ,35+height  ,169.21 ,180    ,169.69 )

# 위치 : 행 3, 열 1 (#7)
step_7              =   posx(399.66 ,-108.28    ,35+height  ,160.73 ,180    ,161.28 )
# 위치 : 행 3, 열 2 (#8)
step_8              =   posx(479.99 ,-108.84    ,35+height  ,167.52 ,180    ,167.91 )
# 위치 : 행 3, 열 3 (#9)
step_9              =   posx(555.68 ,-109.54    ,35+height  ,146.96 ,180    ,147.18 )


# Z값으로 내려가는 값 (레고블럭 잡기용/상대좌표)
block_to_down       =   posx(0      ,0          ,-height    ,0      ,0      ,0      )
# Z값으로 올라가는 값 (Force로 레고블럭 누르기 이전에 사용/상대좌표)
block_for_spread    =   posx(0      ,0          ,40         ,0      ,0      ,0      )
# 레고길이 값 (building 3 전용/상대좌표)
offset_for_short    =   posx(0     ,-31.9       ,0          ,0      ,0      ,0      )
offset_for_long     =   posx(0     ,-47.7       ,0          ,0      ,0      ,0      )

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

def main(args=None):

    set_tool("Tool Weight_2FG")
    set_tcp("2FG_TCP")


    build_list = [0,0,3,1,0,0,0,2,0] # 팔레트 높이에 따른 값 결과 리스트화

    if rclpy.ok():
        
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
                print(f"{idx}번에 요청받은 건설 의뢰가 존재하지 않아 다음 단계로 넘어갑니다.")
                continue

            elif buildings == 1 or buildings == 2:
                
                # buildings가 1이면 
                block_for_construction = block_short if buildings == 1 else block_long
                print(f"{idx}번 위치에 {buildings}타입 4층 탑 건설 시작.")

                # 4층 건설까지 반복.
                for floor in range(4):
                    print(f"{floor+1}4층 탑 건설 시작.")
                    # 레고 블럭 좌표로 이동
                    movel(block_for_construction, vel = VELOCITY, acc = ACC, radius=BLEND)
                    movel(block_to_down, vel = VELOCITY, acc = ACC, mod=DR_MV_MOD_REL)
                    mwait(0.5)
                    grip()
                    mwait(0.5)
                    movel(block_for_construction, vel = VELOCITY, acc = ACC)

                    # 레고블럭 쌓을 위치로 이동
                    movel(block_to_place, vel = VELOCITY, acc = ACC, radius=BLEND)


                    # Force 설정
                    task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
                    time.sleep(1)
                    set_desired_force(fd=[0, 0, -FIRST, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
                    time.sleep(1)
                    force_condition = check_force_condition(DR_AXIS_Z, max=FIRST)

                    while (force_condition == 0): # 힘 제어로 블럭 놓기
                        force_condition = check_force_condition(DR_AXIS_Z, max=FIRST) # 조건 만족하면 0 (ROS2에서)

                    release_force()
                    release_compliance_ctrl()
                    release()
                    mwait(0.2)
                    movel(block_for_spread, vel = VELOCITY, acc = ACC, mod=DR_MV_MOD_REL)
                    mwait(0.2)
                    grip()
                    task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
                    time.sleep(1)
                    set_desired_force(fd=[0, 0, -SECOND, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
                    time.sleep(1)
                    force_condition = check_force_condition(DR_AXIS_Z, max=SECOND)

                    while (force_condition == 0): # 힘 제어로 블럭 놓기
                        force_condition = check_force_condition(DR_AXIS_Z, max=SECOND) # 조건 만족하면 0 (ROS2에서)

                    release_force()
                    release_compliance_ctrl()
                    mwait(0.2)
                    movel(block_to_place, vel = VELOCITY, acc = ACC, radius=BLEND)
                    mwait(0.2)
                    release()


            elif buildings == 3:
                # 층별/블록별 건설 설계도
                _3type_building = [[3, 3], [2, 2, 2], [3, 3], [2, 2, 2]]
                
                # 한 층씩 건설
                for fl, floor_plan in enumerate(_3type_building):
                    print(f"{fl+1}4층 탑 건설 시작.")
                    # 한 층의 블록들을 순서대로 건설
                    for block_count, block_type in enumerate(floor_plan):
                        
                        # 사용할 블록과 옆으로 놓기 위한 상대 좌표 결정
                        block_for_construction = block_short if block_type == 2 else block_long
                        relative_offset = offset_for_short if block_type == 2 else offset_for_long
                        
                        # 블록 집기 동작
                        movel(block_for_construction, vel=VELOCITY, acc=ACC,radius=BLEND)
                        movel(block_to_down, vel=VELOCITY, acc=ACC, mod=DR_MV_MOD_REL)
                        mwait(0.5)
                        grip()
                        mwait(0.5)
                        movel(block_for_construction, vel=VELOCITY, acc=ACC)

                        # 블록 놓을 최종 위치 계산
                        final_placement_pose = block_to_place[:]
                        if block_count > 0:
                            final_placement_pose[0] += relative_offset[0] * block_count
                            final_placement_pose[1] += relative_offset[1] * block_count
                        
                        # 계산된 위치에 블록 조립 (두 번 누르기 로직)
                        movel(final_placement_pose, vel=VELOCITY, acc=ACC,radius=BLEND)


                        task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
                        time.sleep(1)
                        set_desired_force(fd=[0, 0, -FIRST, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
                        time.sleep(1)
                        force_condition = check_force_condition(DR_AXIS_Z, max=FIRST)

                        while (force_condition == 0):
                            force_condition = check_force_condition(DR_AXIS_Z, max=FIRST)

                        release_force()
                        release_compliance_ctrl()
                        release()
                        mwait(0.2)
                        movel(block_for_spread, vel = VELOCITY, acc = ACC, mod=DR_MV_MOD_REL)
                        mwait(0.2)
                        grip()
                        task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
                        time.sleep(1)
                        set_desired_force(fd=[0, 0, -SECOND, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
                        time.sleep(1)
                        force_condition = check_force_condition(DR_AXIS_Z, max=SECOND)

                        while (force_condition == 0):
                            force_condition = check_force_condition(DR_AXIS_Z, max=SECOND)

                        release_force()
                        release_compliance_ctrl()
                        mwait(0.2)
                        movel(final_placement_pose, vel=VELOCITY, acc=ACC, radius=BLEND)
                        mwait(0.2)
                        release()

            print(f"{idx} 단계 완료, 다음 단계로 넘어갑니다.")


    rclpy.shutdown()
if __name__ == "__main__":
    main()
