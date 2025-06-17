# Brik Assemble @20250515
# construction_step2_Jay
import rclpy
import DR_init
import time
import sys


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
##### POSE DEFINITION ####
# position for lego should be written 100 upward (Z + 100 from original position)

# location : home 
pos_home = posj(0, 0, 90, 0, 90, 0) # home position
height = 100

# 레고 좌표
# 좌표       =   posx(X      ,Y          ,Z              ,RX     ,RY     ,RZ     )
block_short =   posx(379.32 ,192.52     ,233.65+height  ,98.77  ,179.74 ,65.97  )
block_long  =   posx(294.65 ,15.92      ,271.62+height  ,112.58 ,-180   ,24.53  )

# 위치 : 행 1, 열 1 (1)
step_1      =   posx(402.29 ,109.11     ,265.47+height  ,11.76  ,-180   ,-169.14)
# 위치 : 행 1, 열 2 (2)
step_2      =   posx(481.74 ,108.88     ,264.7+height   ,21.18  ,180    ,-159.4 )
# 위치 : 행 1, 열 3 (3)
step_3      =   posx(562.35 ,106.4      ,265.4+height   ,8.67   ,-179.99,-171.79)

# 위치 : 행 2, 열 1 (4)
step_4      =   posx(401.63 ,-1.98      ,265+height     ,10.15  ,180    ,-170.17)
# 위치 : 행 2, 열 2 (5)
step_5      =   posx(481.21 ,-3.48      ,265.03+height  ,10.73  ,-179.99,-169.53)
# 위치 : 행 2, 열 3 (6)
step_6      =   posx(559.87 ,-4.97      ,265.04+height  ,11.31  ,180    ,-168.84)

# 위치 : 행 3, 열 1 (7)
step_7      =   posx(400.03 ,-113.39    ,265.83+height  ,95.99  ,-180   ,-83.73 )
step_7_2    =   posx(398.95 ,-146.82    ,264.12+height  ,158.15 ,179.99 ,-20.71 )
step_7_3    =   posx(398.08 ,-162.29    ,266.43+height  ,158.9  ,-180   ,-20.4  )
# 위치 : 행 3, 열 2 (8)
step_8      =   posx(479.09 ,-115.1     ,263.54+height  ,155.72 ,-179.95,-24.07 )
step_8_2    =   posx(477.79 ,-146.42    ,267.02+height  ,169.59 ,-180   ,-9.28  )
step_8_3    =   posx(478.44 ,-163.88    ,265.16+height  ,2.79   ,-180   ,-176.94)
# 위치 : 행 3, 열 3 (9)
step_9      =   posx(558.16 ,-115.92    ,266.15+height  ,158.13 ,-179.89,-22.01 )
step_9_2    =   posx(558.19 ,-148.61    ,267.12+height  ,165.73 ,180    ,-13.31 )
step_9_3    =   posx(556.44 ,-163.38    ,264.77+height  ,171.27 ,-180   ,-8.71  )

# Z값 100으로 내려가는 값 (레고블럭 잡기용/상대좌표)
block_to_down = posx(0,0,-height,0,0,0)
# Z값 30으로 올라가는 값 (Force로 레고블럭 누르기 이전에 사용/상대좌표)
block_for_spread = posx(0,0,30,0,0,0)

step_for_2  =   posx(-1     ,-33        ,0              ,0      ,0      ,0      )
step_for_3  =   posx(-1     ,-48        ,0              ,0      ,0      ,0      )



# Function Definition
def grip():
    set_digital_output(1, 1)
    set_digital_output(2, 0)
    time.sleep(0.5)
  
       
def release():
    set_digital_output(1, 0)
    set_digital_output(2, 1)
    time.sleep(0.5)

def build_typeA():
    pass

def build_typeB():
    pass

def build_typeC():
    pass

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
                print(f"{idx} 단계에 요청받은 건설 의뢰가 존재하지 않아 다음 단계로 넘어갑니다.")
                continue

            elif buildings == 1 or buildings == 2:
                block_for_construction = block_short if buildings == 1 else block_long
                print(f"{idx}번 위치에 {buildings}타입 4층 탑 건설 시작.")

                # 4층 건설까지 반복.
                for floor in range(4):

                    # 레고 블럭 좌표로 이동
                    movel(block_for_construction, vel = VELOCITY, acc = ACC)
                    mwait(0.5)
                    movel(block_to_down, vel = VELOCITY, acc = ACC, mod=DR_MV_MOD_REL)
                    mwait(0.5)
                    grip()
                    mwait(0.5)
                    movel(block_for_construction, vel = VELOCITY, acc = ACC)
                    mwait(0.5)

                    # 레고블럭 쌓을 위치로 이동
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
                    movel(block_for_spread, vel = VELOCITY, acc = ACC, mod=DR_MV_MOD_REL)
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


    rclpy.shutdown()
if __name__ == "__main__":
    main()
