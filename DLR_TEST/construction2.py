# Brik Assemble @20250515

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

try:
        from DSR_ROBOT2 import (
            get_digital_input,
            set_digital_output
        )

except ImportError as e:
        print(f"Error importing DSR_ROBOT2 : {e}")
        sys,exit()

############ Grippper Grip & Rease ##############
def grip():

    set_digital_output(1, 1)
    set_digital_output(2, 0)
    # 그리퍼 닫힘
   
       
def release():

    set_digital_output(1, 0)
    set_digital_output(2, 1)
    # 그리퍼 열림


############ Main ##############
def main(args=None):


    try:
        from DSR_ROBOT2 import (
            set_tool,
            set_tcp,
            movej,
            movel,
            trans,
            mwait,
            task_compliance_ctrl,
            set_desired_force,
            check_force_condition,
            release_force,
            release_compliance_ctrl,
            get_tool_force,
            DR_FC_MOD_REL,
            DR_AXIS_Z,
            DR_BASE,
        )

        from DR_common2 import posx, posj

    except ImportError as e:
        print(f"Error importing DSR_ROBOT2 : {e}")
        return


    ##### POSE DEFINITION ####
    # location : home 
    pos_home = posj(0, 0, 90, 0, 90, 0)

    # position for lego should be written 100 upward (Z + 100 from original position)
    # location : row 2, column 1
    block_to_grip = posx(401.55, -31.29, 353.88, 0.43, -179.99, 3.66)

    # location : row 2, column 2
    block_to_place = posx(512.12, -34.26, 354.68, 12.66, 179.99, 16.25)

    # Just to go up
    block_to_down = posx(0,0,-100,0,0,0)

    # set_tool("Tool Weight_2FG")
    # set_tcp("2FG_TCP")

    while rclpy.ok():

        if(release_force() == 0):
            print("release force")
        mwait(0.5)
        if(release_compliance_ctrl() == 0) :
             print("release compliance ctrl")

        # 동작 전 그리퍼 열림 확인
        release()
        print("Gripper Release")

        # Home Pose
        movel(block_to_grip, vel = 30, acc = 30)
        # mwait(0.5)

        print("블럭 위로 이동")
        mwait(0.5)

        movel(block_to_down, vel = 30, acc = 30, mod=1) #블럭 위치로 이동
        print("블럭 그립 위치로 다운")
        mwait(0.5)
        grip() # 블럭 집기
        print("블럭 집기")
        mwait(0.5)
        movel(block_to_grip, vel = 30, acc = 30) #블럭 들어올리기
        print("블럭 들기")


        # Assemble_1
        movel(block_to_place, vel = 30, acc = 30)

        ret = task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
        if ret == 0:
             print("Compliance_ctrl Set")
        else:
             print("Compliance_ctrl failed!!")

        time.sleep(1)
        ret = set_desired_force(fd=[0, 0, -20, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
        if ret == 0:
             print("set_desired_force Set")
        else:
             print("set_desired_force Set Failed!!!!!")

        time.sleep(1)
        force_condition = check_force_condition(DR_AXIS_Z, max=30) 
        print("force_condition Start : ", force_condition)
        # print("get_tool_force", get_tool_force)

        while (force_condition > -1): # 힘제어로 블럭 놓기
            force_condition = check_force_condition(DR_AXIS_Z, max=10)
            # print("Force Check: ", force_condition)
        
        if(release_force() == 0):
            print("release force")
        time.sleep(0.1)
        if(release_compliance_ctrl() == 0) :
             print("release compliance ctrl")
        time.sleep(0.1)

        release()
        print("여긴가")
        mwait(0.5)
        print("저긴가")
        movel(block_to_place, vel = 30, acc = 30)


    rclpy.shutdown()
if __name__ == "__main__":
    main()
