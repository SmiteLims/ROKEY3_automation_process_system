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

# location : lego
block_short = posx(379.32, 192.52, 233.65+100, 98.77, 179.74, 65.97)
# block_long = posx(294.65, 15.92, 271.62, 112.58, -180, 24.53)

# location : row 1, column 1 (1)
step_1 = posx(402.29, 109.11, 265.47, 11.76, -180, -169.14)
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

    set_tool("Tool Weight_2FG")
    set_tcp("2FG_TCP")
    
    VELOCITY, ACC = 100, 100
    JReady = posj([0, 0, 90, 0, 90, 0])

    # initialization
    movej(JReady, vel = 30, acc = 30)
    release()
    mwait(0.5)
    # POSITION CHECK FOR BUILDING CONSTRUCTION
    

    # REPEAT UNTIL 4TH FLOOR CONSTRUCTED.


    # GET BLOCKS FOR CONSTRUCTION
    movel(block_short, vel = VELOCITY, acc = ACC, ref=DR_BASE) # move to sourcing site 
    mwait(0.5)
    movel(block_to_down, vel = VELOCITY, acc = ACC, mod=DR_MV_MOD_REL) # move down to pick up blocks
    mwait(0.5)
    grip()
    mwait(0.5)
    movel(block_short, vel = VELOCITY, acc = ACC) # move up
    mwait(0.5)

    # CONSTRUCT
    movel(step_1, vel = VELOCITY, acc = ACC)

    task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
    set_desired_force(fd=[0, 0, -30, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
    force_condition = check_force_condition(DR_AXIS_Z, max=20)

    while (force_condition == 0): # 힘 제어로 블럭 놓기
        force_condition = check_force_condition(DR_AXIS_Z, max=20) # 조건 만족하면 0 (ROS2에서)
        
    release_force()
    release_compliance_ctrl()
    release()

    mwait(0.2)
    movel(step_1, vel = VELOCITY, acc = ACC)
    mwait(0.2)
    grip()
    mwait(0.2)

    task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
    mwait(0.2)
    set_desired_force(fd=[0, 0, -50, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
    mwait(0.2)
    force_condition = check_force_condition(DR_AXIS_Z, max=30)
    while (force_condition == 0): # 힘 제어로 블럭 놓기
        force_condition = check_force_condition(DR_AXIS_Z, max=30) # 조건 만족하면 0 (ROS2에서)

    release_force()
    release_compliance_ctrl()

    x, _ = get_current_posx()
    x[2] += 10
    movel(x, vel=150, acc=300, ref=DR_BASE)
    release()
    mwait(0.2)

    x[2] -= 25
    mwait(0.2)

    movel(x, vel=150, acc=300, ref=DR_BASE)
    grip()
    mwait(0.2)

    task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
    mwait(0.2)
    set_desired_force(fd=[10, 0, -5, 0, 20, 0], dir=[1, 0, 1, 0, 1, 0], mod=DR_MV_MOD_REL)
    mwait(0.1)


    while True: # 힘 제어로 블럭 놓기
        mwait(3)
        break

    release_force()
    release_compliance_ctrl()
    movel(posx(0,0,-100,0,0,0), vel=150, acc=300, ref=DR_TOOL,mod=DR_MV_MOD_REL)


# #################################################################################
#     cur_pose, _ = get_current_posx()          # 현재 TCP pose 받아옴
#     new_pose = cur_pose.copy()             # 값 복사해서 새 pose 만들기
#     new_pose[4] += 30                      # Z축 회전(Rz)에 30도 추가
#     movel(new_pose, vel=20, acc=20)        # 위치는 그대로, 방향만 바뀐 상태로 이동

#     mwait(0.2)
#     movel(step_1, vel = VELOCITY, acc = ACC)
#     mwait(0.2)
#     release()

#     JReady = posj([0, 0, 90, 0, 90, 0])
#     example_amp = [100.0, 100.0, 0.0, 0.0, 0.0, 30.0]
#     cup_up = posx(468.11, 240.77, 375.27, 166.06, -179.88, 142.05)
#     cup_down = posx(467.59, 243.72, 302.02, 45.37, -179.7, 21.17)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
