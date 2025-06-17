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
blueprint_status = False


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
            movesx,
            amove_periodic,
            move_spiral,
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
            DR_BASE,
            DR_TOOL,
            DR_MVS_VEL_CONST
        )

        from DR_common2 import posx, posj

    except ImportError as e:
        print(f"Error importing DSR_ROBOT2 : {e}")
        return
    
    
    JReady = posj([0, 0, 90, 0, 90, 0])
    JStart = posj([45.86,35.85,78.57,-34.90,105.14,48.98])
    example_amp = [150.0, 0, 0.0, 0.0, 0.0, 30.0]
    cup_up = posx(457.41, 268.36, 196.67, 104.04, 179.98, 103.65)
    cup_down = posx(456.61, 267.81, 73.48, 16.53, -179.87, 16.16)
    # target_point1 = posx(398.74, 169.38, 309.46, 89.43, -128.43, -90.38) 
    # target_point2 = posx(487.05, 173.94, 295.53, 89.61, -128.46, -90.41) 
    # target_point3 = posx(566.14, 128.39, 282.95, 89.11, -128.23, -91)
    xlist = [posx(395.87, 144.91, 100.93, 90.38, -127.98, 90),
             posx(395.87, -183.79, 100.93, 90.38, -127.98, 90),
             posx(415.86, -224.88, 100.92, 90.38, -127.98, 90),
             posx(448.35, -224.89, 100.92, 90.38, -127.98, 90),
             posx(474.69, -178.35, 100.93, 90.38, -127.98, 90),
             posx(474.68, 136.46, 100.93, 90.38, -127.98, 90),
             posx(516.84, 162.17, 100.93, 90.39, -127.98, 90),
             posx(557.77, 124.13, 100.92, 90.39, -127.98, 90),
             posx(557.78, -193.65, 100.93, 90.39, -127.98, 90)
             ]

    def grip():
        set_digital_output(1,ON)
        set_digital_output(2,OFF)

    def release():
        set_digital_output(2,ON)
        set_digital_output(1,OFF)
    
    def cement():
        # step 1 truck move
        movel(cup_up, vel = VELOCITY, acc = ACC) # 컵 위로 이동
        mwait(0.1)
        movel(cup_down, vel = VELOCITY, acc = ACC) # 컵 아래로 이동
        mwait(0.1)
        grip() # 잡은 후 
        mwait(0.1)
        movel(cup_up, vel = VELOCITY, acc = ACC) # 컵 위로

        
        # step 2 mixing
        movej(JReady, vel=VELOCITY, acc=ACC) # 섞는 위치 이동하기
        move_spiral(rev=3, vel=50, acc=30, rmax=200.0,lmax=0,time=20.0, axis=DR_AXIS_Z, ref=DR_TOOL)
        movej(JStart, vel=VELOCITY, acc=ACC)


        # step 3 pavement
        movesx(xlist,vel=[100,30],acc=[200,60],vel_opt=DR_MVS_VEL_CONST)
        mwait(0.1)

        # movel(target_point1, vel = VELOCITY, acc = ACC) # target_point
        # amove_periodic(amp=example_amp,period=3.0, atime=0.02, repeat=3, ref=DR_TOOL) # 흔들기
        # mwait(5)

        # movel(target_point2, vel = VELOCITY, acc = ACC) # target_point
        # amove_periodic(amp=example_amp,period=3.0, atime=0.02, repeat=3, ref=DR_TOOL) # 흔들기
        # mwait(5)

        # movel(target_point3, vel = VELOCITY, acc = ACC) # target_point
        # amove_periodic(amp=example_amp,period=3.0, atime=0.02, repeat=3, ref=DR_TOOL) # 흔들기
        # mwait(5)

        # step 4 organization
        movel(cup_up, vel = VELOCITY, acc = ACC) # 컵 위로
        mwait(0.1)
        movel(cup_down, vel = VELOCITY, acc = ACC) # 내려가기
        mwait(0.1)
        release()
        mwait(0.1)


    set_tool("Tool Weight_2FG")
    set_tcp("2FG_TCP")

    if rclpy.ok():        

        # 로봇 위치 초기화
        release()
        movej(JReady, vel = 80, acc = 80)
        grip()
        mwait(0.1)

        # Pavement #
        cement()
        mwait(0.1)
        
    rclpy.shutdown()

if __name__ == "__main__":
        main()

