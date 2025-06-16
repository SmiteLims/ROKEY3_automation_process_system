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
            DR_AXIS_Z,
            DR_BASE
        )

        from DR_common2 import posx, posj

    except ImportError as e:
        print(f"Error importing DSR_ROBOT2 : {e}")
        return
    
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
    graps()
     # 시작
    print('측정 시작')
    for idx, check_list in enumerate(position_lst):
        
        print(f"Moving to approach pose: {idx}")
        movel(check_list, vel=150, acc=300, ref=DR_BASE)
        # 팔레트 위치 측정
        check_bar()    
    # 측정 결과 출력
    print('측정 완료')
    print(build_list)

    rclpy.shutdown()

if __name__ == "__main__":
    main()