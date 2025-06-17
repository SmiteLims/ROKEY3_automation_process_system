import rclpy
import DR_init
import time
import sys
from DR_common2 import posx, posj

ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 30, 30
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL
ON, OFF = 1, 0

# 감지 위치 정의
nocontact_z = 350.0

# Original position
position_lst = [
    posx(198.21,-58.48,nocontact_z,147.25,-179.99,145.78),
    posx(249.95,-52.92,nocontact_z,11.30,-179.55,9.85),
    posx(303.75,-50.58,nocontact_z,3.91,-179.05,2.20),
    posx(204.68,-107.52,nocontact_z,6.47,-178.94,4.53),
    posx(257.29,-105.63,nocontact_z,3.56,-178.59,1.44),
    posx(311.57,-102.36,nocontact_z,179.50,177.77,176.98),
    posx(214.47,-159.75,nocontact_z,171.63,177.15,169.30),
    posx(267.19,-156.88,nocontact_z,173.03,176.69,170.36),
    posx(320.45,-155.79,nocontact_z,174.63,176.23,171.84)
]

# target position
short_pallets = [posx(194.06, -212.82, 272.11, 136.89, 180, 135.3), 
                 posx(249.04, -208.6, 271.62, 131.21, 180, 129.54), 
                 posx(299.15, -204.28, 271.21, 125.49, 180, 123.58)]
mid_pallets   = [posx(198.61, -261.26, 270.73, 123.41, 179.99, 121.48), 
                 posx(253.2, -259.33, 270.37, 121.97, 179.99, 119.38), 
                 posx(301.92, -253.32, 270.1, 147.87, -180, 144.85)]
tall_pallets  = [posx(203.49, -311.72, 270.01, 116.34, 179.99, 110.69), 
                 posx(254.49, -307.87, 269.7, 159.93, -180, 153.76), 
                 posx(305.63, -304.16, 269.28, 144.86, -180, 138.41)]
block_to_down = posx(0, 0, -100, 0, 0, 0)  # 상대적 하강

short_idx = 0
mid_idx = 0
tall_idx = 0

status_lst = []
def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("rokey_sorter", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    try:
        from DSR_ROBOT2 import (
            set_tool, set_tcp, movej, movel,
            set_digital_output, get_current_posx,
            set_ref_coord, task_compliance_ctrl, set_stiffnessx,
            set_desired_force, release_force, release_compliance_ctrl,
            check_force_condition, mwait,
            DR_AXIS_Z, DR_BASE, DR_MV_MOD_REL
        )
    except ImportError as e:
        print(f"Error importing DSR_ROBOT2: {e}")
        return

    def grip():
        set_digital_output(1, ON)
        set_digital_output(2, OFF)

    def release():
        set_digital_output(2, ON)
        set_digital_output(1, OFF)

    def check_bar():
        set_ref_coord(0)
        task_compliance_ctrl()
        set_stiffnessx([3000.00, 3000.00, 3000.00, 200.00, 200.00, 200.00],time=0.0)
        set_desired_force([0.00, 0.00, -30.00, 0.00, 0.00, 0.00],[0,0,1,0,0,0],time=0.0)
        # 긴거: 3, 중간거: 2, 짧은거: 1, 없는거 0 
        while True:
            if check_force_condition(axis=DR_AXIS_Z, max=25):
                current_pos = get_current_posx()[0]
                if current_pos[2] >= 290:
                    status_lst.append(3)
                elif current_pos[2] >= 280:
                    status_lst.append(2)
                elif current_pos[2] >= 270:
                    status_lst.append(1)
                else:
                    status_lst.append(0)
                break         
        release_force(time=0.0)
        release_compliance_ctrl()

    def check_and_grab():
        set_ref_coord(DR_BASE)
        task_compliance_ctrl()
        set_stiffnessx([3000.0]*3 + [200.0]*3)
        set_desired_force([0.0, 0.0, -30.0, 0.0, 0.0, 0.0], [0, 0, 1, 0, 0, 0])

        while True:
            if check_force_condition(axis=DR_AXIS_Z, max=20):
                break
            
        release_force()
        release_compliance_ctrl()

        temp = get_current_posx()[0]
        temp[2] += 10
        movel(temp, vel=150, acc=300, ref=DR_BASE)
        release()
        mwait(0.1)
        temp[2] -= 20
        movel(temp, vel=150, acc=300, ref=DR_BASE)
        mwait(0.1)
        grip()
        mwait(0.1)
        movel(posx(0, 0, 100, 0, 0, 0), vel=VELOCITY, acc=ACC, mod=DR_MV_MOD_REL)
        return

    def place_block():
        set_ref_coord(DR_BASE)
        task_compliance_ctrl()
        set_stiffnessx([3000.0]*3 + [200.0]*3)
        set_desired_force([0.0, 0.0, -30.0, 0.0, 0.0, 0.0], [0, 0, 1, 0, 0, 0])

        while True:
            if check_force_condition(axis=DR_AXIS_Z, max=20):
                break
            
        release_force()
        release_compliance_ctrl()
        release()
        mwait(0.1)
        movel(posx(0, 0, 100, 0, 0, 0), vel=VELOCITY, acc=ACC, mod=DR_MV_MOD_REL)
        mwait(0.1)
        grip()
        mwait(0.1)

    set_tool("Tool Weight_2FG")
    set_tcp("2FG_TCP")

    JReady = posj([0.0, 0.0, 90.0, 0.0, 90.0, 0.0])
    movej(JReady, vel=80, acc=80)
    release()
    mwait(0.1)
    grip()
    mwait(0.1)

    if input("Measurement is starting. Please type in 'start'!") == "start":
        print("Start")

    for idx, check_list in enumerate(position_lst):
        
        print(f"Moving to approach pose: {idx}")
        movel(check_list, vel=150, acc=300, ref=DR_BASE)
        # 팔레트 위치 측정
        check_bar()    
        movel(check_list, vel=150, acc=300, ref=DR_BASE)

    # 측정 결과 출력
    print('Measurement Complete!')
    print(status_lst)

    if input("Sorting is starting. Please type in 'start'!") == "start":
        print("Start")

    for status, pos in zip(status_lst,position_lst):
        movel(pos, vel=150, acc=300, ref=DR_BASE)
        check_and_grab()
        
        if status == 0:
            continue
        elif status == 1:
            movel(short_pallets[short_idx], vel=150, acc=300, ref=DR_BASE)
            place_block()
            short_idx += 1
        elif status == 2:
            movel(mid_pallets[mid_idx], vel=150, acc=300, ref=DR_BASE)
            place_block()
            mid_idx += 1
        elif status == 3:
            movel(tall_pallets[tall_idx], vel=150, acc=300, ref=DR_BASE)
            place_block()
            tall_idx += 1
        else:
            print("Error")
            sys.exit()
        
    rclpy.shutdown()




'''raceback (most recent call last):
  File "/home/rokey/ros_ws/install/rokey/lib/rokey/pick_place", line 33, in <module>
    sys.exit(load_entry_point('rokey==0.0.0', 'console_scripts', 'pick_place')())
  File "/home/rokey/ros_ws/build/rokey/rokey/advance/pick_place.py", line 169, in main
    check_and_grab()
  File "/home/rokey/ros_ws/build/rokey/rokey/advance/pick_place.py", line 107, in check_and_grab
    temp = get_current_posx()[0]
  File "/home/rokey/ros2_ws/install/dsr_common2/lib/dsr_common2/imp/DSR_ROBOT2.py", line 956, in get_current_posx
    pos.append(posx_info[0][i])
IndexError: list index out of range
[ros2run]: Process exited with failure 1
rokey@rokey-550XBE-350XBE:~/ros_ws$''' 



if __name__ == "__main__":
    main()
