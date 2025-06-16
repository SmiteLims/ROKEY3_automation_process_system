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

# location : lego
block_short = posx(379.32, 192.52, 233.65+100, 98.77, 179.74, 65.97) # long block position
block_long = posx(295.97, 17.57, 269.64+100, 179.28, 180.0, 91.38) # shot block position

# location : row 1, column 1 (1)
step_1 = posx(405.56, 128.88, 259.14+100, 23.12, -180, 24.17)
# location : row 1, column 2 (2)
step_2 = posx(516.63, 128.32, 258.67+100, 8.61, -180, 10.17)
# location : row 1, column 3 (3)
step_3 = posx(627.96, 126.02, 259.87+100, 43.33, 180, 44.96)

# location : row 2, column 1 (4)
step_4 = posx(403.77, 1.11, 259.86+100, 146.56, -179.97, 148.58)
# location : row 2, column 2 (5)
step_5 = posx(515.01, -0.64, 260.7+100, 176.19, 180, 178.2)
# location : row 2, column 3 (6)
step_6 = posx(625.36, 0.31, 262.03+100, 174.87, -179.96, 176.86)

# location : row 3, column 1 (7)
step_7 = posx(400.35, -125.82, 261.81+100, 2.8, -180, 4.77)
# location : row 3, column 2 (8)
step_8 = posx(512.38, -126.53, 259.75+100, 165.26, 179.99, 167.18)
# location : row 3, column 3 (9)
step_9 = posx(623.47, -128.45, 261.84+100, 167.59, 179.97, 169.52)

# difference
# Just to go up
block_to_down = posx(0,0,-100,0,0,0)


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




    estate = [0,1,0,0,2,0] # blueprint interpretation
    # estate[0] = 1

    if rclpy.ok():
        
        for idx,buildings in enumerate(estate):
            
            # initialization
            movej(pos_home, vel = 30, acc = 30)
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
                movel(block_for_construction, vel = 30, acc = 30, ref=DR_BASE) # move to sourcing site 
                mwait(0.5)
                movel(block_to_down, vel = 30, acc = 30, mod=DR_MV_MOD_REL) # move down to pick up blocks
                mwait(0.5)
                grip()
                mwait(0.5)
                movel(block_for_construction, vel = 30, acc = 30) # move up
                mwait(0.5)

                # CONSTRUCT
                movel(block_to_place, vel = 30, acc = 30)

                task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
                time.sleep(1)
                set_desired_force(fd=[0, 0, -20, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
                time.sleep(1)
                force_condition = check_force_condition(DR_AXIS_Z, max=30)

                while (force_condition == 0): # 힘 제어로 블럭 놓기
                    force_condition = check_force_condition(DR_AXIS_Z, max=20) # 조건 만족하면 0 (ROS2에서)

                release_force()
                release_compliance_ctrl()
                release()
                mwait(0.2)
                movel(block_to_place, vel = 30, acc = 30)

            print(f"{idx} 단계 완료, 다음 단계로 넘어갑니다.")

    rclpy.shutdown()
if __name__ == "__main__":
    main()
