import time
import rclpy
import DR_init

# for single robot
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 60, 60

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

OFF, ON = 0, 1


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("rokey_gear_install", namespace=ROBOT_ID)

    DR_init.__dsr__node = node

    try:
        from DSR_ROBOT2 import (
            set_digital_output,
            get_digital_input,
            release_compliance_ctrl,
            release_force,
            check_force_condition,
            task_compliance_ctrl,
            set_desired_force,
            set_tool,
            set_tcp,
            movej,
            movel,
            DR_FC_MOD_REL,
            DR_AXIS_Z,
            DR_BASE,
        )

        from DR_common2 import posx, posj

    except ImportError as e:
        print(f"Error importing DSR_ROBOT2 : {e}")
        return
    

    def wait_digital_input(sig_num):
        while not get_digital_input(sig_num):
            time.sleep(0.5)
            print(f"Wait for digital input: {sig_num}")
            pass

    def release():
        print("set for digital output 0 1 for release")
        set_digital_output(2, ON)
        set_digital_output(1, OFF)
        wait_digital_input(2)

    def grip():
        print("set for digital output 1 0 for grip")
        set_digital_output(1, ON)
        set_digital_output(2, OFF)
        wait_digital_input(1)

    set_tool("Tool Weight_2FG")
    set_tcp("2FG_TCP")




    height_default = 350
    # pos = posx([496.06, 93.46, 300, 20.75, 179.00, 19.09])
    JReady = posj([0, 0, 90, 0, 90, 0])
    origin1 = posx(460.99, 305.87, height_default, 24.63, 179.16, -44.25)
    origin2 = posx(465.66, 203.52, height_default, 85.67, -179.82, 17.02)
    origin3 = posx(557.22, 259.47, height_default, 24.23, -179.91, -44.71)
    origin4 = posx(496.55, 257.55, height_default, 25.12, -179.99, -43.56)
    origin_lst = [origin1, origin2, origin3, origin4]

    install1 = posx(188.08, 182.3, height_default, 51.18, 180, -17.25)
    install2 = posx(248.51, 95.12, height_default, 3.39, -180, -63.98)
    install3 = posx(295.48, 189.99, height_default, 33.81, -179.99, -33.63)
    install4 = posx(246.75, 155.84, height_default, 28.62, 179.99, -38.52)
    install_lst = [install1, install2, install3, install4]
    
    if rclpy.ok():
        if input("Type in start") == "start":
            
            print(f"Moving to joint position: {JReady}")
            movej(JReady, vel=VELOCITY, acc=ACC)

            for i in range(4):

                movel(origin_lst[i], vel=VELOCITY, acc=ACC, ref=DR_BASE)
                time.sleep(0.5)
                if i == 4:
                    task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
                    time.sleep(0.5)
                    set_desired_force(fd=[0, 0, -30, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
                    while not check_force_condition(DR_AXIS_Z, max=10):
                        print("Waiting for an external force greater than 10 ")
                        time.sleep(0.5)
                        pass
                    release_force()
                    time.sleep(0.5)
                    release_compliance_ctrl()
                 
                movel(install_lst[i], vel=VELOCITY, acc=ACC, ref=DR_BASE)
                if i == 4:

                    time.sleep(0.5)
                    task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
                    time.sleep(0.5)
                    set_desired_force(fd=[0, 0, -30, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
                    while not check_force_condition(DR_AXIS_Z, max=10):
                        print("Waiting for an external force greater than 10 ")
                        time.sleep(0.5)
                        pass
                    release_force()
                    time.sleep(0.5)
                    release_compliance_ctrl()  
        
    rclpy.shutdown()


if __name__ == "__main__":
    main()
