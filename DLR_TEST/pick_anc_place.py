import rclpy
import DR_init
import time

# 로봇 설정: 단일 로봇 사용
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 40, 40  # 전역 속도 및 가속도 설정

# DR_init을 통한 로봇 정보 등록
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

# ROS2 노드 초기화
rclpy.init()
node = rclpy.create_node("rokey_stacking", namespace=ROBOT_ID)
DR_init.__dsr__node = node

ON, OFF = 1, 0
HOME_READY = [0, 0, 90, 0, 90, 0]  # 기본 홈 포즈 (관절 좌표)

# DSR API import
try:
    from DSR_ROBOT2 import (
        get_digital_input, set_digital_output,
        get_current_posx, trans,
        set_tool, set_tcp,DR_BASE,
        movej, movel,
        wait, mwait,
        task_compliance_ctrl, release_compliance_ctrl,
        set_desired_force, release_force, check_force_condition,
        DR_FC_MOD_REL, DR_AXIS_Z,)
    from DR_common2 import posx, posj
except ImportError as e:
    print(f"Error importing DSR_ROBOT2 : {e}")
    exit()

# 툴 및 TCP 설정
set_tool("Tool Weight_2FG")
set_tcp("2FG_TCP")

# 디지털 입력이 들어올 때까지 대기
def wait_digital_input(sig_num):
    while not get_digital_input(sig_num):
        wait(0.5)
        print("Wait for digital input")
        pass

# 그리퍼 열기 (release)
def release():
    set_digital_output(2, ON)
    set_digital_output(1, OFF)
    # wait_digital_input(2)

# 그리퍼 닫기 (grip)
def grip():
    set_digital_output(1, ON)
    set_digital_output(2, OFF)
    # wait_digital_input(1)


# 박스 클래스 정의 (스택/언스택 대상 객체)
class Blueprint:
    def __init__(self):
        self.nocontact_z = 350.0
        self.blueprint_xy1 = posx(198.21,-58.48,350,147.25,-179.99,145.78)
        self.blueprint_xy2 = posx(249.95,-52.92,350,11.30,-179.55,9.85)
        self.blueprint_xy3 = posx(303.75,-50.58,350,3.91,-179.05,2.20)
        self.blueprint_xy4 = posx(204.68,-107.52,350,6.47,-178.94,4.53)
        self.blueprint_xy5 = posx(257.29,-105.63,350,3.56,-178.59,1.44)
        self.blueprint_xy6 = posx(311.57,-102.36,350,179.50,177.77,176.98)
        self.blueprint_xy7 = posx(214.47,-159.75,350,171.63,177.15,169.30)
        self.blueprint_xy8 = posx(267.19,-156.88,350,173.03,176.69,170.36)
        self.blueprint_xy9 = posx(320.45,-155.79,350,174.63,176.23,171.84)
        self.position_lst = [self.blueprint_xy1,
                             self.blueprint_xy2,
                             self.blueprint_xy3,
                             self.blueprint_xy4,
                             self.blueprint_xy5,
                             self.blueprint_xy6,
                             self.blueprint_xy7,
                             self.blueprint_xy8,
                             self.blueprint_xy9]





        self.move_position_xy1 = posx(194.06, -212.82, 272.11, 136.89, 180, 135.3)
        self.move_position_xy2 = posx(249.04, -208.6, 271.62, 131.21, 180, 129.54)
        self.move_position_xy3 = posx(299.15, -204.28, 271.21, 125.49, 180, 123.58)
        self.move_position_xy4 = posx(198.61, -261.26, 270.73, 123.41, 179.99, 121.48)
        self.move_position_xy5 = posx(253.2, -259.33, 270.37, 121.97, 179.99, 119.38)
        self.move_position_xy6 = posx(301.92, -253.32, 270.1, 147.87, -180, 144.85)
        self.move_position_xy7 = posx(203.49, -311.72, 270.01, 116.34, 179.99, 110.69)
        self.move_position_xy8 = posx(254.49, -307.87, 269.7, 159.93, -180, 153.76)
        self.move_position_xy9 = posx(305.63, -304.16, 269.28, 144.86, -180, 138.41)
        self.position_lst2 = [self.move_position_xy1,
                             self.move_position_xy2,
                             self.move_position_xy3,
                             self.move_position_xy4,
                             self.move_position_xy5,
                             self.move_position_xy6,
                             self.move_position_xy7,
                             self.move_position_xy8,
                             self.move_position_xy9]
        self.interpretation = [0 for _ in range(len(self.position_lst))]
        
    def read_blueprint(self):
        release()
        time.sleep(0.3)
        grip()
        time.sleep(0.3)
        print('start')

        for i in range(len(self.position_lst)):
            movel(self.position_lst[i], vel=VELOCITY, acc=ACC, mod=0)
            mwait()
            
            task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
            set_desired_force(fd=[0, 0, -20, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)

            force_condition = 0

            while True:
                force_condition = check_force_condition(DR_AXIS_Z, max=15)
                if force_condition == True:
                    x = get_current_posx()[0]
                    print(f'before {x}')
                    x[2] += 10
                    print(f"after {x}")
                    movel(x, vel=60, acc=60, ref=DR_BASE)
                    x[2] -= 20
                    movel(x, vel=60, acc=60, ref=DR_BASE)
                    grip()
                    time.sleep(0.3)
                    break
                time.sleep(0.1)

            release_force()
            release_compliance_ctrl()

            movel(self.position_lst[i], vel=VELOCITY, acc=ACC, mod=0)
            # 2번째
            movel(self.position_lst2[i], vel=VELOCITY, acc=ACC, mod=0)
            mwait()
            
            task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
            set_desired_force(fd=[0, 0, -20, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)

            force_condition = 0

            while True:
                force_condition = check_force_condition(DR_AXIS_Z, max=15)
                if force_condition == True:
                    release()
                    time.sleep(0.3)
                    break
                time.sleep(0.1)

            release_force()
            release_compliance_ctrl()

            movel(self.position_lst2[i], vel=VELOCITY, acc=ACC, mod=0)


        return self.interpretation


# 메인 루프
def main():
    if rclpy.ok():
        blueprint = Blueprint()
        interpretation_info = blueprint.read_blueprint()
    rclpy.shutdown()

# 스크립트 실행 시 main 함수 호출
if __name__ == "__main__":
    main()
