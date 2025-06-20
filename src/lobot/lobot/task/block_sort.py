import os
import sys
import time
import ast
import threading

# Force Qt to use XCB if Wayland plugin missing
os.environ['QT_QPA_PLATFORM'] = 'xcb'

import rclpy
import DR_init

from PyQt5.QtWidgets import QApplication, QWidget, QLabel, QVBoxLayout, QGridLayout, QPushButton
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QColor, QPalette
from PyQt5.QtWidgets import QWidget, QGraphicsView, QVBoxLayout
from PyQt5.QtGui import QPainter, QPolygonF, QColor
from PyQt5.QtCore import Qt, QPointF

class BuildingView(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.layers = [0] * 9
        self.color_map = [QColor('#ADD8E6'), QColor('#87CEEB'), QColor('#4682B4'), QColor('#005f99')]
    def set_layers(self, layers):
        self.layers = layers
        self.update()
    def paintEvent(self, event):
            painter = QPainter()
            painter.begin(self)
            try:
                painter.setRenderHint(QPainter.Antialiasing)
                w, h = self.width(), self.height()
                cell_w, cell_h = w/3, h/4
                half_w, layer_h = cell_w/2, cell_h*0.2

                for idx, floors in enumerate(self.layers):
                    col, row = idx%3, idx//3
                    base_x = col * cell_w
                    base_y = row * cell_h +  1.4* cell_h
                    # draw base site area in light gray
                    pts_base = [
                        QPointF(base_x + half_w, base_y - layer_h),
                        QPointF(base_x + half_w + half_w/2, base_y),
                        QPointF(base_x + half_w, base_y + layer_h),
                        QPointF(base_x + half_w - half_w/2, base_y)
                    ]
                    painter.setBrush(QColor('#d3d3d3'))
                    painter.setPen(Qt.NoPen)
                    painter.drawPolygon(QPolygonF(pts_base))

                    for level in range(floors):
                        y_off = level * layer_h
                        pts = [
                            QPointF(base_x + half_w,           base_y - y_off - layer_h),
                            QPointF(base_x + half_w + half_w/2, base_y - y_off),
                            QPointF(base_x + half_w,           base_y - y_off + layer_h),
                            QPointF(base_x + half_w - half_w/2, base_y - y_off),
                        ]
                        color = self.color_map[level % len(self.color_map)]
                        painter.setBrush(color)
                        painter.setPen(color.darker())
                        painter.drawPolygon(QPolygonF(pts))
            finally:
                painter.end()

# Configuration
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 100, 200
ON, OFF = 1, 0
FIRST, SECOND = 30, 50  # Force thresholds

# Initialize DR_init identifiers (must set before node creation)
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL
def main():
    app = QApplication(sys.argv)
    gui = StatusGUI()

    # 🔧 여기에 retry_event 정의 필요
    retry_event = threading.Event()

    thread = threading.Thread(target=run_process, args=(gui, retry_event), daemon=True)

    def on_start():
        if not thread.is_alive():
            thread.start()
            gui.start_btn.setText("시작")
        else:
            retry_event.set()

    gui.start_btn.clicked.connect(on_start)

    gui.show()
    sys.exit(app.exec_())

class StatusGUI(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Brick Assembly GUI")
        self.resize(400, 700)

        # 3x3 status grid
        grid = QGridLayout()
        self.labels = []
        for i in range(9):
            lbl = QLabel(str(i+1))
            lbl.setAutoFillBackground(True)
            lbl.setFixedSize(40, 40)
            lbl.setAlignment(Qt.AlignCenter)
            pal = lbl.palette()
            pal.setColor(QPalette.Window, QColor('lightgray'))
            lbl.setPalette(pal)
            grid.addWidget(lbl, i//3, i%3)
            self.labels.append(lbl)

        self.status_label = QLabel("대기 중...")
        self.status_label.setAlignment(Qt.AlignCenter)
        self.start_btn = QPushButton("시작")

        # BuildingView 추가
        self.building_view = BuildingView()
        
        self.building_view.setFixedHeight(200)
        # 층 수 저장용
        self.floors_built = [0] * 9

        layout = QVBoxLayout()
        layout.addLayout(grid)
        layout.addWidget(self.status_label)
        layout.addWidget(self.start_btn)
        layout.addWidget(self.building_view)
        self.setLayout(layout)

    def update_block_status(self, build_list):
        color_map = {0:'lightgray',1:'red',2:'yellow',3:'blue'}
        for i, val in enumerate(build_list):
            pal = self.labels[i].palette()
            pal.setColor(QPalette.Window, QColor(color_map[val]))
            self.labels[i].setPalette(pal)

    def mark_built(self, idx):
        # 해당 부지에 층 추가
        self.floors_built[idx] += 1
        self.building_view.set_layers(self.floors_built)

    def update_status_text(self, text):
        self.status_label.setText(text)

# Main process
def run_process(gui: StatusGUI, retry_event: threading.Event):
    # ROS2 init
    rclpy.init()
    node = rclpy.create_node("brick_assemble_node", namespace=ROBOT_ID)
    DR_init.__dsr__node = node
    # Import DSR API
    try:
        from DSR_ROBOT2 import (
            get_digital_input, set_digital_output,
            get_current_posx, set_ref_coord,
            set_tool, set_tcp,
            movej, movel, set_stiffnessx, amove_periodic,
            mwait, movesx,
            task_compliance_ctrl, release_compliance_ctrl,
            set_desired_force, release_force, check_force_condition,
            DR_BASE, DR_MVS_VEL_CONST, DR_TOOL,
            DR_FC_MOD_REL, DR_AXIS_Z, DR_MV_MOD_REL
        )
        from DR_common2 import posx, posj
    except ImportError as e:
        gui.update_status_text(f"Import error: {e}")
        return
    # Load system files
    with open('/home/rokey/ros_ws/building_text/build_list.txt','r',encoding='utf-8') as f:
        build_list = ast.literal_eval(f.read())
    with open('/home/rokey/ros_ws/building_text/blue_print_index.txt','r',encoding='utf-8') as f:
        blueprint_index = int(f.read())
    with open('/home/rokey/ros_ws/building_text/construct_index.txt','r',encoding='utf-8') as f:
        construct_index = ast.literal_eval(f.read())
    gui.update_block_status(build_list)
    gui.update_status_text(f"도면: {build_list}")
    # Coordinates
    blueprint_xy1 = posx(194.38, -60.5, 150.0, 152.35, 180, 143.21)
    blueprint_xy2 = posx(244.83, -53.04, 150.0, 166.55, 180, 157.29)
    blueprint_xy3 = posx(295.89, -45.59, 150.0, 161, -180, 151.96)
    blueprint_xy4 = posx(201.67, -111.3, 150.0, 144.15, 180, 135.19)
    blueprint_xy5 = posx(252.36, -103.68, 150.0, 141.41, -180, 132.02)
    blueprint_xy6 = posx(302.79, -95.66, 150.0, 165.55, 180, 155.94)
    blueprint_xy7 = posx(209.64, -161.45, 150.0, 139.01, 180, 129.42)
    blueprint_xy8 = posx(259.82, -154.17, 150.0, 153.75, 179.96, 144)
    blueprint_xy9 = posx(310.27, -146.5, 150.0, 20.05, -179.96, 10.52)
    position_lst = [blueprint_xy1, blueprint_xy2, blueprint_xy3,
                    blueprint_xy4, blueprint_xy5, blueprint_xy6,
                    blueprint_xy7, blueprint_xy8, blueprint_xy9]
    JReady = posj([0,0,90,0,90,0])
    cup_up = posx(457.41,268.36,225,104.04,179.98,103.65)
    cup_down = posx(456.61,267.75,65,16.53,-179.87,16.16)
    xlist = [
        posx(457.41,268.36,175,104.04,179.98,103.65), posx(456.61,267.81,175,16.53,-179.87,16.16),
        posx(395.87,144.91,175,90.38,-127.98,90), posx(395.87,-183.79,175,90.38,-127.98,90),
        posx(415.86,-224.88,175,90.38,-127.98,90), posx(448.35,-224.89,175,90.38,-127.98,90),
        posx(474.69,-178.35,175,90.38,-127.98,90), posx(474.68,136.46,175,90.38,-127.98,90),
        posx(516.84,162.17,175,90.39,-127.98,90), posx(557.77,124.13,175,90.39,-127.98,90),
        posx(557.78,-193.65,175,90.39,-127.98,90)
    ]
    example_amp = [0,0,0,0,0,100]
    pos_home = posj(0,0,90,0,90,0)
    height = 120
    block_short = posx(299.07,89.08,41+height,13.84,180,105.56)
    block_long = posx(299.51,14.82,41+height,9.72,180,101.8)
    step_1 = posx(401.43,114.9,35+height,69,180,68.92)
    step_2 = posx(480.75,114.37,35+height,3.87,180,4.17)
    step_3 = posx(555.21,113.93,35+height,10.35,180,10.71)
    step_4 = posx(400.75,2.88,35+height,178.8,180,179.27)
    step_5 = posx(480.54,2.21,35+height,17.65,180,18.13)
    step_6 = posx(555.57,1.85,35+height,169.21,180,169.69)
    step_7 = posx(399.66,-108.28,35+height,160.73,180,161.28)
    step_8 = posx(479.99,-108.84,35+height,167.52,180,167.91)
    step_9 = posx(555.68,-109.54,35+height,146.96,180,147.18)
    block_to_down = posx(0,0,-height,0,0,0)
    block_for_spread = posx(0,0,40,0,0,0)
    offset_for_short = posx(0,-31.9,0,0,0,0)
    offset_for_long = posx(0,-47.7,0,0,0,0)
    dummy = posj([37.12,-0.03,100.27,-0.02,79.76,129.49])
    # Helper functions
    def grip(): 
        set_digital_output(1,ON); set_digital_output(2,OFF); time.sleep(0.5)
    def release(): 
        set_digital_output(1,OFF); set_digital_output(2,ON); time.sleep(0.5)
    def check_bar(idx):
        task_compliance_ctrl()
        set_stiffnessx([3000,3000,3000,200,200,200], time=0)
        set_desired_force([0,0,-50,0,0,0],[0,0,1,0,0,0], time=0)
        while not check_force_condition(DR_AXIS_Z, max=25): pass
        x = get_current_posx()[0]
        if x[2]>=64: build_list[idx]=3
        elif x[2]>=54: build_list[idx]=2
        elif x[2]>=44: build_list[idx]=1
        else: build_list[idx]=0
        release_force(time=0); release_compliance_ctrl()
        gui.update_block_status(build_list)
    def cement():
        gui.update_status_text('시멘트 시작')
        print('시멘트 도구 장비')
        set_ref_coord(DR_BASE); movel(cup_up,vel=VELOCITY,acc=ACC); mwait(0.1)
        release(); movel(cup_down,vel=VELOCITY,acc=ACC); mwait(0.1)
        grip(); movel(cup_up,vel=VELOCITY,acc=ACC)
        gui.update_status_text('시멘트 작업 중')
        print('시멘트 몰탈 시작'); movej(JReady,vel=VELOCITY,acc=ACC)
        amove_periodic(amp=example_amp,period=3,atime=0.02,repeat=2,ref=DR_TOOL); mwait(0.1)
        print('몰탈 시작'); movesx(xlist,vel=[100,30],acc=[200,60],vel_opt=DR_MVS_VEL_CONST); mwait(0.1)
        print('몰탈 정비'); movel(cup_up,vel=VELOCITY,acc=ACC); mwait(0.1)
        movel(cup_down,vel=VELOCITY,acc=ACC); mwait(0.1); release()
        movel(cup_up,vel=VELOCITY,acc=ACC); mwait(0.1)
        gui.update_status_text('시멘트 완료')
    # Main
    set_tool("Tool Weight_2FG"); set_tcp("2FG_TCP")
    release(); movej(pos_home,vel=80,acc=80); time.sleep(0.5); grip()
    gui.update_status_text('초기 위치 완료')
    gui.update_status_text('측정 시작')
    print('측정 시작')
    if blueprint_index<9:
        for idx in range(blueprint_index, len(position_lst)):
            gui.update_status_text(f'{idx+1}번 부지 측정 시작'); print(f'{idx+1}번 부지 측정 시작')
            movel(position_lst[idx],vel=VELOCITY,acc=ACC,ref=DR_BASE); check_bar(idx)
            movel(position_lst[idx],vel=VELOCITY,acc=ACC,ref=DR_BASE)
            with open('/home/rokey/ros_ws/building_text/blue_print_index.txt','w',encoding='utf-8') as f: f.write(str(idx+1))
            with open('/home/rokey/ros_ws/building_text/build_list.txt','w',encoding='utf-8') as f: f.write(str(build_list))
            gui.update_status_text(f'{idx+1}번 완료')
        gui.update_status_text('측정 완료'); print('측정 완료')
    else:
        gui.update_status_text('이미 측정 완료')
    time.sleep(1)
    if blueprint_index<=0:
        cement(); time.sleep(1)
    gui.update_status_text('건설 시작'); print('건설 시작')
    if construct_index<9:
        for idx in range(construct_index, len(build_list)):
            release()
            if build_list[idx]==0:
                gui.update_status_text(f'{idx+1}번 부지 건설 없음')
                with open('/home/rokey/ros_ws/building_text/construct_index.txt','w',encoding='utf-8') as f: f.write(str(idx+1))
                continue
            elif build_list[idx] in (1,2):
                timeout = 10
                block_for_construction = block_short if build_list[idx]==1 else block_long
                gui.update_status_text(f'{idx}타입 {build_list[idx]} 건설 시작')
                for floor in range(4):
                    print(f'{idx}층{floor}집기'); gui.update_status_text(f'{idx+1}번 부지 {build_list[idx]}타입 {floor+1}층 건설 중')                    
                    movel(block_for_construction,vel=VELOCITY,acc=ACC)
                    mwait(0.5) 
                    movel(block_to_down,vel=VELOCITY,acc=ACC,mod=DR_MV_MOD_REL)
                    mwait(0.5) 
                    grip()    
                    while True:
                        di1 = get_digital_input(1)
                        di2 = get_digital_input(2)
                        di3 = get_digital_input(3)
                        print(f'{floor+1}층 진행중')
                        # 1. 그립 실패: [0,1,0]
                        if [di1, di2, di3] == [1,0,0]:
                            release()
                            gui.update_status_text("❌ 그립 실패 — 시작 버튼을 눌러 재시도")
                            retry_event.clear()      # 이전 신호 초기화
                            retry_event.wait()       # 버튼 클릭될 때까지 대기
                            grip()
                            continue
                        # 2. 비정상 블록 감지: [0,0,1]
                        elif [di1, di2, di3] == [0,0,1]:
                            gui.update_status_text("⚠️ 비정상 블록 — 시작 버튼을 눌러 재시도")
                            movej(dummy, vel=VELOCITY, acc=ACC)
                            mwait(0.5)
                            release()
                            movel(block_for_construction, vel=VELOCITY, acc=ACC, ref=DR_BASE)
                            mwait(0.5)
                            movel(block_to_down, vel=VELOCITY, acc=ACC, ref=DR_BASE, mod=DR_MV_MOD_REL)                            
                            retry_event.clear()
                            retry_event.wait()
                            continue
                        # 3. 정상 블록 감지: [0,1,1]
                        elif [di1, di2, di3] == [0,1,1]:
                            print("✅ 정상 블록 감지 완료!")
                            gui.update_status_text("✅ 정상 블록 감지 완료!")
                            break
                        # 4. 예외 처리: 예상치 못한 조합
                        else:
                            print(f"⚠️ 측정중")
                            gui.update_status_text(f"⚠️ 블럭 측정중")
                            time.sleep(1)  # 잠시 대기 후 다시 측정
                            grip()
                            continue
                        
                    movel(block_for_construction, vel = VELOCITY, acc = ACC)
                    mwait(0.5)

                    # 레고블럭 쌓을 위치로 이동
                    print('블럭 쌓기')
                    movel([step_1,step_2,step_3,step_4,step_5,step_6,step_7,step_8,step_9][idx], vel = VELOCITY, acc = ACC)

                    # Force 설정
                    task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
                    time.sleep(1)
                    set_desired_force(fd=[0, 0, -50, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
                    time.sleep(1)
                    force_condition = check_force_condition(DR_AXIS_Z, max=40)
                    start_time = time.time()                    
                    while force_condition == 0 and (time.time() - start_time < timeout):
                        force_condition = check_force_condition(DR_AXIS_Z, max=SECOND)
                    timeout -= 1.2                    
                    release_compliance_ctrl()
                    release()
                    mwait(0.2)
                    movel(block_for_spread, vel = VELOCITY, acc = ACC, mod=DR_MV_MOD_REL)
                    print('블럭 고정')
                    gui.update_status_text(f"{idx+1}번 부지 {floor+1}층 건설 완료")
                    gui.mark_built(idx)  # <-- 한 층 쌓일 때마다 호출
                    mwait(0.2)
                    grip()
                    task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
                    time.sleep(1)
                    set_desired_force(fd=[0, 0, -50, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
                    time.sleep(1)
                    force_condition = check_force_condition(DR_AXIS_Z, max=40)
                    start_time = time.time()
                    while force_condition == 0:
                        force_condition = check_force_condition(DR_AXIS_Z, max=SECOND)
                           
                    release_force()
                    release_compliance_ctrl()
                    mwait(0.2)
                    movel([step_1,step_2,step_3,step_4,step_5,step_6,step_7,step_8,step_9][idx], vel = VELOCITY, acc = ACC)
                    mwait(0.2)
                    release()               
                with open('/home/rokey/ros_ws/building_text/construct_index.txt','w',encoding='utf-8') as f: f.write(str(idx+1))
                gui.update_status_text(f'{idx}완료')
            else:
                gui.update_status_text(f'{idx}타입3 건설 시작')
                _3type_building=[[3,3],[2,2,2],[3,3],[2,2,2]]
                timeout = 11
                for fl,plan in enumerate(_3type_building):
                    timeout -= 1
                    
                    for bc,bt in enumerate(plan):
                        print(f'{idx}타입3 플로어{fl} 블록{bc}'); gui.update_status_text(f'{idx+1}번 부지 3타입 {fl+1}층 {bc+1}번째 건설 중')
                        block_for_construction=block_short if bt==2 else block_long
                        movel(block_for_construction,vel=VELOCITY,acc=ACC);mwait(0.5); movel(block_to_down,vel=VELOCITY,acc=ACC,mod=DR_MV_MOD_REL); mwait(0.5); grip()
                        while True:
                            di1 = get_digital_input(1)
                            di2 = get_digital_input(2)
                            di3 = get_digital_input(3)
                            print(f'{fl+1}층 진행중')
                            # 1. 그립 실패: [0,1,0]
                            if [di1, di2, di3] == [1,0,0]:
                                release()
                                gui.update_status_text("❌ 그립 실패 — 시작 버튼을 눌러 재시도")
                                retry_event.clear()      # 이전 신호 초기화
                                retry_event.wait()       # 버튼 클릭될 때까지 대기
                                grip()
                                continue
                            # 2. 비정상 블록 감지: [0,0,1]
                            elif [di1, di2, di3] == [0,0,1]:
                                gui.update_status_text("⚠️ 비정상 블록 — 시작 버튼을 눌러 재시도")
                                movej(dummy, vel=VELOCITY, acc=ACC)
                                mwait(0.5)
                                release()
                                movel(block_for_construction, vel=VELOCITY, acc=ACC, ref=DR_BASE)
                                mwait(0.5)
                                movel(block_to_down, vel=VELOCITY, acc=ACC, ref=DR_BASE, mod=DR_MV_MOD_REL)                            
                                retry_event.clear()
                                retry_event.wait()
                                continue
                            # 3. 정상 블록 감지: [0,1,1]
                            elif [di1, di2, di3] == [0,1,1]:
                                print("✅ 정상 블록 감지 완료!")
                                gui.update_status_text("✅ 정상 블록 감지 완료!")
                                break
                            # 4. 예외 처리: 예상치 못한 조합
                            else:
                                print(f"⚠️ 측정중")
                                gui.update_status_text(f"⚠️ 블럭 측정중")
                                time.sleep(1)  # 잠시 대기 후 다시 측정
                                grip()
                                continue
                        movel(block_for_construction, vel = VELOCITY, acc = ACC)
                        mwait(0.5)
                        rel=offset_for_short if bt==2 else offset_for_long
                        pose=[step_1,step_2,step_3,step_4,step_5,step_6,step_7,step_8,step_9][idx].copy()
                        if bc: pose[0]+=rel[0]*bc; pose[1]+=rel[1]*bc
                        movel(pose,vel=VELOCITY,acc=ACC)
                        task_compliance_ctrl(stx=[500,500,500,100,100,100]); time.sleep(1)
                        set_desired_force(fd=[0,0,-30,0,0,0],dir=[0,0,1,0,0,0],mod=DR_FC_MOD_REL);time.sleep(1)
                        force_condition = check_force_condition(DR_AXIS_Z, max=FIRST)
                        start_time = time.time()                        
                        while force_condition == 0 and (time.time() - start_time < timeout):
                             force_condition = check_force_condition(DR_AXIS_Z, max=SECOND)       
                        release_force(); release_compliance_ctrl(); release()
                        movel(block_for_spread,vel=VELOCITY,acc=ACC,mod=DR_MV_MOD_REL); grip()
                        task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
                        time.sleep(1)
                        set_desired_force(fd=[0, 0, -50, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
                        time.sleep(1)
                        force_condition = check_force_condition(DR_AXIS_Z, max=SECOND)
                        start_time = time.time()    
                        while force_condition == 0:
                            force_condition = check_force_condition(DR_AXIS_Z, max=SECOND)                        
                        release_force()
                        release_compliance_ctrl()
                        mwait(0.2)
                        movel(pose, vel=VELOCITY, acc=ACC)
                        gui.update_status_text(f"{idx+1}번 부지 3타입 {fl+1}층 {bc+1}번째 건설 완료")
                        mwait(0.2)
                        release()  
                    gui.mark_built(idx)  # <-- 타입3에서도 호출              
                with open('/home/rokey/ros_ws/building_text/construct_index.txt','w',encoding='utf-8') as f: f.write(str(idx+1))
                gui.update_status_text(f'{idx}완료')
        gui.update_status_text('전체 건설 완료')
        movej(pos_home,vel=30,acc=30); 
    else:
        gui.update_status_text('건설 이미 완료')
        with open('/home/rokey/ros_ws/building_text/blue_print_index.txt', 'w', encoding='utf-8') as file:
            file.write(str(0))
        with open('/home/rokey/ros_ws/building_text/construct_index.txt', 'w', encoding='utf-8') as file:
            file.write(str(0))   
        with open('/home/rokey/ros_ws/building_text/build_list.txt', 'w', encoding='utf-8') as file:
                    file.write(str([0,0,0,0,0,0,0,0,0])) 
    rclpy.shutdown()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    gui = StatusGUI()

    # 1) 재시도용 Event 객체 생성'
    retry_event = threading.Event()

    # 2) run_process 스레드 미리 생성(한 번만)
    worker_thread = threading.Thread(
        target=run_process,
        args=(gui, retry_event),
        daemon=True
    )

    # 3) Start 버튼 클릭 핸들러 정의
    def on_start():
        # 스레드를 한 번만 시작
        if not worker_thread.is_alive():
            worker_thread.start()
            gui.start_btn.setText("시작")      # 버튼 텍스트 바꾸거나
        # 이후 누를 땐 retry_event.set()만 작동
        retry_event.set()

    gui.start_btn.clicked.connect(on_start)

    gui.show()
    sys.exit(app.exec_())