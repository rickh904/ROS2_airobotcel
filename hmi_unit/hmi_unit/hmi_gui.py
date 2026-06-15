# hmi_gui.py
from PyQt5.QtWidgets import (QMainWindow, QWidget, QLabel, QPushButton, 
                             QVBoxLayout, QHBoxLayout, QGridLayout, QGroupBox, 
                             QDoubleSpinBox, QSlider, QTextEdit)
from PyQt5.QtCore import pyqtSignal, Qt, QTimer
from PyQt5.QtGui import QImage, QPixmap, QFont

class RobotCellGUI(QMainWindow):
    # Events die we naar de buitenwereld (de bridge) communiceren
    start_pressed = pyqtSignal()
    stop_pressed = pyqtSignal()
    reset_pressed = pyqtSignal()
    home_pressed = pyqtSignal()
    confidence_changed = pyqtSignal(float)
    speed_changed = pyqtSignal(float)

    def __init__(self):
        super().__init__()
        self.setWindowTitle("FRS - HMI Sorteerinstallatie GES (S1)")
        self.resize(1200, 800)

        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)
        
        # Timer voor het knipperen van de noodstop-indicator
        self.estop_blink_timer = QTimer()
        self.estop_blink_timer.timeout.connect(self.toggle_estop_flash)
        self.estop_state_active = False
        self.estop_visible = False
        
        self.init_ui()

    def init_ui(self):
        main_layout = QHBoxLayout(self.central_widget)

        # --- LINKERKANT: Live Feeds ---
        left_feeds_layout = QVBoxLayout()

        # Camera feed
        self.camera_box = QGroupBox("Live OAK AI-Vision Feed")
        camera_layout = QVBoxLayout()
        self.image_label = QLabel(self)
        self.image_label.setAlignment(Qt.AlignCenter)
        self.image_label.setText("Wachten op AI-beeldstroom...")
        self.image_label.setStyleSheet("background-color: #1a1a1a; color: white;")
        camera_layout.addWidget(self.image_label)
        self.camera_box.setLayout(camera_layout)
        left_feeds_layout.addWidget(self.camera_box, stretch=3)

        # Live foutmeldingen direct onder de camera
        self.error_box = QGroupBox("Systeem Logboek & Foutmeldingen")
        error_layout = QVBoxLayout()
        self.txt_errors = QTextEdit()
        self.txt_errors.setReadOnly(True)
        self.txt_errors.setStyleSheet("background-color: #000000; color: #ff3333; font-family: Courier;")
        self.txt_errors.setFont(QFont("Courier", 10))
        error_layout.addWidget(self.txt_errors)
        self.error_box.setLayout(error_layout)
        left_feeds_layout.addWidget(self.error_box, stretch=1)

        main_layout.addLayout(left_feeds_layout, stretch=3)

        # --- RECHTERKANT: Status, Tellers, Parameters, Besturing ---
        right_layout = QVBoxLayout()

        # Noodstop Waarschuwingsindicator (Standaard verborgen)
        self.lbl_estop = QLabel("!!! NOODSTOP ACTIEF !!!")
        self.lbl_estop.setAlignment(Qt.AlignCenter)
        self.lbl_estop.setStyleSheet("background-color: #8B2E2E; color: white; font-size: 16px; font-weight: bold; border-radius: 8px; padding: 6px;")
        self.lbl_estop.hide()
        right_layout.addWidget(self.lbl_estop)

        # 1. Status display
        self.status_box = QGroupBox("Systeem Status (State Machine)")
        status_layout = QVBoxLayout()
        self.lbl_status = QLabel("Status: INIT")
        self.lbl_status.setStyleSheet("font-size: 20px; font-weight: bold; color: blue;")
        status_layout.addWidget(self.lbl_status)
        self.status_box.setLayout(status_layout)
        right_layout.addWidget(self.status_box)

        # 2. Sorteer Tellers (NU ECHT CORRECT AANGEPAST!)
        self.counter_box = QGroupBox("Sorteertellers")
        counter_grid = QGridLayout()
        self.lbl_bak1 = QLabel("Opzetstukjes: 0")
        self.lbl_bak2 = QLabel("Batterijen: 0")
        self.lbl_bak3 = QLabel("Bouten: 0")
        self.lbl_bak4 = QLabel("Pluggen: 0")
        counter_grid.addWidget(self.lbl_bak1, 0, 0)
        counter_grid.addWidget(self.lbl_bak2, 0, 1)
        counter_grid.addWidget(self.lbl_bak3, 1, 0)
        counter_grid.addWidget(self.lbl_bak4, 1, 1)
        self.counter_box.setLayout(counter_grid)
        right_layout.addWidget(self.counter_box)

        # 3. AI Tuning Parameters & Snelheid
        self.param_box = QGroupBox("AI & Snelheid Instellingen")
        param_layout = QGridLayout()

        # Confidence Threshold
        param_layout.addWidget(QLabel("Confidence Threshold:"), 0, 0)
        self.spin_confidence = QDoubleSpinBox()
        self.spin_confidence.setRange(0.0, 1.0)
        self.spin_confidence.setSingleStep(0.05)
        self.spin_confidence.setValue(0.85)
        self.spin_confidence.valueChanged.connect(self.confidence_changed.emit)
        param_layout.addWidget(self.spin_confidence, 0, 1)

        # Snelheidsregeling Slider
        param_layout.addWidget(QLabel("Robot Snelheid Limit:"), 1, 0)
        self.lbl_speed_val = QLabel("50%")
        param_layout.addWidget(self.lbl_speed_val, 1, 1)

        self.slider_speed = QSlider(Qt.Horizontal)
        self.slider_speed.setRange(0, 100)
        self.slider_speed.setValue(50)
        self.slider_speed.valueChanged.connect(self.on_speed_slider_moved)
        param_layout.addWidget(self.slider_speed, 2, 0, 1, 2)

        self.param_box.setLayout(param_layout)
        right_layout.addWidget(self.param_box)

        # 4. Knoppenpaneel (Originele Styles volledig behouden)
        self.control_box = QGroupBox("Besturing")
        control_grid = QGridLayout()
        btn_start = QPushButton("START")
        btn_stop = QPushButton("STOP")
        btn_reset = QPushButton("RESET")
        btn_home = QPushButton("MANUAL OVERRIDE: HOME")

        glass_green = '''
QPushButton {
  background-color: #5A8C5A;
  border: 2px solid #7AB07A;
  border-radius: 12px;
  padding: 10px 16px;
  font-size: 13px;
  font-weight: bold;
  color: white;
}
QPushButton:pressed {
  background-color: #3A5C3A;
}
'''
        glass_red = '''
QPushButton {
  background-color: #8B5A5A;
  border: 2px solid #A87A7A;
  border-radius: 12px;
  padding: 10px 16px;
  font-size: 13px;
  font-weight: bold;
  color: white;
}
QPushButton:pressed {
  background-color: #6B3A3A;
}
'''
        glass_blue = '''
QPushButton {
  background-color: #5A7A9A;
  border: 2px solid #7A9ABA;
  border-radius: 12px;
  padding: 10px 16px;
  font-size: 13px;
  font-weight: bold;
  color: white;
}
QPushButton:pressed {
  background-color: #3A5A7A;
}
'''
        glass_purple = '''
QPushButton {
  background-color: #8B5A9A;
  border: 2px solid #A87ABA;
  border-radius: 12px;
  padding: 10px 16px;
  font-size: 13px;
  font-weight: bold;
  color: white;
}
QPushButton:pressed {
  background-color: #6B3A7A;
}
'''

        btn_start.setStyleSheet(glass_green)
        btn_stop.setStyleSheet(glass_red)
        btn_reset.setStyleSheet(glass_blue)
        btn_home.setStyleSheet(glass_purple)

        btn_start.setMinimumHeight(44)
        btn_stop.setMinimumHeight(44)
        btn_reset.setMinimumHeight(44)
        btn_home.setMinimumHeight(44)

        btn_start.setCursor(Qt.PointingHandCursor)
        btn_stop.setCursor(Qt.PointingHandCursor)
        btn_reset.setCursor(Qt.PointingHandCursor)
        btn_home.setCursor(Qt.PointingHandCursor)

        btn_start.clicked.connect(self.start_pressed.emit)
        btn_stop.clicked.connect(self.stop_pressed.emit)
        btn_reset.clicked.connect(self.reset_pressed.emit)
        btn_home.clicked.connect(self.home_pressed.emit)

        control_grid.addWidget(btn_start, 0, 0)
        control_grid.addWidget(btn_stop, 0, 1)
        control_grid.addWidget(btn_reset, 1, 0, 1, 2)
        control_grid.addWidget(btn_home, 2, 0, 1, 2)
        self.control_box.setLayout(control_grid)
        right_layout.addWidget(self.control_box)

        main_layout.addLayout(right_layout, stretch=1)

    # --- INTERNE LOGICA ---
    def on_speed_slider_moved(self, value):
        self.lbl_speed_val.setText(f"{value}%")
        self.speed_changed.emit(value / 100.0)

    def toggle_estop_flash(self):
        """Knipperlogica voor de noodstop banner."""
        if self.estop_visible:
            self.lbl_estop.setStyleSheet("background-color: #1a1a1a; color: #ff3333; font-size: 16px; font-weight: bold; border-radius: 8px; padding: 6px; border: 1px solid #ff3333;")
            self.estop_visible = False
        else:
            self.lbl_estop.setStyleSheet("background-color: #ff3333; color: white; font-size: 16px; font-weight: bold; border-radius: 8px; padding: 6px;")
            self.estop_visible = True

    # --- SETTERS ---
    def set_camera_image(self, qt_img):
        pixmap = QPixmap.fromImage(qt_img)
        scaled_pixmap = pixmap.scaled(self.image_label.width(), self.image_label.height(), Qt.KeepAspectRatio)
        self.image_label.setPixmap(scaled_pixmap)

    def set_twin_image(self, qt_img):
        pass

    def set_system_state(self, state_text):
        self.lbl_status.setText(f"Status: {state_text}")
        
        # Noodstop detectie op basis van state machine data
        if state_text.upper() in ["EMERGENCY_STOP", "ESTOP", "EMERGENCY"]:
            if not self.estop_state_active:
                self.estop_state_active = True
                self.lbl_estop.show()
                self.estop_blink_timer.start(400) # Knippersnelheid (ms)
            self.lbl_status.setStyleSheet("font-size: 20px; font-weight: bold; color: red;")
        elif state_text.upper() in ["ERROR", "FAULT"]:
            self.stop_estop_blink()
            self.lbl_status.setStyleSheet("font-size: 20px; font-weight: bold; color: red;")
        elif state_text.upper() in ["RUNNING", "ACTIVE"]:
            self.stop_estop_blink()
            self.lbl_status.setStyleSheet("font-size: 20px; font-weight: bold; color: green;")
        else:
            self.stop_estop_blink()
            self.lbl_status.setStyleSheet("font-size: 20px; font-weight: bold; color: blue;")

    def stop_estop_blink(self):
        """Zet het knipperen uit en verbergt de banner veilig."""
        self.estop_state_active = False
        self.estop_blink_timer.stop()
        self.lbl_estop.hide()

    def set_product_counts(self, counts_array):
        if len(counts_array) == 4:
            self.lbl_bak1.setText(f"Oral-B Opzetstukjes: {counts_array[0]}")
            self.lbl_bak2.setText(f"AAA-Batterijen: {counts_array[1]}")
            self.lbl_bak3.setText(f"M6-Bouten: {counts_array[2]}")
            self.lbl_bak4.setText(f"Wandpluggen: {counts_array[3]}")

    def add_error_message(self, message_text):
        self.txt_errors.append(f"• {message_text}")
        self.txt_errors.ensureCursorVisible()