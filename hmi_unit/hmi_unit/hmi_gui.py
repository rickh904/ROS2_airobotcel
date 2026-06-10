# hmi_gui.py
from PyQt5.QtWidgets import (QMainWindow, QWidget, QLabel, QPushButton, 
                             QVBoxLayout, QHBoxLayout, QGridLayout, QGroupBox, QDoubleSpinBox)
from PyQt5.QtCore import pyqtSignal, Qt
from PyQt5.QtGui import QImage, QPixmap

class RobotCellGUI(QMainWindow):
    # Events die we naar de buitenwereld (de bridge) communiceren
    start_pressed = pyqtSignal()
    stop_pressed = pyqtSignal()
    reset_pressed = pyqtSignal()
    home_pressed = pyqtSignal()
    confidence_changed = pyqtSignal(float)

    def __init__(self):
        super().__init__()
        self.setWindowTitle("FRS - HMI Sorteerinstallatie GES (S1)")
        self.resize(1100, 650)

        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)
        self.init_ui()

    def init_ui(self):
        main_layout = QHBoxLayout(self.central_widget)

        # --- LINKERKANT: Live AI-Vision Camera ---
        self.camera_box = QGroupBox("Live OAK AI-Vision Feed")
        camera_layout = QVBoxLayout()
        self.image_label = QLabel(self)
        self.image_label.setAlignment(Qt.AlignCenter)
        self.image_label.setText("Wachten op beeldstroom...")
        self.image_label.setStyleSheet("background-color: #1a1a1a; color: white;")
        camera_layout.addWidget(self.image_label)
        self.camera_box.setLayout(camera_layout)
        main_layout.addWidget(self.camera_box, stretch=3)

        # --- RECHTERKANT: Status, Tellers en Knoppen ---
        right_layout = QVBoxLayout()

        # 1. Status display
        self.status_box = QGroupBox("Systeem Status (State Machine)")
        status_layout = QVBoxLayout()
        self.lbl_status = QLabel("Status: INIT")
        self.lbl_status.setStyleSheet("font-size: 20px; font-weight: bold; color: blue;")
        status_layout.addWidget(self.lbl_status)
        self.status_box.setLayout(status_layout)
        right_layout.addWidget(self.status_box)

        # 2. Sorteer Tellers (4 productcategorieën)
        self.counter_box = QGroupBox("Sorteertellers")
        counter_grid = QGridLayout()
        self.lbl_bak1 = QLabel("Bak 1 (Categorie 1): 0")
        self.lbl_bak2 = QLabel("Bak 2 (Categorie 2): 0")
        self.lbl_bak3 = QLabel("Bak 3 (Categorie 3): 0")
        self.lbl_bak4 = QLabel("Bak 4 (Categorie 4): 0")
        counter_grid.addWidget(self.lbl_bak1, 0, 0)
        counter_grid.addWidget(self.lbl_bak2, 0, 1)
        counter_grid.addWidget(self.lbl_bak3, 1, 0)
        counter_grid.addWidget(self.lbl_bak4, 1, 1)
        self.counter_box.setLayout(counter_grid)
        right_layout.addWidget(self.counter_box)

        # 3. AI Tuning Parameters
        self.param_box = QGroupBox("AI Instellingen")
        param_layout = QHBoxLayout()
        param_layout.addWidget(QLabel("Confidence Threshold:"))
        self.spin_confidence = QDoubleSpinBox()
        self.spin_confidence.setRange(0.0, 1.0)
        self.spin_confidence.setSingleStep(0.05)
        self.spin_confidence.setValue(0.85)
        self.spin_confidence.valueChanged.connect(self.confidence_changed.emit)
        param_layout.addWidget(self.spin_confidence)
        self.param_box.setLayout(param_layout)
        right_layout.addWidget(self.param_box)

        # 4. Knoppenpaneel
        self.control_box = QGroupBox("Besturing")
        control_grid = QGridLayout()
        btn_start = QPushButton("START")
        btn_stop = QPushButton("STOP")
        btn_reset = QPushButton("RESET ERROR")
        btn_home = QPushButton("MANUAL OVERRIDE: HOME")

        btn_start.setStyleSheet("background-color: green; color: white; font-weight: bold;")
        btn_stop.setStyleSheet("background-color: orange; color: black; font-weight: bold;")
        btn_home.setStyleSheet("background-color: blue; color: white;")

        # Verbind knop-clicks direct met de PyQt Signals
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

    # --- SETTERS (Aan te roepen door de Bridge of Unit-tests) ---
    def set_camera_image(self, qt_img):
        pixmap = QPixmap.fromImage(qt_img)
        scaled_pixmap = pixmap.scaled(self.image_label.width(), self.image_label.height(), Qt.KeepAspectRatio)
        self.image_label.setPixmap(scaled_pixmap)

    def set_system_state(self, state_text):
        self.lbl_status.setText(f"Status: {state_text}")
        if state_text.upper() in ["ERROR", "FAULT"]:
            self.lbl_status.setStyleSheet("font-size: 20px; font-weight: bold; color: red;")
        elif state_text.upper() in ["RUNNING", "ACTIVE"]:
            self.lbl_status.setStyleSheet("font-size: 20px; font-weight: bold; color: green;")
        else:
            self.lbl_status.setStyleSheet("font-size: 20px; font-weight: bold; color: blue;")

    def set_product_counts(self, counts_array):
        if len(counts_array) == 4:
            self.lbl_bak1.setText(f"Bak 1 (Categorie 1): {counts_array[0]}")
            self.lbl_bak2.setText(f"Bak 2 (Categorie 2): {counts_array[1]}")
            self.lbl_bak3.setText(f"Bak 3 (Categorie 3): {counts_array[2]}")
            self.lbl_bak4.setText(f"Bak 4 (Categorie 4): {counts_array[3]}")