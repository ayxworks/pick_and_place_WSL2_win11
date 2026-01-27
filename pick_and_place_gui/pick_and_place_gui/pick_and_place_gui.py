#!/usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from PyQt5.QtWidgets import (QApplication, QMainWindow, QPushButton, 
                             QVBoxLayout, QWidget, QLabel, QTextEdit)
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QFont

class PickPlaceGUI(QMainWindow):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.init_ui()
        
        # Timer to process ROS events
        self.ros_timer = QTimer()
        self.ros_timer.timeout.connect(self.spin_ros)
        self.ros_timer.start(100)  # 10 Hz
        
    def init_ui(self):
        self.setWindowTitle('Pick & Place Control')
        self.setGeometry(100, 100, 400, 300)
        
        # Central widget and layout
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        layout = QVBoxLayout(central_widget)
        
        # Title
        title = QLabel('Pick & Place Controller')
        title.setAlignment(Qt.AlignCenter)
        title.setFont(QFont('Arial', 16, QFont.Bold))
        layout.addWidget(title)
        
        # Status label
        self.status_label = QLabel('Status: Ready')
        self.status_label.setAlignment(Qt.AlignCenter)
        self.status_label.setFont(QFont('Arial', 12))
        layout.addWidget(self.status_label)
        
        # START button
        self.start_button = QPushButton('START')
        self.start_button.setMinimumHeight(50)
        self.start_button.setStyleSheet("""
            QPushButton {
                background-color: #4CAF50;
                color: white;
                font-size: 16px;
                font-weight: bold;
                border-radius: 5px;
            }
            QPushButton:hover {
                background-color: #45a049;
            }
            QPushButton:disabled {
                background-color: #cccccc;
            }
        """)
        self.start_button.clicked.connect(self.call_start)
        layout.addWidget(self.start_button)
        
        # STOP button
        self.stop_button = QPushButton('STOP')
        self.stop_button.setMinimumHeight(50)
        self.stop_button.setStyleSheet("""
            QPushButton {
                background-color: #f44336;
                color: white;
                font-size: 16px;
                font-weight: bold;
                border-radius: 5px;
            }
            QPushButton:hover {
                background-color: #da190b;
            }
            QPushButton:disabled {
                background-color: #cccccc;
            }
        """)
        self.stop_button.clicked.connect(self.call_stop)
        self.stop_button.setEnabled(False)
        layout.addWidget(self.stop_button)
        
        # Message log
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setMaximumHeight(100)
        layout.addWidget(QLabel('Log:'))
        layout.addWidget(self.log_text)
        
    def spin_ros(self):
        rclpy.spin_once(self.node, timeout_sec=0)
        
    def log_message(self, message):
        self.log_text.append(message)
        
    def call_start(self):
        self.start_button.setEnabled(False)
        self.status_label.setText('Status: Starting...')
        self.log_message('Calling start service...')
        
        request = Trigger.Request()
        future = self.node.start_client.call_async(request)
        future.add_done_callback(self.handle_start_response)
        
    def handle_start_response(self, future):
        try:
            response = future.result()
            if response.success:
                self.status_label.setText('Status: Running')
                self.log_message(f'✓ {response.message}')
                self.stop_button.setEnabled(True)
            else:
                self.status_label.setText('Status: Error')
                self.log_message(f'✗ {response.message}')
                self.start_button.setEnabled(True)
        except Exception as e:
            self.status_label.setText('Status: Error')
            self.log_message(f'✗ Error: {str(e)}')
            self.start_button.setEnabled(True)
            
    def call_stop(self):
        self.stop_button.setEnabled(False)
        self.status_label.setText('Status: Stopping...')
        self.log_message('Calling stop service...')
        
        request = Trigger.Request()
        future = self.node.stop_client.call_async(request)
        future.add_done_callback(self.handle_stop_response)
        
    def handle_stop_response(self, future):
        try:
            response = future.result()
            if response.success:
                self.status_label.setText('Status: Stopped')
                self.log_message(f'✓ {response.message}')
                self.start_button.setEnabled(True)
            else:
                self.status_label.setText('Status: Error')
                self.log_message(f'✗ {response.message}')
                self.stop_button.setEnabled(True)
        except Exception as e:
            self.status_label.setText('Status: Error')
            self.log_message(f'✗ Error: {str(e)}')
            self.stop_button.setEnabled(True)
            
    def closeEvent(self, event):
        self.ros_timer.stop()
        event.accept()


class PickPlaceGUINode(Node):
    def __init__(self):
        super().__init__('pick_place_gui')
        
        # Create service clients
        self.start_client = self.create_client(Trigger, 'start_pick_place')
        self.stop_client = self.create_client(Trigger, 'stop_pick_place')
        
        # Wait for services to be available
        self.get_logger().info('Waiting for services...')
        self.start_client.wait_for_service(timeout_sec=5.0)
        self.stop_client.wait_for_service(timeout_sec=5.0)
        self.get_logger().info('Services available')


def main(args=None):
    rclpy.init(args=args)
    
    # Create ROS node
    node = PickPlaceGUINode()
    
    # Create Qt application
    app = QApplication(sys.argv)
    gui = PickPlaceGUI(node)
    gui.show()
    
    # Execute application
    exit_code = app.exec_()
    
    # Cleanup
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == '__main__':
    main()