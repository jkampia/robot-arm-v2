#!/usr/bin/env python
# This Python file uses the following encoding: utf-8
import sys
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import QObject, QThread, pyqtSignal
from PyQt5.QtWidgets import QApplication, QListView
import interface
import serial
from serial.tools import list_ports
import os
import time

VENDOR_ID = "16C0"
PRODUCT_ID = "0483"
SERIAL_NUMBER = "12420850"

if os.path.exists("command_logs.txt"):
    os.remove("command_logs.txt")

class serial_comms(QObject):
    data = pyqtSignal(str)
    def run(self):
        while True:
            #print("Waiting for serial data...")
            time.sleep(1)

class Widget(QtWidgets.QDialog, interface.Ui_Dialog):

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setupUi(self)
        self.setWindowTitle("6DOF Arm - Manual Control")
        
        self.max_console_entries = 13
        self.joint_entries = [0, 0, 0, 0, 0, 0]
        self.ena_flags = [1, 1, 1, 1, 1, 1]
        self.home_pos = [174.39, 0, 348.05, 0, 90, 180]
        self.home_joints = [0, 0, 0, 0, 0, 0]
        self.pose_entries = self.home_pos
        self.connected = False
        self.QUEUE_ID = ""
        self.queue_filename = ""

        self.port = self.get_microcontroller_port()
        if self.connected:
            self.teensy = serial.Serial(self.port, 115200)
            print("Connected to teensy on port " + str(self.port))
        else:
            print("ERROR: Can't connect to teensy, is it connected?")

        self.command_logs = open("command_logs.txt", 'a')
        
        self.dial_1.valueChanged[int].connect(self.dial1_val_change)
        self.dial_2.valueChanged[int].connect(self.dial2_val_change)
        self.dial_3.valueChanged[int].connect(self.dial3_val_change)
        self.dial_4.valueChanged[int].connect(self.dial4_val_change)
        self.dial_5.valueChanged[int].connect(self.dial5_val_change)
        self.dial_6.valueChanged[int].connect(self.dial6_val_change)

        self.pushButton.setStyleSheet("background-color: green")
        self.pushButton.clicked.connect(self.button1_clicked)
        self.button1_ena = True
        self.pushButton_2.setStyleSheet("background-color: green")
        self.pushButton_2.clicked.connect(self.button2_clicked)
        self.button2_ena = True
        self.pushButton_3.setStyleSheet("background-color: green")
        self.pushButton_3.clicked.connect(self.button3_clicked)
        self.button3_ena = True
        self.pushButton_4.setStyleSheet("background-color: green")
        self.pushButton_4.clicked.connect(self.button4_clicked)
        self.button4_ena = True
        self.pushButton_5.setStyleSheet("background-color: green")
        self.pushButton_5.clicked.connect(self.button5_clicked)
        self.button5_ena = True
        self.pushButton_6.setStyleSheet("background-color: green")
        self.pushButton_6.clicked.connect(self.button6_clicked)
        self.button6_ena = True

        self.joints_lineedit_1.textChanged[str].connect(self.joint1_changed)
        self.joints_lineedit_2.textChanged[str].connect(self.joint2_changed)
        self.joints_lineedit_3.textChanged[str].connect(self.joint3_changed)
        self.joints_lineedit_4.textChanged[str].connect(self.joint4_changed)
        self.joints_lineedit_5.textChanged[str].connect(self.joint5_changed)
        self.joints_lineedit_6.textChanged[str].connect(self.joint6_changed)

        self.pose_lineedit_1.textChanged[str].connect(self.pose1_changed)
        self.pose_lineedit_2.textChanged[str].connect(self.pose2_changed)
        self.pose_lineedit_3.textChanged[str].connect(self.pose3_changed)
        self.pose_lineedit_4.textChanged[str].connect(self.pose4_changed)
        self.pose_lineedit_5.textChanged[str].connect(self.pose5_changed)
        self.pose_lineedit_6.textChanged[str].connect(self.pose6_changed)

        self.jog_joints.clicked.connect(self.jog_joints_clicked)
        self.jog_pose.clicked.connect(self.jog_pose_clicked)
        self.home.clicked.connect(self.home_clicked)
        self.start.clicked.connect(self.start_clicked)
        self.stop.clicked.connect(self.stop_clicked)
        self.add_joints.clicked.connect(self.add_joints_clicked)
        self.add_pose.clicked.connect(self.add_pose_clicked)
        self.clear_queue.clicked.connect(self.clear_queue_clicked)
        self.run_queue_button.clicked.connect(self.run_queue_clicked)
        self.load_queue.clicked.connect(self.load_queue_clicked)
        self.queue_file_entry.textChanged[str].connect(self.queue_file_changed)

        self.console_msg = ""
        self.console.setStyleSheet("background-color: white")
    
    def write_to_teensy(self, msg):
        if self.connected:
            self.teensy.write(msg.encode())

    def recv_from_teensy(self):
        self.thread = QThread()
        self.serial_comms = serial_comms()
        self.serial_comms.moveToThread(self.thread)
        self.thread.started.connect(self.serial_comms.run)
        self.thread.start()
    
    def get_microcontroller_port(self):
        for port in list(list_ports.comports()):
            if VENDOR_ID in port[2] and PRODUCT_ID in port[2] and SERIAL_NUMBER in port[2]:
                self.connected = True
                return port[0]
            else:
                self.connected = False

    def dial1_val_change(self, value):
        self.label_1.setText(str('{0:.2f}'.format(value*0.1).rstrip('0').rstrip('.')) + " rad/s")
        #msg = "<0:0:" + str(self.ena_flags[0]) + ":" + str('{0:.2f}'.format(value*0.1).rstrip('0').rstrip('.')) + ">"
    def dial2_val_change(self, value):
        self.label_2.setText(str('{0:.2f}'.format(value*0.1).rstrip('0').rstrip('.')) + " rad/s")
        #msg = "<0:1:" + str(self.ena_flags[1]) + ":" + str('{0:.2f}'.format(value*0.1).rstrip('0').rstrip('.')) + ">"
    def dial3_val_change(self, value):
        self.label_3.setText(str('{0:.2f}'.format(value*0.1).rstrip('0').rstrip('.')) + " rad/s")
        #msg = "<0:2:" + str(self.ena_flags[2]) + ":" + str('{0:.2f}'.format(value*0.1).rstrip('0').rstrip('.')) + ">"
    def dial4_val_change(self, value):
        self.label_4.setText(str('{0:.2f}'.format(value*0.1).rstrip('0').rstrip('.')) + " rad/s")
    def dial5_val_change(self, value):
        self.label_5.setText(str('{0:.2f}'.format(value*0.1).rstrip('0').rstrip('.')) + " rad/s")
    def dial6_val_change(self, value):
        self.label_6.setText(str('{0:.2f}'.format(value*0.1).rstrip('0').rstrip('.')) + " rad/s")

    def console_setnewtext(self, msg):
        self.console_msg = self.console_msg + msg + "\n"
        num_lines = len(self.console_msg.split('\n'))
        if num_lines > self.max_console_entries:
            temparr = self.console_msg.split('\n')
            self.console_msg = ""
            for i in range(1,self.max_console_entries-1):
                self.console_msg = self.console_msg + temparr[i] + "\n"
            self.console_msg = self.console_msg + msg + "\n"
        self.console.setText(self.console_msg)

    def button1_clicked(self):
        self.button1_ena = not self.button1_ena
        if self.button1_ena:
            self.pushButton.setStyleSheet("background-color: green")
            self.pushButton.setText("Enabled")
            self.console_setnewtext("Enabled joint 1")
            self.ena_flags[0] = 1
            msg = "<0:0:1:0>"
            self.write_to_teensy(msg)
        else:
            self.pushButton.setStyleSheet("background-color: red")
            self.pushButton.setText("Disabled")
            self.console_setnewtext("Disabled joint 1")
            self.ena_flags[0] = 0
            msg = "<0:0:0:0>"
            self.write_to_teensy(msg)
    def button2_clicked(self):
        self.button2_ena = not self.button2_ena
        if self.button2_ena:
            self.pushButton_2.setStyleSheet("background-color: green")
            self.pushButton_2.setText("Enabled")
            self.console_setnewtext("Enabled joint 2")
            self.ena_flags[1] = 1
            msg = "<0:1:1:0>"
            self.write_to_teensy(msg)
        else:
            self.pushButton_2.setStyleSheet("background-color: red")
            self.pushButton_2.setText("Disabled")
            self.console_setnewtext("Disabled joint 2")
            self.ena_flags[1] = 0
            msg = "<0:1:0:0>"
            self.write_to_teensy(msg)
    def button3_clicked(self):
        self.button3_ena = not self.button3_ena
        if self.button3_ena:
            self.pushButton_3.setStyleSheet("background-color: green")
            self.pushButton_3.setText("Enabled")
            self.console_setnewtext("Enabled joint 3")
            self.ena_flags[2] = 1
            msg = "<0:2:1:0>"
            self.write_to_teensy(msg)
        else:
            self.pushButton_3.setStyleSheet("background-color: red")
            self.pushButton_3.setText("Disabled")
            self.console_setnewtext("Disabled joint 3")
            self.ena_flags[2] = 0
            msg = "<0:2:0:0>"
            self.write_to_teensy(msg)
    def button4_clicked(self):
        self.button4_ena = not self.button4_ena
        if self.button4_ena:
            self.pushButton_4.setStyleSheet("background-color: green")
            self.pushButton_4.setText("Enabled")
            self.console_setnewtext("Enabled joint 4")
            msg = "<0:3:1:0>"
            self.write_to_teensy(msg)
        else:
            self.pushButton_4.setStyleSheet("background-color: red")
            self.pushButton_4.setText("Disabled")
            self.console_setnewtext("Disabled joint 4")
            msg = "<0:3:0:0>"
            self.write_to_teensy(msg)
    def button5_clicked(self):
        self.button5_ena = not self.button5_ena
        if self.button5_ena:
            self.pushButton_5.setStyleSheet("background-color: green")
            self.pushButton_5.setText("Enabled")
            self.console_setnewtext("Enabled joint 5")
            msg = "<0:4:1:0>"
            self.write_to_teensy(msg)
        else:
            self.pushButton_5.setStyleSheet("background-color: red")
            self.pushButton_5.setText("Disabled")
            self.console_setnewtext("Disabled joint 5")
            msg = "<0:4:0:0>"
            self.write_to_teensy(msg)
    def button6_clicked(self):
        self.button6_ena = not self.button6_ena
        if self.button6_ena:
            self.pushButton_6.setStyleSheet("background-color: green")
            self.pushButton_6.setText("Enabled")
            self.console_setnewtext("Enabled joint 6")
        else:
            self.pushButton_6.setStyleSheet("background-color: red")
            self.pushButton_6.setText("Disabled")
            self.console_setnewtext("Disabled joint 6")
    
    def joint1_changed(self, str):
        if (str != ""):
            self.joint_entries[0] = str
    def joint2_changed(self,str):
        if (str != ""):
            self.joint_entries[1] = str
    def joint3_changed(self,str):
        if (str != ""):
            self.joint_entries[2] = str
    def joint4_changed(self,str):
        if (str != ""):
            self.joint_entries[3] = str
    def joint5_changed(self,str):
        if (str != ""):
            self.joint_entries[4] = str
    def joint6_changed(self,str):
        if (str != ""):
            self.joint_entries[5] = str

    def pose1_changed(self, str):
        if (str != ""):
            self.pose_entries[0] = str
    def pose2_changed(self,str):
        if (str != ""):
            self.pose_entries[1] = str
    def pose3_changed(self,str):
        if (str != ""):
            self.pose_entries[2] = str
    def pose4_changed(self,str):
        if (str != ""):
            self.pose_entries[3] = str
    def pose5_changed(self,str):
        if (str != ""):
            self.pose_entries[4] = str
    def pose6_changed(self,str):
        if (str != ""):
            self.pose_entries[5] = str

    def jog_joints_clicked(self):
        self.console_setnewtext("Joints: " + str(self.joint_entries))
        self.command_logs.write("Joints: " + str(self.joint_entries) + "\n")
        msg = "<2:0:0:" + str(self.joint_entries[0]) + ":" + str(self.joint_entries[1]) + ":" + str(self.joint_entries[2]) + ":" + str(self.joint_entries[3]) + ":" + str(self.joint_entries[4]) + ":" + str(self.joint_entries[5]) + ">"
        print(msg)
        self.write_to_teensy(msg)

    def jog_pose_clicked(self):
        self.console_setnewtext("Pose: " + str(self.pose_entries))
        msg = "<1:0:0:" + str(self.pose_entries[0]) + ":" + str(self.pose_entries[1]) + ":" + str(self.pose_entries[2]) + ":" + str(self.pose_entries[3]) + ":" + str(self.pose_entries[4]) + ":" + str(self.pose_entries[5]) + ">"
        print(msg)
        self.write_to_teensy(msg)

    def home_clicked(self):
        msg = "<2:0:0:0:0:0:0:0:0>"
        print(msg)
        self.write_to_teensy(msg)

    def start_clicked(self):
        msg = "<3:start>"
        print(msg)
        self.write_to_teensy(msg)

    def stop_clicked(self):
        msg = "<3:stop>"
        print(msg)
        self.write_to_teensy(msg)

    def add_joints_clicked(self):
        self.console_setnewtext("Added: " + str(self.joint_entries))
        msg = "<4:" + str(self.joint_entries[0]) + ":" + str(self.joint_entries[1]) + ":" + str(self.joint_entries[2]) + ":" + str(self.joint_entries[3]) + ":" + str(self.joint_entries[4]) + ":" + str(self.joint_entries[5]) + ":0>"
        print(msg)
        self.write_to_teensy(msg)

    def add_pose_clicked(self):
        self.console_setnewtext("Added: " + str(self.pose_entries))
        msg = "<4:" + str(self.pose_entries[0]) + ":" + str(self.pose_entries[1]) + ":" + str(self.pose_entries[2]) + ":" + str(self.pose_entries[3]) + ":" + str(self.pose_entries[4]) + ":" + str(self.pose_entries[5]) + ":1>" 
        print(msg)
        self.write_to_teensy(msg)

    def clear_queue_clicked(self):
        self.console_setnewtext("Queue cleared.")
        msg = "<3:clearqueue>"
        self.QUEUE_ID = ""
        print(msg)
        self.write_to_teensy(msg)

    def run_queue_clicked(self):
        self.console_setnewtext("Executing based on current queue: " + self.QUEUE_ID)
        msg = "<3:runqueue>"
        print(msg)
        self.write_to_teensy(msg)

    def queue_file_changed(self, str):
        self.queue_filename = str

    def load_queue_clicked(self):
        print(os.getcwd())
        cwd = os.getcwd()
        found_file = False
        if "queuefiles" not in cwd:
            os.chdir("queuefiles")
        try:
            queuefile = open(self.queue_filename, 'r')
            found_file = True
        except:
            print ("Unable to find file " + self.queue_filename + " at directory " + os.getcwd())
        if found_file:
            Lines = queuefile.readlines()
            for line in Lines:
                print(line.strip())
                self.write_to_teensy(line)




if __name__ == "__main__":
    app = QApplication([])
    window = Widget()
    window.recv_from_teensy()
    window.show()
    sys.exit(app.exec())
