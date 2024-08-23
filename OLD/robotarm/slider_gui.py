import sys
import serial
from PyQt5.QtWidgets import QApplication, QMainWindow, QSlider, QLineEdit, QLabel
from PyQt5.QtCore import Qt

arduino = serial.Serial(port='/dev/ttyACM0', baudrate=115200, timeout=.1)

class Example(QMainWindow):

    def __init__(self):
        super().__init__()

        self.mySlider = QSlider(Qt.Horizontal, self)
        self.mySlider.setGeometry(60, 80, 200, 40)
        self.mySlider.setMaximum(20)
        self.mySlider.setMinimum(-20)
        self.mySlider.valueChanged[int].connect(self.changeValue)

        self.textbox = QLabel(self)
        self.textbox.setGeometry(140, 60, 40, 20)

        self.setGeometry(50,50,320,200)
        self.setWindowTitle("Checkbox Example")
        self.show()

    def changeValue(self, value):
        value = value * 0.1
        print(value)
        self.textbox.setText(str('{0:.2f}'.format(value).rstrip('0')))
        arduino.write(bytes(value, 'utf-8'))
                        

if __name__ == '__main__':
    app = QApplication(sys.argv)
    ex = Example()
    sys.exit(app.exec_())