#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64

import sys
from PyQt5.QtCore import Qt

from PyQt5.QtWidgets import * 
from PyQt5.QtGui import *


class UIClass(QWidget):

    def __init__(self):

        super(UIClass, self).__init__()
        self.initUI()
        
        rospy.init_node('fertilization_visualisation', anonymous = False) 

        rospy.Subscriber('fertilization', Float64, self.data_cb)

        self.rate = rospy.Rate(10)


    def initUI(self):

        Label = QLabel()
        Label.setText('Fertilization [%]')
        Label.setFont(QFont('Arial', 22))

        self.LCD = QLCDNumber(self)
        self.LCD.display(0) 

        layoutH = QHBoxLayout()
        layoutH.addWidget(Label)
        layoutH.addWidget(self.LCD)

        self.sl = QSlider(Qt.Horizontal)
        self.sl.setMinimum(10)
        self.sl.setMaximum(30)
        self.sl.setValue(20)
        self.sl.setTickPosition(QSlider.TicksBelow)
        self.sl.setTickInterval(5)

        layout = QVBoxLayout()
        layout.addLayout(layoutH)
        layout.addWidget(self.sl)
        self.setLayout(layout)

        self.setGeometry(300, 300, 250, 150)
        self.setWindowTitle('Fertilization Visualisation')
        self.show()


    def data_cb(self, data):

        self.LCD.display(data.data)
        self.sl.setValue(data.data)

    
if __name__ == '__main__':

    app = QApplication(sys.argv)
    ex = UIClass()
    sys.exit(app.exec_())