#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Vector3

import sys
from PyQt5.QtCore import Qt

from PyQt5.QtWidgets import * 
from PyQt5.QtGui import *


class UIClass(QWidget):

    def __init__(self):

        super(UIClass, self).__init__()
        self.initUI()
        
        rospy.init_node('fertilization_visualisation', anonymous = False) 

        rospy.Subscriber('/uav2/fertilizer_stock', Vector3, self.data_cb)

        self.rate = rospy.Rate(10)


    def initUI(self):
        
        # stock
        Label_stock = QLabel()
        Label_stock.setText('fertilizer stock [g] : ')
        Label_stock.setFont(QFont('Arial', 18))

        self.LCD_stock = QLCDNumber(self)
        self.LCD_stock.display(1000) 

        layout_stock = QHBoxLayout()
        layout_stock.addWidget(Label_stock)
        layout_stock.addWidget(self.LCD_stock)

        # actual segment
        Label_segment= QLabel()
        Label_segment.setText('previous segment [g] : ')
        Label_segment.setFont(QFont('Arial', 18))

        self.LCD_segment = QLCDNumber(self)
        self.LCD_segment.display(0) 

        layout_segment = QHBoxLayout()
        layout_segment.addWidget(Label_segment)
        layout_segment.addWidget(self.LCD_segment)

        layout = QVBoxLayout()
        layout.addLayout(layout_stock)
        layout.addLayout(layout_segment)
        self.setLayout(layout)

        self.setGeometry(300, 300, 250, 150)
        self.setWindowTitle('Fertilization')
        self.show()


    def data_cb(self, data):

        self.LCD_stock.display(data.y)
        self.LCD_segment.display(data.z)

    
if __name__ == '__main__':

    app = QApplication(sys.argv)
    ex = UIClass()
    sys.exit(app.exec_())