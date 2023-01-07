'''
Write a program that makes a button that when pressed will run a python script called test.py
'''

#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QToolTip, QMessageBox
from PyQt5.QtGui import QIcon, QFont
from PyQt5.QtCore import QCoreApplication

class Example(QWidget):
    
        def __init__(self):
            super().__init__()
    
            self.initUI()
    
        def initUI(self):
    
            QToolTip.setFont(QFont('SansSerif', 10))
    
            self.setToolTip('This is a <b>QWidget</b> widget')
    
            btn = QPushButton('Button', self)
            btn.setToolTip('This is a <b>QPushButton</b> widget')
            btn.resize(btn.sizeHint())
            btn.move(10, 50)
            #run test.py when button is pressed
            btn.clicked.connect(self.run_test)
            
    
            qbtn = QPushButton('Quit', self)
            qbtn.clicked.connect(QCoreApplication.instance().quit)
            qbtn.clicked.connect(self.closeEvent)
            qbtn.resize(qbtn.sizeHint())
            qbtn.move(50, 100)
    
            self.setGeometry(200, 200, 200, 200)
            self.setWindowTitle('Tooltips')
            self.setWindowIcon(QIcon('web.png'))
    
            self.show()
    
        def run_test(self):
            #run python test.py program
            exec(open("test.py").read())
            print("test complete")


        def closeEvent(self, event):
    
            reply = QMessageBox.question(self, 'Message',
                "Are you sure to quit?", QMessageBox.Yes |
                QMessageBox.No, QMessageBox.No)
    
            if reply == QMessageBox.Yes:
                event.accept()
            else:
                event.ignore()

if __name__ == '__main__':
    
        app = QApplication(sys.argv)
        ex = Example()
        sys.exit(app.exec_())






