# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'main_ui.ui'
#
# Created by: PyQt5 UI code generator 5.15.2
#
# WARNING: Any manual changes made to this file will be lost when pyuic5 is
# run again.  Do not edit this file unless you know what you are doing.


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(800, 600)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.velocity_horizontalSlider = QtWidgets.QSlider(self.centralwidget)
        self.velocity_horizontalSlider.setGeometry(QtCore.QRect(470, 420, 261, 22))
        self.velocity_horizontalSlider.setMinimum(-100)
        self.velocity_horizontalSlider.setMaximum(100)
        self.velocity_horizontalSlider.setOrientation(QtCore.Qt.Horizontal)
        self.velocity_horizontalSlider.setTickPosition(QtWidgets.QSlider.TicksAbove)
        self.velocity_horizontalSlider.setTickInterval(10)
        self.velocity_horizontalSlider.setObjectName("velocity_horizontalSlider")
        self.velocity_lineEdit = QtWidgets.QLineEdit(self.centralwidget)
        self.velocity_lineEdit.setGeometry(QtCore.QRect(580, 460, 113, 21))
        self.velocity_lineEdit.setObjectName("velocity_lineEdit")
        self.label = QtWidgets.QLabel(self.centralwidget)
        self.label.setGeometry(QtCore.QRect(520, 460, 51, 16))
        self.label.setObjectName("label")
        self.gridLayoutWidget = QtWidgets.QWidget(self.centralwidget)
        self.gridLayoutWidget.setGeometry(QtCore.QRect(180, 20, 581, 361))
        self.gridLayoutWidget.setObjectName("gridLayoutWidget")
        self.gridLayout = QtWidgets.QGridLayout(self.gridLayoutWidget)
        self.gridLayout.setContentsMargins(0, 0, 0, 0)
        self.gridLayout.setObjectName("gridLayout")
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 800, 26))
        self.menubar.setObjectName("menubar")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.label.setText(_translate("MainWindow", "速度："))
