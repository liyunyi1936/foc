# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'main_ui.ui'
#
# Created by: PyQt5 UI code generator 5.15.4
#
# WARNING: Any manual changes made to this file will be lost when pyuic5 is
# run again.  Do not edit this file unless you know what you are doing.


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(1187, 707)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.groupBox = QtWidgets.QGroupBox(self.centralwidget)
        self.groupBox.setGeometry(QtCore.QRect(20, 460, 271, 61))
        self.groupBox.setObjectName("groupBox")
        self.horizontalLayoutWidget = QtWidgets.QWidget(self.groupBox)
        self.horizontalLayoutWidget.setGeometry(QtCore.QRect(10, 20, 251, 31))
        self.horizontalLayoutWidget.setObjectName("horizontalLayoutWidget")
        self.horizontalLayout = QtWidgets.QHBoxLayout(self.horizontalLayoutWidget)
        self.horizontalLayout.setContentsMargins(0, 0, 0, 0)
        self.horizontalLayout.setSpacing(1)
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.wifi_IP_lineEdit = QtWidgets.QLineEdit(self.horizontalLayoutWidget)
        self.wifi_IP_lineEdit.setObjectName("wifi_IP_lineEdit")
        self.horizontalLayout.addWidget(self.wifi_IP_lineEdit)
        self.wifi_config_pushButton = QtWidgets.QPushButton(self.horizontalLayoutWidget)
        self.wifi_config_pushButton.setObjectName("wifi_config_pushButton")
        self.horizontalLayout.addWidget(self.wifi_config_pushButton)
        self.groupBox_2 = QtWidgets.QGroupBox(self.centralwidget)
        self.groupBox_2.setGeometry(QtCore.QRect(310, 10, 811, 481))
        self.groupBox_2.setObjectName("groupBox_2")
        self.gridLayoutWidget = QtWidgets.QWidget(self.groupBox_2)
        self.gridLayoutWidget.setGeometry(QtCore.QRect(10, 50, 791, 371))
        self.gridLayoutWidget.setObjectName("gridLayoutWidget")
        self.gridLayout = QtWidgets.QGridLayout(self.gridLayoutWidget)
        self.gridLayout.setContentsMargins(0, 0, 0, 0)
        self.gridLayout.setObjectName("gridLayout")
        self.horizontalLayoutWidget_4 = QtWidgets.QWidget(self.groupBox_2)
        self.horizontalLayoutWidget_4.setGeometry(QtCore.QRect(10, 420, 791, 51))
        self.horizontalLayoutWidget_4.setObjectName("horizontalLayoutWidget_4")
        self.tool_layout = QtWidgets.QHBoxLayout(self.horizontalLayoutWidget_4)
        self.tool_layout.setContentsMargins(0, 0, 0, 0)
        self.tool_layout.setObjectName("tool_layout")
        self.horizontalLayoutWidget_6 = QtWidgets.QWidget(self.groupBox_2)
        self.horizontalLayoutWidget_6.setGeometry(QtCore.QRect(10, 20, 791, 31))
        self.horizontalLayoutWidget_6.setObjectName("horizontalLayoutWidget_6")
        self.tool_layout_2 = QtWidgets.QHBoxLayout(self.horizontalLayoutWidget_6)
        self.tool_layout_2.setContentsMargins(0, 0, 0, 0)
        self.tool_layout_2.setObjectName("tool_layout_2")
        self.groupBox_3 = QtWidgets.QGroupBox(self.centralwidget)
        self.groupBox_3.setGeometry(QtCore.QRect(10, 10, 291, 321))
        self.groupBox_3.setObjectName("groupBox_3")
        self.horizontalLayoutWidget_7 = QtWidgets.QWidget(self.groupBox_3)
        self.horizontalLayoutWidget_7.setGeometry(QtCore.QRect(10, 30, 271, 31))
        self.horizontalLayoutWidget_7.setObjectName("horizontalLayoutWidget_7")
        self.horizontalLayout_3 = QtWidgets.QHBoxLayout(self.horizontalLayoutWidget_7)
        self.horizontalLayout_3.setContentsMargins(0, 0, 0, 0)
        self.horizontalLayout_3.setObjectName("horizontalLayout_3")
        self.wifi_command_lineEdit_1 = QtWidgets.QLineEdit(self.horizontalLayoutWidget_7)
        self.wifi_command_lineEdit_1.setObjectName("wifi_command_lineEdit_1")
        self.horizontalLayout_3.addWidget(self.wifi_command_lineEdit_1)
        self.wifi_command_pushButton_1 = QtWidgets.QPushButton(self.horizontalLayoutWidget_7)
        self.wifi_command_pushButton_1.setObjectName("wifi_command_pushButton_1")
        self.horizontalLayout_3.addWidget(self.wifi_command_pushButton_1)
        self.horizontalLayoutWidget_8 = QtWidgets.QWidget(self.groupBox_3)
        self.horizontalLayoutWidget_8.setGeometry(QtCore.QRect(10, 60, 271, 31))
        self.horizontalLayoutWidget_8.setObjectName("horizontalLayoutWidget_8")
        self.horizontalLayout_4 = QtWidgets.QHBoxLayout(self.horizontalLayoutWidget_8)
        self.horizontalLayout_4.setContentsMargins(0, 0, 0, 0)
        self.horizontalLayout_4.setObjectName("horizontalLayout_4")
        self.wifi_command_lineEdit_2 = QtWidgets.QLineEdit(self.horizontalLayoutWidget_8)
        self.wifi_command_lineEdit_2.setObjectName("wifi_command_lineEdit_2")
        self.horizontalLayout_4.addWidget(self.wifi_command_lineEdit_2)
        self.wifi_command_pushButton_2 = QtWidgets.QPushButton(self.horizontalLayoutWidget_8)
        self.wifi_command_pushButton_2.setObjectName("wifi_command_pushButton_2")
        self.horizontalLayout_4.addWidget(self.wifi_command_pushButton_2)
        self.horizontalLayoutWidget_9 = QtWidgets.QWidget(self.groupBox_3)
        self.horizontalLayoutWidget_9.setGeometry(QtCore.QRect(10, 90, 271, 31))
        self.horizontalLayoutWidget_9.setObjectName("horizontalLayoutWidget_9")
        self.horizontalLayout_7 = QtWidgets.QHBoxLayout(self.horizontalLayoutWidget_9)
        self.horizontalLayout_7.setContentsMargins(0, 0, 0, 0)
        self.horizontalLayout_7.setObjectName("horizontalLayout_7")
        self.wifi_command_lineEdit_3 = QtWidgets.QLineEdit(self.horizontalLayoutWidget_9)
        self.wifi_command_lineEdit_3.setObjectName("wifi_command_lineEdit_3")
        self.horizontalLayout_7.addWidget(self.wifi_command_lineEdit_3)
        self.wifi_command_pushButton_3 = QtWidgets.QPushButton(self.horizontalLayoutWidget_9)
        self.wifi_command_pushButton_3.setObjectName("wifi_command_pushButton_3")
        self.horizontalLayout_7.addWidget(self.wifi_command_pushButton_3)
        self.groupBox_4 = QtWidgets.QGroupBox(self.centralwidget)
        self.groupBox_4.setGeometry(QtCore.QRect(310, 500, 591, 131))
        self.groupBox_4.setObjectName("groupBox_4")
        self.horizontalLayoutWidget_2 = QtWidgets.QWidget(self.groupBox_4)
        self.horizontalLayoutWidget_2.setGeometry(QtCore.QRect(10, 20, 571, 31))
        self.horizontalLayoutWidget_2.setObjectName("horizontalLayoutWidget_2")
        self.horizontalLayout_2 = QtWidgets.QHBoxLayout(self.horizontalLayoutWidget_2)
        self.horizontalLayout_2.setContentsMargins(0, 0, 0, 0)
        self.horizontalLayout_2.setObjectName("horizontalLayout_2")
        self.left_lineEdit_1 = QtWidgets.QLineEdit(self.horizontalLayoutWidget_2)
        self.left_lineEdit_1.setObjectName("left_lineEdit_1")
        self.horizontalLayout_2.addWidget(self.left_lineEdit_1)
        self.horizontalSlider_1 = QtWidgets.QSlider(self.horizontalLayoutWidget_2)
        self.horizontalSlider_1.setMinimum(-100)
        self.horizontalSlider_1.setMaximum(100)
        self.horizontalSlider_1.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalSlider_1.setTickPosition(QtWidgets.QSlider.TicksAbove)
        self.horizontalSlider_1.setTickInterval(10)
        self.horizontalSlider_1.setObjectName("horizontalSlider_1")
        self.horizontalLayout_2.addWidget(self.horizontalSlider_1)
        self.right_lineEdit_1 = QtWidgets.QLineEdit(self.horizontalLayoutWidget_2)
        self.right_lineEdit_1.setObjectName("right_lineEdit_1")
        self.horizontalLayout_2.addWidget(self.right_lineEdit_1)
        self.label = QtWidgets.QLabel(self.horizontalLayoutWidget_2)
        self.label.setObjectName("label")
        self.horizontalLayout_2.addWidget(self.label)
        self.command_lineEdit_1 = QtWidgets.QLineEdit(self.horizontalLayoutWidget_2)
        self.command_lineEdit_1.setObjectName("command_lineEdit_1")
        self.horizontalLayout_2.addWidget(self.command_lineEdit_1)
        self.label_4 = QtWidgets.QLabel(self.horizontalLayoutWidget_2)
        self.label_4.setObjectName("label_4")
        self.horizontalLayout_2.addWidget(self.label_4)
        self.num_lineEdit_1 = QtWidgets.QLineEdit(self.horizontalLayoutWidget_2)
        self.num_lineEdit_1.setObjectName("num_lineEdit_1")
        self.horizontalLayout_2.addWidget(self.num_lineEdit_1)
        self.horizontalLayout_2.setStretch(0, 1)
        self.horizontalLayout_2.setStretch(1, 18)
        self.horizontalLayout_2.setStretch(2, 1)
        self.horizontalLayout_2.setStretch(3, 1)
        self.horizontalLayout_2.setStretch(4, 1)
        self.horizontalLayout_2.setStretch(5, 1)
        self.horizontalLayout_2.setStretch(6, 1)
        self.horizontalLayoutWidget_3 = QtWidgets.QWidget(self.groupBox_4)
        self.horizontalLayoutWidget_3.setGeometry(QtCore.QRect(10, 50, 571, 31))
        self.horizontalLayoutWidget_3.setObjectName("horizontalLayoutWidget_3")
        self.horizontalLayout_5 = QtWidgets.QHBoxLayout(self.horizontalLayoutWidget_3)
        self.horizontalLayout_5.setContentsMargins(0, 0, 0, 0)
        self.horizontalLayout_5.setObjectName("horizontalLayout_5")
        self.left_lineEdit_2 = QtWidgets.QLineEdit(self.horizontalLayoutWidget_3)
        self.left_lineEdit_2.setObjectName("left_lineEdit_2")
        self.horizontalLayout_5.addWidget(self.left_lineEdit_2)
        self.horizontalSlider_2 = QtWidgets.QSlider(self.horizontalLayoutWidget_3)
        self.horizontalSlider_2.setMinimum(-100)
        self.horizontalSlider_2.setMaximum(100)
        self.horizontalSlider_2.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalSlider_2.setTickPosition(QtWidgets.QSlider.TicksAbove)
        self.horizontalSlider_2.setTickInterval(10)
        self.horizontalSlider_2.setObjectName("horizontalSlider_2")
        self.horizontalLayout_5.addWidget(self.horizontalSlider_2)
        self.right_lineEdit_2 = QtWidgets.QLineEdit(self.horizontalLayoutWidget_3)
        self.right_lineEdit_2.setObjectName("right_lineEdit_2")
        self.horizontalLayout_5.addWidget(self.right_lineEdit_2)
        self.label_5 = QtWidgets.QLabel(self.horizontalLayoutWidget_3)
        self.label_5.setObjectName("label_5")
        self.horizontalLayout_5.addWidget(self.label_5)
        self.command_lineEdit_2 = QtWidgets.QLineEdit(self.horizontalLayoutWidget_3)
        self.command_lineEdit_2.setObjectName("command_lineEdit_2")
        self.horizontalLayout_5.addWidget(self.command_lineEdit_2)
        self.label_6 = QtWidgets.QLabel(self.horizontalLayoutWidget_3)
        self.label_6.setObjectName("label_6")
        self.horizontalLayout_5.addWidget(self.label_6)
        self.num_lineEdit_2 = QtWidgets.QLineEdit(self.horizontalLayoutWidget_3)
        self.num_lineEdit_2.setObjectName("num_lineEdit_2")
        self.horizontalLayout_5.addWidget(self.num_lineEdit_2)
        self.horizontalLayout_5.setStretch(0, 1)
        self.horizontalLayout_5.setStretch(1, 18)
        self.horizontalLayout_5.setStretch(2, 1)
        self.horizontalLayout_5.setStretch(3, 1)
        self.horizontalLayout_5.setStretch(4, 1)
        self.horizontalLayout_5.setStretch(5, 1)
        self.horizontalLayout_5.setStretch(6, 1)
        self.horizontalLayoutWidget_5 = QtWidgets.QWidget(self.groupBox_4)
        self.horizontalLayoutWidget_5.setGeometry(QtCore.QRect(10, 80, 571, 31))
        self.horizontalLayoutWidget_5.setObjectName("horizontalLayoutWidget_5")
        self.horizontalLayout_6 = QtWidgets.QHBoxLayout(self.horizontalLayoutWidget_5)
        self.horizontalLayout_6.setContentsMargins(0, 0, 0, 0)
        self.horizontalLayout_6.setObjectName("horizontalLayout_6")
        self.left_lineEdit_3 = QtWidgets.QLineEdit(self.horizontalLayoutWidget_5)
        self.left_lineEdit_3.setObjectName("left_lineEdit_3")
        self.horizontalLayout_6.addWidget(self.left_lineEdit_3)
        self.horizontalSlider_3 = QtWidgets.QSlider(self.horizontalLayoutWidget_5)
        self.horizontalSlider_3.setMinimum(-100)
        self.horizontalSlider_3.setMaximum(100)
        self.horizontalSlider_3.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalSlider_3.setTickPosition(QtWidgets.QSlider.TicksAbove)
        self.horizontalSlider_3.setTickInterval(10)
        self.horizontalSlider_3.setObjectName("horizontalSlider_3")
        self.horizontalLayout_6.addWidget(self.horizontalSlider_3)
        self.right_lineEdit_3 = QtWidgets.QLineEdit(self.horizontalLayoutWidget_5)
        self.right_lineEdit_3.setObjectName("right_lineEdit_3")
        self.horizontalLayout_6.addWidget(self.right_lineEdit_3)
        self.label_7 = QtWidgets.QLabel(self.horizontalLayoutWidget_5)
        self.label_7.setObjectName("label_7")
        self.horizontalLayout_6.addWidget(self.label_7)
        self.command_lineEdit_3 = QtWidgets.QLineEdit(self.horizontalLayoutWidget_5)
        self.command_lineEdit_3.setObjectName("command_lineEdit_3")
        self.horizontalLayout_6.addWidget(self.command_lineEdit_3)
        self.label_8 = QtWidgets.QLabel(self.horizontalLayoutWidget_5)
        self.label_8.setObjectName("label_8")
        self.horizontalLayout_6.addWidget(self.label_8)
        self.num_lineEdit_3 = QtWidgets.QLineEdit(self.horizontalLayoutWidget_5)
        self.num_lineEdit_3.setObjectName("num_lineEdit_3")
        self.horizontalLayout_6.addWidget(self.num_lineEdit_3)
        self.horizontalLayout_6.setStretch(0, 1)
        self.horizontalLayout_6.setStretch(1, 18)
        self.horizontalLayout_6.setStretch(2, 1)
        self.horizontalLayout_6.setStretch(3, 1)
        self.horizontalLayout_6.setStretch(4, 1)
        self.horizontalLayout_6.setStretch(5, 1)
        self.horizontalLayout_6.setStretch(6, 1)
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 1187, 26))
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
        self.groupBox.setTitle(_translate("MainWindow", "本机Wifi_IP"))
        self.wifi_IP_lineEdit.setText(_translate("MainWindow", "192.168.4.2"))
        self.wifi_config_pushButton.setText(_translate("MainWindow", "设置"))
        self.groupBox_2.setTitle(_translate("MainWindow", "可视化图表"))
        self.groupBox_3.setTitle(_translate("MainWindow", "command命令"))
        self.wifi_command_pushButton_1.setText(_translate("MainWindow", "发送"))
        self.wifi_command_pushButton_2.setText(_translate("MainWindow", "发送"))
        self.wifi_command_pushButton_3.setText(_translate("MainWindow", "发送"))
        self.groupBox_4.setTitle(_translate("MainWindow", "滑条command命令"))
        self.left_lineEdit_1.setText(_translate("MainWindow", "0"))
        self.right_lineEdit_1.setText(_translate("MainWindow", "15"))
        self.label.setText(_translate("MainWindow", "command:"))
        self.command_lineEdit_1.setText(_translate("MainWindow", "K31"))
        self.label_4.setText(_translate("MainWindow", "num:"))
        self.num_lineEdit_1.setText(_translate("MainWindow", "0"))
        self.left_lineEdit_2.setText(_translate("MainWindow", "0"))
        self.right_lineEdit_2.setText(_translate("MainWindow", "3"))
        self.label_5.setText(_translate("MainWindow", "command:"))
        self.command_lineEdit_2.setText(_translate("MainWindow", "K32"))
        self.label_6.setText(_translate("MainWindow", "num:"))
        self.num_lineEdit_2.setText(_translate("MainWindow", "0"))
        self.left_lineEdit_3.setText(_translate("MainWindow", "0"))
        self.right_lineEdit_3.setText(_translate("MainWindow", "1.5"))
        self.label_7.setText(_translate("MainWindow", "command:"))
        self.command_lineEdit_3.setText(_translate("MainWindow", "K33"))
        self.label_8.setText(_translate("MainWindow", "num:"))
        self.num_lineEdit_3.setText(_translate("MainWindow", "0"))
