import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QMessageBox
from main_ui import *
from wifi_udp import *
import threading    #引入并行
import numpy as np
import pyqtgraph as pg
import re

RED_COLOR = (255, 92, 92)
GREEN_COLOR = (57, 217, 138)
BLUE_COLOR = (91, 141, 236)
ORANGE_COLOR = (253, 172, 66)
YELLOW_COLOR = (255,255,51)
PURPLE_COLOR = (75,0,130)
MAROON_COLOR = (222,184,135)

class MyWindow(QMainWindow, Ui_MainWindow):
    def __init__(self, parent=None):
        super(MyWindow, self).__init__(parent)
        self.setupUi(self)
        # 变量初始化
        self.variable_init()
        # 设置实例
        self.CreateItems()
        # 设置信号与槽
        self.CreateSignalSlot()


    # 设置信号与槽
    def CreateSignalSlot(self):
        self.wifi_recv_open_pushButton.clicked.connect(self.wifi_recv_open_pushButton_clicked)
        self.velocity_horizontalSlider.valueChanged.connect(self.velocity_horizontalSlider_valueChanged)
        self.wifi_config_pushButton.clicked.connect(self.wifi_config_pushButton_clicked)

    # 设置实例
    def CreateItems(self):
        # 定时器-绘图刷新
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(20)
        # wifi udp
        self.udp = udp()
        # self.wifi_IP_lineEdit.setText(self.udp.user_ip)
    def variable_init(self):
        # 图表数据变量
        self.wifi_recv_flag = 0
        self.udp_data = 0
        self.target_velocity = 0
        self.now_velocity = 0
        self.close_flag = 1
        self.re_item = []
    def plot_init(self):
        # 绘图对象
        self.plotWidget = pg.PlotWidget()
        self.plotWidget.showGrid(x=True, y=True, alpha=0.5)
        # 图表可视化数组
        signalColors = [RED_COLOR, BLUE_COLOR, PURPLE_COLOR, YELLOW_COLOR,
                        MAROON_COLOR, ORANGE_COLOR, GREEN_COLOR]
        signalIcons = ['reddot', 'bluedot', 'purpledot', 'yellowdot', 'maroondot', 'orangedot', 'greendot']
        self.numberOfSamples = 300
        self.signalDataArrays = []
        self.signalPlots = []
        self.signalPlotFlags = []
        self.timeArray = np.arange(-self.numberOfSamples, 0, 1)
        for (sig, sigColor, tooltip) in zip(self.re_item, signalColors, self.re_item):
            # define signal plot data array
            self.signalDataArrays.append(np.zeros(self.numberOfSamples))
            # configure signal plot parameters
            signalPen = pg.mkPen(color=sigColor, width=1.5)
            self.signalPlots.append(pg.PlotDataItem(self.timeArray,
                                            self.signalDataArrays[-1],
                                            pen=signalPen, name=tooltip))
            self.plotWidget.addItem(self.signalPlots[-1])
            # is plotted flag
            self.signalPlotFlags.append(True)

        self.gridLayout.addWidget(self.plotWidget)

    # 滑条绑定
    def velocity_horizontalSlider_valueChanged(self):
        self.target_velocity = self.velocity_horizontalSlider.value()
        self.velocity_lineEdit.setText(str(self.target_velocity))
        self.udp.send_message(str(self.target_velocity))
        print(str(self.target_velocity))
    def wifi_config_pushButton_clicked(self):
        try:
            print(self.wifi_IP_lineEdit.text(),type(self.wifi_IP_lineEdit.text()))
            self.udp.udpClientSocket.bind((self.wifi_IP_lineEdit.text(), 2333))
            # 第一次接受数据，用于判断项目数，
            recv_data = self.udp.udpClientSocket.recv(1024)
            recv_data = recv_data.decode('utf-8')
            recv_data = recv_data[:-1]
            recv_data = recv_data.split(',')
            """处理接受的信息"""
            for i, data in enumerate(recv_data):
                self.re_item.append(''.join(re.split(r'[^A-Za-z]', data)))
            print(self.re_item)
            # 图表初始化
            self.plot_init()
            t1 = threading.Thread(target=self.udp_recv)
            t1.start()
            self.wifi_recv_open_pushButton.setEnabled(True)
        except:
            QMessageBox.critical(self, "错误", '该请求的地址无效')
    def wifi_recv_open_pushButton_clicked(self):
        if self.wifi_recv_flag == 0:
            # 打开wifi接收
            self.wifi_recv_flag = 1
            self.wifi_recv_open_pushButton.setText('关闭')
        else:
            self.wifi_recv_flag = 0
            self.wifi_recv_open_pushButton.setText('打开')
    def udp_recv(self):
        while self.close_flag:
            recv_data = self.udp.udpClientSocket.recv(1024)
            recv_data = recv_data.decode('utf-8')
            recv_data = recv_data[:-1]
            recv_data = recv_data.split(',')
            """处理接受的信息"""
            print(recv_data)
            for i, data in enumerate(recv_data):
                self.re_item.append(''.join(re.split(r'[^A-Za-z]', data)))
                data = re.findall(r"\d+\.?\d*", data)
                # print(i,data)

                self.signalDataArrays[i] = np.roll(self.signalDataArrays[i], -1)
                self.signalDataArrays[i][-1] = data[0]
                pass
    def update_plot(self):
        if self.wifi_recv_flag:
            for i, plotFlag in enumerate(self.signalPlotFlags):
                self.signalPlots[i].setData(self.timeArray, self.signalDataArrays[i])
                self.signalPlots[i].updateItems()
                self.signalPlots[i].sigPlotChanged.emit(self.signalPlots[i])

    def closeEvent(self, a0: QtGui.QCloseEvent) -> None:
        self.close_flag = 0
if __name__ == '__main__':
    app = QApplication(sys.argv)
    myWin = MyWindow()
    myWin.show()
    sys.exit(app.exec_())