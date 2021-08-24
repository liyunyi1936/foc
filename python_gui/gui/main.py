import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QMessageBox
from main_ui import *
from wifi_udp import *
import threading    #引入并行
import numpy as np
import pyqtgraph as pg


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
        # 图表初始化
        self.plot_init()

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
        self.timer.start(50)
        # wifi udp
        self.udp = udp()
        self.wifi_IP_lineEdit.setText(self.udp.user_ip)
    def variable_init(self):
        # 图表数据变量
        self.wifi_recv_flag = 0
        self.udp_data = 0
        self.target_velocity = 0
        self.now_velocity = 0

    def plot_init(self):
        # 图表可视化数组
        self.v_list = []
        self.v_list.append(np.zeros(300))
        self.timeArray = np.arange(-300, 0, 1)
        # 绘图对象
        self.pw = pg.PlotWidget()
        self.curve = self.pw.plot(pen='y')
        self.gridLayout.addWidget(self.pw)

    # 滑条绑定
    def velocity_horizontalSlider_valueChanged(self):
        self.target_velocity = self.velocity_horizontalSlider.value()
        self.velocity_lineEdit.setText(str(self.target_velocity))
        self.udp.send_message(str(self.velocity_lineEdit))

    def wifi_config_pushButton_clicked(self):
        try:
            self.udp.udpClientSocket.bind((self.wifi_IP_lineEdit.text(), 2333))
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
        while self.wifi_recv_flag:
            recv_data = self.udp.udpClientSocket.recv(1024)
            recv_data = recv_data.decode('utf-8')
            self.udp_data = recv_data
    def update_plot(self):
        if self.wifi_recv_flag:
            self.v_list[0] = np.roll(self.v_list[0], -1)
            self.v_list[0][-1] = self.udp_data
            self.curve.setData(self.timeArray, self.v_list[0])  # 在绘图部件中绘制折线图
if __name__ == '__main__':
    app = QApplication(sys.argv)
    myWin = MyWindow()
    myWin.show()
    sys.exit(app.exec_())