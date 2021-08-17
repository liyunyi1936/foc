import sys
from PyQt5.QtWidgets import QApplication, QMainWindow
from main_ui import *
from wifi_udp import *
import threading    #引入并行
import numpy as np
import pyqtgraph as pg


class MyWindow(QMainWindow, Ui_MainWindow):
    def __init__(self, parent=None):
        super(MyWindow, self).__init__(parent)
        self.setupUi(self)
        self.v_list=[]
        self.v_list.append(np.zeros(300))
        self.timeArray = np.arange(-300, 0, 1)

        self.velocity = 0
        # 滑条
        self.velocity_lineEdit.setText(str(self.velocity_horizontalSlider.value()))
        self.velocity_horizontalSlider.valueChanged.connect(self.velocity_horizontalSlider_valueChanged)
        # wifi udp
        self.udp = udp()

        # 接收数据
        self.udp_data = ''
        t1 = threading.Thread(target=self.udp_recv)
        t1.start()
        # 绘图对象
        self.pw = pg.PlotWidget()
        self.curve = self.pw.plot(pen='y')
        self.gridLayout.addWidget(self.pw)
        # 定时器
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(50)



    # 滑条事件绑定
    def velocity_horizontalSlider_valueChanged(self):
        value = self.velocity_horizontalSlider.value()
        self.velocity_lineEdit.setText(str(value))
        # self.udp.send_message(str(value))
        # self.curve.updateItems()
        # self.curve.sigPlotChanged.emit(self.curve)

    def udp_recv(self):
        while True:
            recv_data = self.udp.udpClientSocket.recv(1024)
            recv_data = recv_data.decode('utf-8')
            self.udp_data = recv_data
    def update_plot(self):
        print(self.udp_data)
        self.v_list[0] = np.roll(self.v_list[0], -1)
        self.v_list[0][-1] = self.udp_data
        self.curve.setData(self.timeArray, self.v_list[0])  # 在绘图部件中绘制折线图
if __name__ == '__main__':
    app = QApplication(sys.argv)
    myWin = MyWindow()
    myWin.show()
    sys.exit(app.exec_())