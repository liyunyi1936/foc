from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *
import sys

'''pyqt5动态添加删除控件'''


class DynAddObject(QDialog):
    def __init__(self, parent=None):
        super(DynAddObject, self).__init__(parent)
        self.widgetList = []
        addButton = QPushButton(u"添加控件")
        delBUtton = QPushButton(u"删除控件")
        self.layout = QGridLayout()
        self.layout.addWidget(addButton, 1, 0)
        self.layout.addWidget(delBUtton, 2, 0)
        self.setLayout(self.layout)
        addButton.clicked.connect(self.add)
        delBUtton.clicked.connect(self.delete)

    def add(self):
        btncont = self.layout.count()
        widget = QPushButton(str(btncont - 1), self)
        self.layout.addWidget(widget)

    def delete(self):
        for i in range(self.layout.count())[2:]:
            self.layout.itemAt(i).widget().deleteLater()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    form = DynAddObject()
    form.show()
    app.exec_()
