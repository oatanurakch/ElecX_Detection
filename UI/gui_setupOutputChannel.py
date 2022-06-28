# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'setupOutputChannel.ui'
#
# Created by: PyQt5 UI code generator 5.15.4
#
# WARNING: Any manual changes made to this file will be lost when pyuic5 is
# run again.  Do not edit this file unless you know what you are doing.


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(365, 588)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.gridLayout = QtWidgets.QGridLayout(self.centralwidget)
        self.gridLayout.setObjectName("gridLayout")
        self.gridLayout_2 = QtWidgets.QGridLayout()
        self.gridLayout_2.setObjectName("gridLayout_2")
        self.verticalLayout_3 = QtWidgets.QVBoxLayout()
        self.verticalLayout_3.setObjectName("verticalLayout_3")
        self.label_3 = QtWidgets.QLabel(self.centralwidget)
        font = QtGui.QFont()
        font.setPointSize(10)
        self.label_3.setFont(font)
        self.label_3.setAlignment(QtCore.Qt.AlignCenter)
        self.label_3.setObjectName("label_3")
        self.verticalLayout_3.addWidget(self.label_3)
        self.NGList = QtWidgets.QListWidget(self.centralwidget)
        font = QtGui.QFont()
        font.setPointSize(10)
        self.NGList.setFont(font)
        self.NGList.setObjectName("NGList")
        self.verticalLayout_3.addWidget(self.NGList)
        self.gridLayout_2.addLayout(self.verticalLayout_3, 1, 3, 1, 1)
        self.verticalLayout = QtWidgets.QVBoxLayout()
        self.verticalLayout.setObjectName("verticalLayout")
        self.label = QtWidgets.QLabel(self.centralwidget)
        font = QtGui.QFont()
        font.setPointSize(10)
        self.label.setFont(font)
        self.label.setAlignment(QtCore.Qt.AlignCenter)
        self.label.setObjectName("label")
        self.verticalLayout.addWidget(self.label)
        self.ChannelList = QtWidgets.QListWidget(self.centralwidget)
        font = QtGui.QFont()
        font.setPointSize(10)
        self.ChannelList.setFont(font)
        self.ChannelList.setObjectName("ChannelList")
        self.verticalLayout.addWidget(self.ChannelList)
        self.gridLayout_2.addLayout(self.verticalLayout, 1, 1, 1, 1)
        self.AddtoNG = QtWidgets.QPushButton(self.centralwidget)
        self.AddtoNG.setObjectName("AddtoNG")
        self.gridLayout_2.addWidget(self.AddtoNG, 2, 3, 1, 1)
        self.Confirm = QtWidgets.QPushButton(self.centralwidget)
        self.Confirm.setObjectName("Confirm")
        self.gridLayout_2.addWidget(self.Confirm, 5, 3, 1, 1)
        self.verticalLayout_2 = QtWidgets.QVBoxLayout()
        self.verticalLayout_2.setObjectName("verticalLayout_2")
        self.label_2 = QtWidgets.QLabel(self.centralwidget)
        font = QtGui.QFont()
        font.setPointSize(10)
        self.label_2.setFont(font)
        self.label_2.setAlignment(QtCore.Qt.AlignCenter)
        self.label_2.setObjectName("label_2")
        self.verticalLayout_2.addWidget(self.label_2)
        self.OKList = QtWidgets.QListWidget(self.centralwidget)
        font = QtGui.QFont()
        font.setPointSize(10)
        self.OKList.setFont(font)
        self.OKList.setObjectName("OKList")
        self.verticalLayout_2.addWidget(self.OKList)
        self.gridLayout_2.addLayout(self.verticalLayout_2, 1, 2, 1, 1)
        self.RemovefromOK = QtWidgets.QPushButton(self.centralwidget)
        self.RemovefromOK.setObjectName("RemovefromOK")
        self.gridLayout_2.addWidget(self.RemovefromOK, 3, 2, 1, 1)
        self.RemovefromNG = QtWidgets.QPushButton(self.centralwidget)
        self.RemovefromNG.setObjectName("RemovefromNG")
        self.gridLayout_2.addWidget(self.RemovefromNG, 3, 3, 1, 1)
        self.AddtoOK = QtWidgets.QPushButton(self.centralwidget)
        self.AddtoOK.setObjectName("AddtoOK")
        self.gridLayout_2.addWidget(self.AddtoOK, 2, 2, 1, 1)
        self.horizontalLayout = QtWidgets.QHBoxLayout()
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.label_4 = QtWidgets.QLabel(self.centralwidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Maximum, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.label_4.sizePolicy().hasHeightForWidth())
        self.label_4.setSizePolicy(sizePolicy)
        font = QtGui.QFont()
        font.setPointSize(10)
        self.label_4.setFont(font)
        self.label_4.setObjectName("label_4")
        self.horizontalLayout.addWidget(self.label_4)
        self.Currentmodule = QtWidgets.QLabel(self.centralwidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.Currentmodule.sizePolicy().hasHeightForWidth())
        self.Currentmodule.setSizePolicy(sizePolicy)
        font = QtGui.QFont()
        font.setPointSize(10)
        self.Currentmodule.setFont(font)
        self.Currentmodule.setText("")
        self.Currentmodule.setObjectName("Currentmodule")
        self.horizontalLayout.addWidget(self.Currentmodule)
        self.gridLayout_2.addLayout(self.horizontalLayout, 0, 1, 1, 2)
        self.gridLayout.addLayout(self.gridLayout_2, 0, 0, 1, 1)
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 365, 21))
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
        self.label_3.setText(_translate("MainWindow", "NG"))
        self.label.setText(_translate("MainWindow", "Channel"))
        self.AddtoNG.setText(_translate("MainWindow", "Add to NG"))
        self.Confirm.setText(_translate("MainWindow", "Confirm"))
        self.label_2.setText(_translate("MainWindow", "OK"))
        self.RemovefromOK.setText(_translate("MainWindow", "Remove from OK"))
        self.RemovefromNG.setText(_translate("MainWindow", "Remove from NG"))
        self.AddtoOK.setText(_translate("MainWindow", "Add to OK"))
        self.label_4.setText(_translate("MainWindow", "Used:"))


if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    MainWindow.show()
    sys.exit(app.exec_())