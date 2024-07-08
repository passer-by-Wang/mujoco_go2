
from PySide6.QtWidgets import QMainWindow, QApplication, QVBoxLayout, QPushButton, QVBoxLayout, QLineEdit, QGridLayout
from matplotlib.figure import Figure
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg, NavigationToolbar2QT as NavigationToolbar
from PySide6.QtCore import QSharedMemory, QSize, QBuffer
from PySide6 import QtWidgets, QtCore
import sys
import matplotlib
import pandas as pd

matplotlib.use('Qt5Agg')


class AddMplTab(QtWidgets.QWidget):

    def __init__(self, plot, parent=None):
        super(AddMplTab, self).__init__(parent)
        self.layout1 = QVBoxLayout()
        self.toolbar = NavigationToolbar(plot, self)
        self.setLayout(self.layout1)
        self.layout1.addWidget(self.toolbar)
        self.layout1.addWidget(plot)


class MplCanvas(FigureCanvasQTAgg):

    def __init__(self, parent=None, width=5, height=4, dpi=100):
        fig = Figure(figsize=(width, height), dpi=dpi)
        self.axes = fig.add_subplot(111)
        super(MplCanvas, self).__init__(fig)


class MainWindow(QtWidgets.QMainWindow):

    def __init__(self):
        super().__init__()

        # Create the maptlotlib FigureCanvas object,
        # which defines a single set of axes as self.axes.
        self.setMainWidget()
        self.setPlotLayout()
        self.setControlWidget()
        self.setCentralWidget(self.mainTabwidget)
        self.LineEdit.returnPressed.connect(self.plot)
        self.SharedMemory()

    def setMainWidget(self):
        self.mainTabwidget = QtWidgets.QTabWidget()
        self.plotWidget = QtWidgets.QWidget()
        self.controlWidget = QtWidgets.QWidget()

        self.mainTabwidget.addTab(self.controlWidget, 'ui')
        self.mainTabwidget.addTab(self.plotWidget, 'plot')

    def setControlWidget(self):
        self.ControlLayout = QGridLayout()
        self.ServoOnButton = QPushButton('Servo On')
        self.ServoOnButton.setFixedSize(QSize(150, 150))
        self.ControlLayout.addWidget(self.ServoOnButton, 1, 1)

        self.HomeOnButton = QPushButton('Home On')
        self.HomeOnButton.setFixedSize(QSize(150, 150))
        self.ControlLayout.addWidget(self.HomeOnButton, 1, 2)

        self.controlWidget.setLayout(self.ControlLayout)

    def setPlotLayout(self):
        self.tabwidget = QtWidgets.QTabWidget()
        self.LineEdit = QLineEdit()

        self.PlotLayout = QVBoxLayout()
        self.PlotLayout.addWidget(self.LineEdit)
        self.PlotLayout.addWidget(self.tabwidget)

        self.plotWidget.setLayout(self.PlotLayout)

    def SharedMemory(self):
        self.buffer = QBuffer()
        self.shm = QSharedMemory("SharedMem_key")
        self.shm.attach(QSharedMemory.ReadWrite)
        self.buffer.open(QBuffer.ReadWrite)
        self.shm.lock()

        # self.buffer.write()
        self.shm.unlock()
        print(self.buffer.data())

    def plot(self):
        path = '/home/wh/simulation/bitbot_mujoco_galileo_2/log/'
        data = pd.read_csv(path + self.LineEdit.text())
        self.tabwidget.clear()
        pi = 3.1415926
        ###############
        sc = MplCanvas(self)
        sc.axes.plot(data.iloc[1:, 0]*180/pi, color='r', label='CoMPos')

        sc.axes.legend(loc='upper left')
        
        # add tab to widget
        self.tab1 = AddMplTab(sc)
        self.tabwidget.addTab(self.tab1, "CoMPos")

        sc = MplCanvas(self)
        sc.axes.plot(data.iloc[1:, 7]*180/pi, color='r', label='q')
        sc.axes1 = sc.axes.twinx()
        sc.axes1.plot(data.iloc[1:, 39+1], 'b-*', label='t_mpc')
        sc.axes1.plot(data.iloc[1:, 146+1], 'g-*', label='t_pd')
        sc.axes1.plot(data.iloc[1:, 152+1], 'y-*', label='t_wbc')
        sc.axes2 = sc.axes.twinx()
        sc.axes2.plot(-data.iloc[1:, 69], '-', label='state')
        sc.axes1.legend(loc='upper left')
        # add tab to widget
        self.tab2 = AddMplTab(sc)
        self.tabwidget.addTab(self.tab2, "pitch")

        sc = MplCanvas(self)
        sc.axes.plot(data.iloc[1:, 8]*180/pi, color='r', label='q')
        sc.axes1 = sc.axes.twinx()
        sc.axes1.plot(data.iloc[1:, 39+2], color='b', label='t_mpc')
        sc.axes1.plot(data.iloc[1:, 146+2], color='g', label='t_pd')
        sc.axes1.plot(data.iloc[1:, 152+2], color='y', label='t_wbc')
        sc.axes1.legend(loc='upper left')
        # add tab to widget
        self.tab3 = AddMplTab(sc)
        self.tabwidget.addTab(self.tab3, "yaw")

        sc = MplCanvas(self)
        sc.axes.plot(data.iloc[1:, 9]*180/pi)
        sc.axes1 = sc.axes.twinx()
        sc.axes1.plot(data.iloc[1:, 39], color='b', label='t')
        # add tab to widget
        self.tab4 = AddMplTab(sc)
        self.tabwidget.addTab(self.tab4, "roll velo")

        sc = MplCanvas(self)
        sc.axes.plot(data.iloc[1:, 10]*180/pi)
        sc.axes1 = sc.axes.twinx()
        sc.axes1.plot(data.iloc[1:, 40], color='b', label='t')
        # add tab to widget
        self.tab5 = AddMplTab(sc)
        self.tabwidget.addTab(self.tab5, "pitch velo")

        sc = MplCanvas(self)
        sc.axes.plot(data.iloc[1:, 11]*180/pi)
        sc.axes1 = sc.axes.twinx()
        sc.axes1.plot(data.iloc[1:, 41], color='b', label='t')
        # add tab to widget
        self.tab6 = AddMplTab(sc)
        self.tabwidget.addTab(self.tab6, "yaw velo")

        sc = MplCanvas(self)
        sc.axes.plot(data.iloc[1:, 0], color='r', label='x')
        sc.axes.plot(data.iloc[1:, 121+3], color='g', label='xd')
        sc.axes1 = sc.axes.twinx()
        sc.axes1.plot(data.iloc[1:, 39+3], color='b', label='t_mpc')
        sc.axes1.plot(data.iloc[1:, 146+3], color='g', label='t_pd')
        sc.axes1.plot(data.iloc[1:, 152+3], color='r', label='t_wbc')
        sc.axes1.legend(loc='upper left')
        # add tab to widget
        self.tab7 = AddMplTab(sc)
        self.tabwidget.addTab(self.tab7, "x")

        sc = MplCanvas(self)
        sc.axes.plot(data.iloc[1:, 1], color='r', label='y')
        sc.axes.plot(data.iloc[1:, 121+4], color='g', label='xd')
        sc.axes1 = sc.axes.twinx()
        sc.axes1.plot(data.iloc[1:, 39+4], color='b', label='t_mpc')
        sc.axes1.plot(data.iloc[1:, 146+4], color='g', label='t_pd')
        sc.axes1.plot(data.iloc[1:, 152+4], color='r', label='t_wbc')
        sc.axes1.legend(loc='upper left')
        # add tab to widget
        self.tab8 = AddMplTab(sc)
        self.tabwidget.addTab(self.tab8, "y")

        sc = MplCanvas(self)
        sc.axes.plot(data.iloc[1:, 2], color='r', label='z')
        sc.axes1 = sc.axes.twinx()
        sc.axes1.plot(data.iloc[1:, 39+5], color='b', label='t_mpc')
        sc.axes1.plot(data.iloc[1:, 146+5], color='g', label='t_pd')
        sc.axes1.plot(data.iloc[1:, 152+5], color='r', label='t_wbc')
        sc.axes1.legend(loc='upper left')
        # add tab to widget
        self.tab9 = AddMplTab(sc)
        self.tabwidget.addTab(self.tab9, "z")

        sc = MplCanvas(self)
        sc.axes.plot(data.iloc[1:, 3], 'r')
        sc.axes.plot(data.iloc[1:, 12], 'g')
        sc.axes1 = sc.axes.twinx()
        sc.axes1.plot(data.iloc[1:, 41], color='b', label='f')
        # add tab to widget
        self.tab10 = AddMplTab(sc)
        self.tabwidget.addTab(self.tab10, "vx")

        sc = MplCanvas(self)
        sc.axes.plot(data.iloc[1:, 4], 'r')
        sc.axes.plot(data.iloc[1:, 121+10], 'g')
        sc.axes1 = sc.axes.twinx()
        sc.axes1.plot(data.iloc[1:, 42], color='b', label='f')
        # add tab to widget
        self.tab11 = AddMplTab(sc)
        self.tabwidget.addTab(self.tab11, "vy")

        sc = MplCanvas(self)
        sc.axes.plot(data.iloc[1:, 5], 'r')
        sc.axes1 = sc.axes.twinx()
        sc.axes1.plot(data.iloc[1:, 43], color='b', label='f')
        # add tab to widget
        self.tab12 = AddMplTab(sc)
        self.tabwidget.addTab(self.tab12, "vz")

        sc = MplCanvas(self)
        sc.axes.plot(data.iloc[1:, 45], 'b-')
        sc.axes.plot(data.iloc[1:, 46], 'r-')
        sc.axes.plot(data.iloc[1:, 47], 'g-')
        # add tab to widget
        self.tab12 = AddMplTab(sc)
        self.tabwidget.addTab(self.tab12, "current")

        sc = MplCanvas(self)
        sc.axes.plot(-data.iloc[1:, 69], '-', label='state')
        sc.axes1 = sc.axes.twinx()
        sc.axes1.plot(data.iloc[1:, 15], 'b-', label='tar')
        sc.axes1.plot(data.iloc[1:, 27], 'r-', label='act')
        sc.axes.legend(loc='upper left')
        # add tab to widget
        self.tab12 = AddMplTab(sc)
        self.tabwidget.addTab(self.tab12, "toeposX1")

        sc = MplCanvas(self)
        sc.axes.plot(-data.iloc[1:, 69], '-', label='state')
        sc.axes1 = sc.axes.twinx()
        sc.axes1.plot(data.iloc[1:, 15+1], 'b-', label='tar')
        sc.axes1.plot(data.iloc[1:, 27+1], 'r-', label='act')
        sc.axes.legend(loc='upper left')
        # add tab to widget
        self.tab12 = AddMplTab(sc)
        self.tabwidget.addTab(self.tab12, "toeposY1")

        sc = MplCanvas(self)
        sc.axes.plot(-data.iloc[1:, 69], '-', label='state')
        sc.axes1 = sc.axes.twinx()
        sc.axes1.plot(data.iloc[1:, 15+2], 'b-', label='tar')
        sc.axes1.plot(data.iloc[1:, 27+2], 'r-', label='act')
        sc.axes.legend(loc='upper left')
        # add tab to widget
        self.tab12 = AddMplTab(sc)
        self.tabwidget.addTab(self.tab12, "toeposZ1")

        sc = MplCanvas(self)
        sc.axes.plot(-data.iloc[1:, 69], '-', label='state')
        sc.axes1 = sc.axes.twinx()
        sc.axes1.plot(data.iloc[1:, 15+3], 'b-', label='tar')
        sc.axes1.plot(data.iloc[1:, 27+3], 'r-', label='act')
        sc.axes.legend(loc='upper left')
        # add tab to widget
        self.tab12 = AddMplTab(sc)
        self.tabwidget.addTab(self.tab12, "toeposX2")

        sc = MplCanvas(self)
        sc.axes.plot(-data.iloc[1:, 69], '-', label='state')
        sc.axes1 = sc.axes.twinx()
        sc.axes1.plot(data.iloc[1:, 15+1+3], 'b-', label='tar')
        sc.axes1.plot(data.iloc[1:, 27+1+3], 'r-', label='act')
        sc.axes.legend(loc='upper left')
        # add tab to widget
        self.tab12 = AddMplTab(sc)
        self.tabwidget.addTab(self.tab12, "toeposY2")

        sc = MplCanvas(self)
        sc.axes.plot(-data.iloc[1:, 69], '-', label='state')
        sc.axes1 = sc.axes.twinx()
        sc.axes1.plot(data.iloc[1:, 15+2+3], 'b-', label='tar')
        sc.axes1.plot(data.iloc[1:, 27+2+3], 'r-', label='act')
        sc.axes.legend(loc='upper left')
        # add tab to widget
        self.tab12 = AddMplTab(sc)
        self.tabwidget.addTab(self.tab12, "toeposZ2")

        sc = MplCanvas(self)
        sc.axes.plot(data.iloc[1:, 70], 'g-', label='Nof')
        sc.axes.plot(data.iloc[1:, 12], 'r-', label='TDf')
        sc.axes.plot(data.iloc[1:, 3], 'b-', label='kalmanf')

        sc.axes.legend(loc='upper left')
        # add tab to widget
        self.tab12 = AddMplTab(sc)
        self.tabwidget.addTab(self.tab12, "com vx")

        sc = MplCanvas(self)
        sc.axes.plot(data.iloc[1:, 70], 'g-', label='Nof')
        sc.axes.plot(data.iloc[1:, 12+2], 'r-', label='TDf')
        sc.axes.plot(data.iloc[1:, 3+2], 'b-', label='kalmanf')

        sc.axes.legend(loc='upper left')
        # add tab to widget
        self.tab12 = AddMplTab(sc)
        self.tabwidget.addTab(self.tab12, "com vz")

        sc = MplCanvas(self)
        sc.axes.plot(data.iloc[1:, 71], 'g-', label='Nof')
        sc.axes.plot(data.iloc[1:, 13], 'r-', label='TDf')
        sc.axes.plot(data.iloc[1:, 4], 'b-', label='kalmanf')

        sc.axes.legend(loc='upper left')
        # add tab to widget
        self.tab12 = AddMplTab(sc)
        self.tabwidget.addTab(self.tab12, "com vy")

        sc = MplCanvas(self)
        sc.axes.plot(data.iloc[1:, 57+2], 'r-*', label='1')
        sc.axes.plot(data.iloc[1:, 57+5], 'b-*', label='2')
        sc.axes.plot(data.iloc[1:, 57+8], 'g-*', label='3')
        sc.axes.plot(data.iloc[1:, 57+11], 'y-*', label='4')
        sc.axes1 = sc.axes.twinx()
        sc.axes1.plot(-data.iloc[1:, 69], '-', label='state')
        sc.axes.legend(loc='upper left')
        # add tab to widget
        self.tab12 = AddMplTab(sc)
        self.tabwidget.addTab(self.tab12, "real force")

        sc = MplCanvas(self)
        sc.axes.plot(data.iloc[1:, 85], 'r-', label='TDf')
        sc.axes.plot(data.iloc[1:, 73], 'g-', label='Nof')
        sc.axes.legend(loc='upper left')
        # add tab to widget
        self.tab12 = AddMplTab(sc)
        self.tabwidget.addTab(self.tab12, "JointVelo1")

        sc = MplCanvas(self)
        sc.axes.plot(data.iloc[1:, 86], 'r-', label='TDf')
        sc.axes.plot(data.iloc[1:, 74], 'g-', label='Nof')
        sc.axes.legend(loc='upper left')
        # add tab to widget
        self.tab12 = AddMplTab(sc)
        self.tabwidget.addTab(self.tab12, "JointVelo2")

        sc = MplCanvas(self)
        sc.axes.plot(data.iloc[1:, 87], 'r-', label='TDf')
        sc.axes.plot(data.iloc[1:, 75], 'g-', label='Nof')
        sc.axes.legend(loc='upper left')
        # add tab to widget
        self.tab12 = AddMplTab(sc)
        self.tabwidget.addTab(self.tab12, "JointVelo3")

        sc = MplCanvas(self)
        sc.axes.plot(data.iloc[1:, 109], 'r-', label='Nof')
        sc.axes.plot(data.iloc[1:, 97], 'g-', label='TDf')
        sc.axes.plot(data.iloc[1:, 134], 'b-', label='tar')
        sc.axes1 = sc.axes.twinx()
        sc.axes1.plot(-data.iloc[1:, 69], '-', label='state')
        sc.axes.legend(loc='upper left')
        # add tab to widget
        self.tab12 = AddMplTab(sc)
        self.tabwidget.addTab(self.tab12, "ToeVelox1")

        sc = MplCanvas(self)
        sc.axes.plot(data.iloc[1:, 109+1], 'r-', label='Nof')
        sc.axes.plot(data.iloc[1:, 97+1], 'g-', label='TDf')
        sc.axes.plot(data.iloc[1:, 135], 'b-', label='tar')
        sc.axes1 = sc.axes.twinx()
        sc.axes1.plot(-data.iloc[1:, 69], '-', label='state')
        sc.axes.legend(loc='upper left')
        # add tab to widget
        self.tab12 = AddMplTab(sc)
        self.tabwidget.addTab(self.tab12, "ToeVeloy1")

        sc = MplCanvas(self)
        sc.axes.plot(data.iloc[1:, 109+2], 'r-', label='Nof')
        sc.axes.plot(data.iloc[1:, 97+2], 'g-', label='TDf')
        sc.axes.plot(data.iloc[1:, 136], 'b-', label='tar')
        sc.axes1 = sc.axes.twinx()
        sc.axes1.plot(-data.iloc[1:, 69], '-', label='state')
        sc.axes.legend(loc='upper left')
        # add tab to widget
        self.tab12 = AddMplTab(sc)
        self.tabwidget.addTab(self.tab12, "ToeVeloz1")

        sc = MplCanvas(self)
        sc.axes.plot(data.iloc[1:, 109+3], 'r-', label='Nof')
        sc.axes.plot(data.iloc[1:, 97+3], 'g-', label='TDf')
        sc.axes.plot(data.iloc[1:, 134+3], 'b-', label='tar')
        sc.axes1 = sc.axes.twinx()
        sc.axes1.plot(-data.iloc[1:, 69], '-', label='state')
        sc.axes.legend(loc='upper left')
        # add tab to widget
        self.tab12 = AddMplTab(sc)
        self.tabwidget.addTab(self.tab12, "ToeVelox2")

        sc = MplCanvas(self)
        sc.axes.plot(data.iloc[1:, 109+1+3], 'r-', label='Nof')
        sc.axes.plot(data.iloc[1:, 97+1+3], 'g-', label='TDf')
        sc.axes.plot(data.iloc[1:, 135+3], 'b-', label='tar')
        sc.axes1 = sc.axes.twinx()
        sc.axes1.plot(-data.iloc[1:, 69], '-', label='state')
        sc.axes.legend(loc='upper left')
        # add tab to widget
        self.tab12 = AddMplTab(sc)
        self.tabwidget.addTab(self.tab12, "ToeVeloy2")

        sc = MplCanvas(self)
        sc.axes.plot(data.iloc[1:, 109+2+3], 'r-', label='Nof')
        sc.axes.plot(data.iloc[1:, 97+2+3], 'g-', label='TDf')
        sc.axes.plot(data.iloc[1:, 136+3], 'b-', label='tar')
        sc.axes1 = sc.axes.twinx()
        sc.axes1.plot(-data.iloc[1:, 69], '-', label='state')
        sc.axes.legend(loc='upper left')
        # add tab to widget
        self.tab12 = AddMplTab(sc)
        self.tabwidget.addTab(self.tab12, "ToeVeloz2")

        sc = MplCanvas(self)
        sc.axes.plot(data.iloc[1:, 158+8], 'r-', label='fz1')
        sc.axes1 = sc.axes.twinx()
        sc.axes1.plot(data.iloc[1:, 69], '-', label='state')
        sc.axes2 = sc.axes.twinx()
        sc.axes2.plot(data.iloc[1:, 27+2], 'b-', label='act')
        sc.axes.legend(loc='upper left')
        # add tab to widget
        self.tab12 = AddMplTab(sc)
        self.tabwidget.addTab(self.tab12, "fz1")
        
        # sc = MplCanvas(self)
        # sc.axes.plot(data.iloc[1:, 170], 'r-', label='x')
        # sc.axes.plot(data.iloc[1:, 171], 'b-', label='y')
        # sc.axes.plot(data.iloc[1:, 172], 'g-', label='z')
        # sc.axes1 = sc.axes.twinx()
        # sc.axes1.plot(data.iloc[1:, 69], '-', label='state')
        # sc.axes2 = sc.axes.twinx()
        # sc.axes2.plot(data.iloc[1:, 27+2], 'b-', label='act')
        # sc.axes.legend(loc='upper left')
        # # add tab to widget
        # self.tab12 = AddMplTab(sc)
        # self.tabwidget.addTab(self.tab12, "fz1")

        '''# sc = MplCanvas(self)
        # sc.axes.plot(data.iloc[1:, 108], color='r', label='q')
        # sc.axes1 = sc.axes.twinx()q
        # sc.axes1.plot(data.iloc[1:, 170], color='b', label='t')
        # sc.axes.legend(loc='upper left')
        # # add tab to widget
        # self.tab1 = AddMplTab(sc)
        # self.tabwidget.addTab(self.tab1, "roll")

        # sc = MplCanvas(self)
        # sc.axes.plot(data.iloc[1:, 109], color='r', label='q')
        # sc.axes1 = sc.axes.twinx()
        # sc.axes1.plot(data.iloc[1:, 171], color='b', label='t')
        # sc.axes.legend(loc='upper left')
        # # add tab to widget
        # self.tab2 = AddMplTab(sc)
        # self.tabwidget.addTab(self.tab2, "pitch")

        # sc = MplCanvas(self)
        # sc.axes.plot(data.iloc[1:, 110], color='r', label='q')
        # sc.axes1 = sc.axes.twinx()
        # sc.axes1.plot(data.iloc[1:, 172], color='b', label='t')
        # sc.axes.legend(loc='upper left')
        # # add tab to widget
        # self.tab3 = AddMplTab(sc)
        # self.tabwidget.addTab(self.tab3, "yaw")

        # sc = MplCanvas(self)
        # sc.axes.plot(data.iloc[1:, 114])
        # sc.axes1 = sc.axes.twinx()
        # sc.axes1.plot(data.iloc[1:, 170], color='b', label='t')
        # # add tab to widget
        # self.tab4 = AddMplTab(sc)
        # self.tabwidget.addTab(self.tab4, "roll velo")

        # sc = MplCanvas(self)
        # sc.axes.plot(data.iloc[1:, 115])
        # sc.axes1 = sc.axes.twinx()
        # sc.axes1.plot(data.iloc[1:, 171], color='b', label='t')
        # # add tab to widget
        # self.tab5 = AddMplTab(sc)
        # self.tabwidget.addTab(self.tab5, "pitch velo")

        # sc = MplCanvas(self)
        # sc.axes.plot(data.iloc[1:, 116])
        # sc.axes1 = sc.axes.twinx()
        # sc.axes1.plot(data.iloc[1:, 172], color='b', label='t')
        # # add tab to widget
        # self.tab6 = AddMplTab(sc)
        # self.tabwidget.addTab(self.tab6, "yaw velo")

        # sc = MplCanvas(self)
        # sc.axes.plot(data.iloc[1:, 140], color='r', label='x')
        # sc.axes1 = sc.axes.twinx()
        # sc.axes1.plot(data.iloc[1:, 173], color='b', label='f')
        # sc.axes.legend(loc='upper left')
        # # add tab to widget
        # self.tab7 = AddMplTab(sc)
        # self.tabwidget.addTab(self.tab7, "x")

        # sc = MplCanvas(self)
        # sc.axes.plot(data.iloc[1:, 141], color='r', label='y')
        # sc.axes1 = sc.axes.twinx()
        # sc.axes1.plot(data.iloc[1:, 174], color='b', label='f')
        # sc.axes.legend(loc='upper left')
        # # add tab to widget
        # self.tab8 = AddMplTab(sc)
        # self.tabwidget.addTab(self.tab8, "y")

        # sc = MplCanvas(self)
        # sc.axes.plot(data.iloc[1:, 142], color='r', label='z')
        # sc.axes1 = sc.axes.twinx()
        # sc.axes1.plot(data.iloc[1:, 175], color='b', label='f')
        # sc.axes.legend(loc='upper left')
        # # add tab to widget
        # self.tab9 = AddMplTab(sc)
        # self.tabwidget.addTab(self.tab9, "z")

        # sc = MplCanvas(self)
        # sc.axes.plot(data.iloc[1:, 143], 'r')
        # sc.axes1 = sc.axes.twinx()
        # sc.axes1.plot(data.iloc[1:, 173], color='b', label='f')
        # # add tab to widget
        # self.tab10 = AddMplTab(sc)
        # self.tabwidget.addTab(self.tab10, "vx")

        # sc = MplCanvas(self)
        # sc.axes.plot(data.iloc[1:, 144], 'r')
        # sc.axes1 = sc.axes.twinx()
        # sc.axes1.plot(data.iloc[1:, 174], color='b', label='f')
        # # add tab to widget
        # self.tab11 = AddMplTab(sc)
        # self.tabwidget.addTab(self.tab11, "vy")

        # sc = MplCanvas(self)
        # sc.axes.plot(data.iloc[1:, 145], 'r')
        # sc.axes1 = sc.axes.twinx()
        # sc.axes1.plot(data.iloc[1:, 175], color='b', label='f')
        # # add tab to widget
        # self.tab12 = AddMplTab(sc)
        # self.tabwidget.addTab(self.tab12, "vz")

        # sc = MplCanvas(self)
        # sc.axes.plot(data.iloc[1:, 36], 'b-')
        # sc.axes.plot(data.iloc[1:, 37], 'r-')
        # sc.axes.plot(data.iloc[1:, 38], 'g-')
        # # add tab to widget
        # self.tab12 = AddMplTab(sc)
        # self.tabwidget.addTab(self.tab12, "current")

        # sc = MplCanvas(self)
        # sc.axes.plot(-data.iloc[1:, 191], '-', label='state')
        # sc.axes1 = sc.axes.twinx()
        # sc.axes1.plot(data.iloc[1:, 179+2], 'b-', label='tar')
        # sc.axes1.plot(data.iloc[1:, 158+2], 'r-', label='act')
        # sc.axes.legend(loc='upper left')
        # # add tab to widget
        # self.tab12 = AddMplTab(sc)
        # self.tabwidget.addTab(self.tab12, "toeposX")

        # sc = MplCanvas(self)
        # sc.axes.plot(-data.iloc[1:, 191], '-', label='state')
        # sc.axes1 = sc.axes.twinx()
        # sc.axes1.plot(data.iloc[1:, 179+1], 'b-', label='tar')
        # sc.axes1.plot(data.iloc[1:, 158+1], 'r-', label='act')
        # sc.axes.legend(loc='upper left')
        # # add tab to widget
        # self.tab12 = AddMplTab(sc)
        # self.tabwidget.addTab(self.tab12, "toeposY")

        # sc = MplCanvas(self)
        # sc.axes.plot(data.iloc[1:, 176], 'r-', label='Nof')
        # sc.axes.plot(data.iloc[1:, 143], 'b-', label='f')
        # sc.axes.legend(loc='upper left')
        # # add tab to widget
        # self.tab12 = AddMplTab(sc)
        # self.tabwidget.addTab(self.tab12, "com vx")

        # sc = MplCanvas(self)
        # sc.axes.plot(data.iloc[1:, 177], 'r-', label='Nof')
        # sc.axes.plot(data.iloc[1:, 144], 'b-', label='f')
        # sc.axes.legend(loc='upper left')
        # # add tab to widget
        # self.tab12 = AddMplTab(sc)
        # self.tabwidget.addTab(self.tab12, "com vy")

        # sc = MplCanvas(self)
        # sc.axes.plot(data.iloc[1:, 192+2], 'r-', label='1')
        # sc.axes.plot(data.iloc[1:, 192+5], 'b-', label='2')
        # sc.axes.plot(data.iloc[1:, 192+8], 'g-', label='3')
        # sc.axes.plot(data.iloc[1:, 192+11], 'y-', label='4')
        # sc.axes1 = sc.axes.twinx()
        # sc.axes1.plot(-data.iloc[1:, 191], '-', label='state')
        # sc.axes.legend(loc='upper left')
        # # add tab to widget
        # self.tab12 = AddMplTab(sc)
        # self.tabwidget.addTab(self.tab12, "real force")
'''


if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)
    main_window = MainWindow()
    main_window.resize(800, 600)
    main_window.show()
    app.exec()
