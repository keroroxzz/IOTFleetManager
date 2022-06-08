#! /usr/bin/env python3
'''
IOT AGV Fleet project - Control Panel

Author: RTU
Data: 2022/06/04
Ver: 1.0

Description:
    The control panel with PyQT UI interface. This panel is responsible for display fleet status, fleet contorl, sending commands with GUI.
'''

import rospy
import numpy as np

from nav_msgs.msg import OccupancyGrid, MapMetaData
from fleet_msgs.msg import PlanRequest, Path, Point, Status, FleetStatus

from functools import partial
from PyQt5 import QtCore, QtGui, QtWidgets

class PanelUI(QtWidgets.QMainWindow):
    '''
    The panel UI implemented with PyQT5
    '''
    def __init__(self, sendWaypointF, resetWaypointF, gridClickedF, sizeChangedF, changeTargetF, sendPlaneF):
        super().__init__()
        self.Dialog = self
        self.resize(800, 600)

        self.gridMap = QtWidgets.QFrame(self.Dialog)
        self.gridMap.setObjectName("Grid Frame")
        self.gridMap.setGeometry(QtCore.QRect(50, 50, 700, 400))
        self.gridButtonList = []

        self.planeButton = QtWidgets.QPushButton(self.Dialog)
        self.planeButton.setGeometry(QtCore.QRect(520, 490, 80, 25))
        self.planeButton.setObjectName("planeButton")
        self.planeButton.setText('Planning')

        self.resetButton = QtWidgets.QPushButton(self.Dialog)
        self.resetButton.setGeometry(QtCore.QRect(620, 490, 80, 25))
        self.resetButton.setObjectName("resetPathButton")
        self.resetButton.setText('Reset')

        self.sendButton = QtWidgets.QPushButton(self.Dialog)
        self.sendButton.setGeometry(QtCore.QRect(520, 525, 80, 25))
        self.sendButton.setObjectName("sendPlan")
        self.sendButton.setText('Send Plan')

        self.horizontalLayoutWidget = QtWidgets.QWidget(self.Dialog)
        self.horizontalLayoutWidget.setGeometry(QtCore.QRect(70, 450, 200, 35))
        self.horizontalLayoutWidget.setObjectName("horizontalLayoutWidget")
        self.MapSize = QtWidgets.QHBoxLayout(self.horizontalLayoutWidget)
        self.MapSize.setContentsMargins(0, 0, 0, 0)
        self.MapSize.setObjectName("MapSize")
        self.width = QtWidgets.QLineEdit(self.horizontalLayoutWidget)
        self.width.setObjectName("width")
        self.MapSize.addWidget(self.width)
        self.height = QtWidgets.QLineEdit(self.horizontalLayoutWidget)
        self.height.setObjectName("height")
        self.MapSize.addWidget(self.height)

        self.listWidget = QtWidgets.QListWidget(self.Dialog)
        self.listWidget.setGeometry(QtCore.QRect(70, 500, 200, 150))
        self.listWidget.setObjectName("listWidget")
        self.listWidget.addItem("dashboard")
        self.listWidget.addItem("car_1")
        self.listWidget.addItem("car_2")
        self.listWidget.setCurrentRow(0)

        self.__retranslateUi(self.Dialog)

        self.width.setText('10')
        self.height.setText('10')

        # the functions to be bridged
        self.sendWaypointF = sendWaypointF
        self.resetWaypointF = resetWaypointF
        self.gridClickedF = gridClickedF
        self.sizeChangedF = sizeChangedF
        self.changeTargetF = changeTargetF
        self.sendPlaneF = sendPlaneF

        # function connection
        self.planeButton.clicked.connect(self.__sendWaypoints)
        self.resetButton.clicked.connect(self.__resetWaypoints)
        self.sendButton.clicked.connect(self.__sendPlane)
        self.listWidget.itemClicked.connect(self.__changeTarget)
        self.width.textChanged.connect(self.__changeSize)
        self.height.textChanged.connect(self.__changeSize)

        QtCore.QMetaObject.connectSlotsByName(self.Dialog)

        self.__changeSize()
        self.Dialog.show()

        self.__rgb_vec = np.asarray(((np.cos(0.0), np.sin(0.0)), (np.cos(2.094395), np.sin(2.094395)),(np.cos(4.18879), np.sin(4.18879))), dtype=float)

        self.points = QtGui.QPolygon()

    def mouseMoveEvent(self, e: QtGui.QMouseEvent):
        self.points << e.pos()
        self.update()

    def mousePressEvent(self, e):
        self.points << e.pos()
        self.update()
        
    def paintEvent(self, ev):
        qp = QtGui.QPainter(self)
        qp.setRenderHint(QtGui.QPainter.Antialiasing)
        pen = QtGui.QPen(QtCore.Qt.red, 5)
        brush = QtGui.QBrush(QtCore.Qt.red, QtCore.Qt.BrushStyle.Dense2Pattern)
        #qp.setPen(pen)
        qp.setBrush(brush)
        for i in range(self.points.count()):
            qp.end
            qp.drawPoints(self.points)

    '''
    Private functions
    '''
    def __rgb2hex(self, r, g, b):
        return '#%02x%02x%02x'%(int(r), int(g), int(b))

    def __rainbow(self, num):
        angle = np.linspace(0.0, 3.14159*2.0*((num-1)/num), num=num)[:,np.newaxis]
        vec = np.concatenate((np.cos(angle), np.sin(angle)), axis=1)
        rgb = np.einsum('ij,kj -> ik', vec,self.__rgb_vec)
        exp = np.exp(rgb)
        rgb = exp/np.sum(exp,axis=1, keepdims=True)*255.0
        return np.clip(rgb.astype(int), 0, 255)

    def __setButtonColor(self, rgb=(0,0,0), x=-1, y=-1, button : QtWidgets.QPushButton = None) -> QtWidgets.QPushButton:
        if button is None:
            button = self.gridButtonList[x][y]
        #button.setStyleSheet(f"background-color: {self.__rgb2hex(rgb[0], rgb[1], rgb[2]) if hex is None else hex}")

        button.setAutoFillBackground(True)
        palette = button.palette()
        palette.setColor(button.backgroundRole(), QtGui.QColor(rgb[0], rgb[1], rgb[2]))
        button.setPalette(palette)

        return button

    def __resetGridMap(self):
        for button in self.__iterButton():
            self.__setButtonColor(rgb=(255,255,255),button=button).setText('')
               
    def __iterButton(self):
        for row in self.gridButtonList:
            for b in row:
                yield b

    def __retranslateUi(self, Dialog):
        _translate = QtCore.QCoreApplication.translate
        Dialog.setWindowTitle(_translate("Dialog", "Dialog"))

    # callbacks of weidgets
    def __changeSize(self):
        w,h = self.getMapSize()
        gw,gh = self.gridMap.geometry().width(),self.gridMap.geometry().height()
        sx, sy = gw/w, gh/h

        buttonSize = min(30,sx,sy)
        offset = (sx/2-buttonSize/2, sy/2-buttonSize/2)

        if w<=0 or h<=0:
            return

        for b in self.__iterButton():
            b.deleteLater()
            b.setVisible(False)
        
        col=[]
        for x in range(w):
            row = []
            for y in range(h):
                button = QtWidgets.QPushButton(self.gridMap)
                button.setGeometry(QtCore.QRect(x*sx+offset[0],y*sy+offset[1], buttonSize, buttonSize))
                button.setObjectName(f"{x}_{y}")
                self.__setButtonColor(rgb=(255,255,255),button=button)
                button.clicked.connect(partial(self.__gridClicked, button, x, y))
                row.append(button)
            col.append(row)
        self.gridButtonList=col
        
        self.sizeChangedF(w,h)
        self.update()

    # clicked events of weidgets
    def __sendPlane(self):

        if self.isDashBoardMode():
            rospy.loginfo('It\'s dashboard mode. No opreation allowed.')
            return

        self.sendPlaneF(self.getTarget())

    def __sendWaypoints(self):

        if self.isDashBoardMode():
            rospy.loginfo('It\'s dashboard mode. No opreation allowed.')
            return

        self.sendWaypointF(self.getTarget())

    def __resetWaypoints(self):

        if self.isDashBoardMode():
            rospy.loginfo('It\'s dashboard mode. No opreation allowed.')
            return

        self.resetWaypointF(self.getTarget())
        self.__resetGridMap()

    def __gridClicked(self, button, x, y):

        if self.isDashBoardMode():
            rospy.loginfo('It\'s dashboard mode. No opreation allowed.')
            return

        self.gridClickedF(x, y, self.getTarget())
        #button.setStyleSheet(f"background-color: {self.__rgb2hex(255,0,0)}")

    def __changeTarget(self, item):
        rospy.loginfo(f'Select target {item.text()}.')
        self.changeTargetF(item.text())
 
    '''
    Public functions
    '''
    def getMapSize(self):
        
        try:
            w,h = int(self.width.text()), int(self.height.text())
            if w>100 or h>100 or w<1 or h<1:
                raise("Wrong width or height!")
            return w,h
        except Exception as e:
            print(e.args)
            return -1,-1

    # fucntions used from outside
    def showPath(self, path, beg_color=(255,0,0), end_color=(0,0,255)):

        self.__resetGridMap()
        i,l = -1,len(path.path)
        beg_color = np.array(beg_color)
        end_color = np.array(end_color)

        for p in path.path:
            i+=1
            fuse_color = beg_color*(1.0-i/l) + end_color*i/l
            x, y = int(p.x), int(p.y)
            self.__setButtonColor(rgb=fuse_color, x=x, y=y).setText(f'{p.id}')

        self.update()

    # fucntions used from outside
    def showDashBoard(self, paths, allstatus):

        self.__resetGridMap()

        paths = paths.values()
        rainbow = self.__rainbow(len(paths))
        for color,path in zip(rainbow,paths):
            status = allstatus[path.target]
            for p in path.path[status.current.id: status.current.id+3]:
                x, y = int(p.x), int(p.y)
                self.__setButtonColor(rgb=color, x=x, y=y).setText(f'{p.id}')

        self.update()

    def getTarget(self):
        return self.listWidget.currentItem().text()

    def isDashBoardMode(self):
        return self.getTarget()=='dashboard'


class Console():
    '''
    Control panel manager class to bridge the ros message and the panel UI
    '''
    def __init__(self):
        rospy.Subscriber('console/path', Path, callback=self.__path_cb, queue_size=10)
        rospy.Subscriber('/console/fleet_status', FleetStatus, callback=self.__fleet_cb, queue_size=10)
        self.waypoint_pub = rospy.Publisher('planner/plan', PlanRequest, queue_size=1)
        self.plan_pub = rospy.Publisher('manager/plan', Path, queue_size=1)

        self.ui = PanelUI(self.sendWaypoint, self.resetWaypoint, self.gridClicked, self.sizeChanged, self.changeTarget, self.sendPlane)

        self.status = {}
        self.path = {}
        for item in [self.ui.listWidget.item(i) for i in range(self.ui.listWidget.count())]:
            if item.text() == 'dashboard':
                continue
            self.path[item.text()] = Path()
            self.path[item.text()].target = item.text()
            rospy.loginfo(f'Found target {item.text()}, add to list...')

    '''
    Private functions
    '''
    def __path_cb(self, msg: Path):
        self.path[msg.target] = msg
        self.ui.showPath(msg)
        rospy.loginfo(f'Receive a path of target {msg.target}.')

    def __fleet_cb(self, msg: FleetStatus):
        for s in msg.fleet:
            self.status[s.target] = s

        if self.ui.isDashBoardMode():
            self.ui.showDashBoard(self.path, self.status)

    def __isNeighbor(self, a: Point, b : Point):
        return abs(a.x-b.x)+abs(a.y-b.y) == 1 

    def __isContinuous(self, path: Path):
        for i in range(1, len(path.path)):
            if not self.__isNeighbor(path.path[i-1], path.path[i]):
                return False
        return True

    def __generateCostMap(self, target):
        w,h = self.ui.getMapSize()
        costmap = OccupancyGrid()
        costmap.info = MapMetaData(width=w, height=h)
        map = np.zeros((h, w),dtype=np.uint8)
        for k,v in self.path.items():
            if k != target:
                for p in v.path:
                    map[p.y, p.x] += 10
        costmap.data = map.reshape(-1)
        return costmap

    '''
    injection callback for ui trigger
    '''
    def sizeChanged(self, w, h):
        print(f'Set the map size to {w} by {h}')


    def sendPlane(self, target):
        path: Path = self.path[target]
        if not self.__isContinuous(path):
            return False
        
        self.plan_pub.publish(path)
        rospy.loginfo(f'Assign a task with length:{len(path.path)} to target {target}.')

    def sendWaypoint(self, target):
        planRequest = PlanRequest()

        planRequest.target = target
        planRequest.costmap = self.__generateCostMap(target)
        planRequest.waypoints = self.path[target]

        print(f'Send a task for {target} with {len(planRequest.waypoints.path)} waypoints.')
        self.waypoint_pub.publish(planRequest)
        self.resetWaypoint(target)

    def resetWaypoint(self, target):
        if target in self.path.keys():
            self.path[target].path = []

    def gridClicked(self, x, y, target):
        path_length = len(self.path[target].path)
        pos = Point(x=x, y=y, id=path_length)
        self.path[target].path.append(pos)
        self.ui.showPath(self.path[target])
        print(f'Set {path_length}th waypoint as ({x},{y})')

    def changeTarget(self, target):
        if target == 'dashboard':
            self.ui.showDashBoard(self.path, self.status)
            rospy.loginfo(f'Switch to all target.')
        else:
            self.ui.showPath(self.path[target])
            rospy.loginfo(f'Switch to target {target}.')

if __name__ == "__main__":
    import sys
    rospy.init_node('Console', anonymous=True, log_level=rospy.INFO)
    app = QtWidgets.QApplication(sys.argv)
    Console = Console()
    sys.exit(app.exec_())
