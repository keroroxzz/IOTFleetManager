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

class GridMap(QtGui.QPolygon):
    def __init__(self, rect, mapsize, node_radius, qp):
        super().__init__()
        self.rect = np.asarray(rect, dtype=int)
        self.mapsize = np.asarray(mapsize, dtype=int)
        self.node_radius = node_radius
        self.grid_size = self.rect[2:]/(self.mapsize+1) #grid size
        self.qp = qp

        self.__rgb_vec = np.asarray(((np.cos(0.0), np.sin(0.0)), (np.cos(2.094395), np.sin(2.094395)),(np.cos(4.18879), np.sin(4.18879))), dtype=float)

        w,h = self.mapsize
        ox,oy,gw,gh = self.rect

        self.x_mapping = np.linspace(ox, ox+gw, num=w+2, dtype=int)[1:-1]
        self.y_mapping = np.linspace(oy, oy+gh, num=h+2, dtype=int)[1:-1]
        
        for x in self.x_mapping:
            for y in self.y_mapping:
                self << QtCore.QPoint(int(x), int(y))

    def __rainbow(self, num):
        angle = np.linspace(0.0, 3.14159*2.0*((num-1)/num), num=num)[:,np.newaxis]
        vec = np.concatenate((np.cos(angle), np.sin(angle)), axis=1)
        rgb = np.einsum('ij,kj -> ik', vec,self.__rgb_vec)
        exp = np.exp(rgb)
        rgb = exp/np.sum(exp,axis=1, keepdims=True)*255.0
        return np.clip(rgb.astype(int), 0, 255)

    def draw(self):

        self.qp.setBrush(QtGui.QColor(0,0,0))
        for i in range(self.count()):
            self.qp.drawEllipse(self.at(i), self.node_radius, self.node_radius)

    def getFocus(self, e: QtGui.QMouseEvent):
        mouse = np.asarray((e.pos().x(), e.pos().y()), dtype=float)

        mouse = (mouse - self.rect[:2] - self.grid_size)    #mouse position in subcoordinate
        closest_target = np.round(mouse/self.grid_size).astype(int)
        relative_position = closest_target*self.grid_size - mouse
        if np.linalg.norm(relative_position) < self.node_radius:
            return closest_target
        return None

    def drawNode(self, x, y, color):

        self.qp.setBrush(QtGui.QColor(color[0],color[1],color[2]))
        self.qp.drawEllipse(QtCore.QPoint(x,y), self.node_radius, self.node_radius)

    def drawPath(self, path: Path, beg_color=(255,0,0), end_color=(0,0,255)):

        shift = self.grid_size + self.rect[:2]

        i,l = -1,len(path.path)
        beg_color = np.array(beg_color)
        end_color = np.array(end_color)

        for p in path.path:
            i+=1
            fuse_color = (beg_color*(1.0-i/l) + end_color*i/l).astype(int)

            pos = np.asarray((p.x,p.y), dtype=int)*self.grid_size + shift

            self.drawNode(pos[0], pos[1], fuse_color)
            
    def drawMultiplePaths(self, paths, allstatus):

        shift = self.grid_size + self.rect[:2]

        paths = paths.values()
        rainbow = self.__rainbow(len(paths))
        for color,path in zip(rainbow,paths):
            status = allstatus[path.target]
            for p in path.path[status.current.id: status.current.id+3]:

                pos = np.asarray((p.x,p.y), dtype=int)*self.grid_size + shift

                self.drawNode(pos[0], pos[1], color)

class PanelUI(QtWidgets.QMainWindow):
    '''
    The panel UI implemented with PyQT5
    '''
    def __init__(self, sendWaypointF, resetWaypointF, gridClickedF, sizeChangedF, changeTargetF, sendPlaneF):
        super().__init__()
        self.resize(800, 600)

        # self.gridMap = QtWidgets.QFrame(self)
        # self.gridMap.setObjectName("Grid Frame")
        # self.gridMap.setGeometry(QtCore.QRect(50, 50, 700, 400))
        # self.gridButtonList = []
        self.qp = QtGui.QPainter(self)
        self.draw_multi_targets = None
        self.draw_target = None

        self.planeButton = QtWidgets.QPushButton(self)
        self.planeButton.setGeometry(QtCore.QRect(520, 490, 80, 25))
        self.planeButton.setObjectName("planeButton")
        self.planeButton.setText('Planning')

        self.resetButton = QtWidgets.QPushButton(self)
        self.resetButton.setGeometry(QtCore.QRect(620, 490, 80, 25))
        self.resetButton.setObjectName("resetPathButton")
        self.resetButton.setText('Reset')

        self.sendButton = QtWidgets.QPushButton(self)
        self.sendButton.setGeometry(QtCore.QRect(520, 525, 80, 25))
        self.sendButton.setObjectName("sendPlan")
        self.sendButton.setText('Send Plan')

        self.sendButton = QtWidgets.QPushButton(self)
        self.sendButton.setGeometry(QtCore.QRect(400, 525, 80, 25))
        self.sendButton.setObjectName("setSize")
        self.sendButton.setText('Set Map')

        self.horizontalLayoutWidget = QtWidgets.QWidget(self)
        self.horizontalLayoutWidget.setGeometry(QtCore.QRect(400, 450, 200, 35))
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

        self.targetlist = QtWidgets.QListWidget(self)
        self.targetlist.setGeometry(QtCore.QRect(70, 450, 200, 100))
        self.targetlist.setObjectName("targetlist")
        self.targetlist.addItem("dashboard")
        self.targetlist.addItem("car_1")
        self.targetlist.addItem("car_2")
        self.targetlist.setCurrentRow(0)

        self.__retranslateUi(self)

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
        self.targetlist.itemClicked.connect(self.__changeTarget)
        self.width.textChanged.connect(self.__changeSize)
        self.height.textChanged.connect(self.__changeSize)

        QtCore.QMetaObject.connectSlotsByName(self)

        self.__changeSize()
        self.show()

    
    def mouseMoveEvent(self, e: QtGui.QMouseEvent):
        target=self.gridMap.getFocus(e)
        if target is not None:
            print(target)
            self.update()

    def mousePressEvent(self, e):
        target=self.gridMap.getFocus(e)
        if target is not None:
            self.gridClickedF(target[0], target[1], self.getTarget())
            self.update()
        
    def paintEvent(self, ev):
        if not self.qp.isActive():
            self.qp.begin(self)
        self.gridMap.draw()
        if self.draw_target is not None:
            self.gridMap.drawPath(self.draw_target)
        if self.draw_multi_targets is not None:
            self.gridMap.drawMultiplePaths(self.draw_multi_targets[0], self.draw_multi_targets[1])
        self.qp.end()
    '''
    Private functions
    '''

    def __resetGridMap(self):
        for button in self.__iterButton():
            self.__setButtonColor(rgb=(255,255,255),button=button).setText('')
               
    def __retranslateUi(self, Dialog):
        _translate = QtCore.QCoreApplication.translate
        Dialog.setWindowTitle(_translate("Dialog", "Dialog"))

    # callbacks of weidgets
    def __changeSize(self):

        self.gridMap = GridMap((50, 50, 700, 400), (self.getMapSize()), 15, qp=self.qp)
        # w,h = self.getMapSize()
        # gw,gh = self.gridMap.geometry().width(),self.gridMap.geometry().height()
        # sx, sy = gw/w, gh/h

        # buttonSize = min(30,sx,sy)
        # offset = (sx/2-buttonSize/2, sy/2-buttonSize/2)

        # if w<=0 or h<=0:
        #     return

        # for b in self.__iterButton():
        #     b.deleteLater()
        #     b.setVisible(False)
        
        # col=[]
        # for x in range(w):
        #     row = []
        #     for y in range(h):
        #         button = QtWidgets.QPushButton(self.gridMap)
        #         button.setGeometry(QtCore.QRect(x*sx+offset[0],y*sy+offset[1], buttonSize, buttonSize))
        #         button.setObjectName(f"{x}_{y}")
        #         self.__setButtonColor(rgb=(255,255,255),button=button)
        #         button.clicked.connect(partial(self.__gridClicked, button, x, y))
        #         row.append(button)
        #     col.append(row)
        # self.gridButtonList=col
        
        # self.sizeChangedF(w,h)
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
    def showPath(self, path):
        self.draw_multi_targets = None
        self.draw_target = None
        self.draw_target = path
        self.update()

    # fucntions used from outside
    def showDashBoard(self, paths, allstatus):
        self.draw_multi_targets = None
        self.draw_target = None
        self.draw_multi_targets = (paths, allstatus)
        self.update()

    def getTarget(self):
        return self.targetlist.currentItem().text()

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
        for item in [self.ui.targetlist.item(i) for i in range(self.ui.targetlist.count())]:
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
