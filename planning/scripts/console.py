#! /usr/bin/env python3
'''
IOT AGV Fleet project - Control Panel

Author: RTU
Data: 2022/06/04
Ver: 1.0

Description:
    The control panel with PyQT UI interface. This panel is responsible for display fleet status, fleet contorl, sending commands with GUI.
'''

from os import stat
import rospy
import numpy as np

from nav_msgs.msg import OccupancyGrid, MapMetaData
from fleet_msgs.msg import PlanRequest, Path, Point, Status, FleetStatus

from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *

class WindowStyle():
    def __init__(self):
        self.backgroundcolor = QColor(75,75,75)
        self.border = QColor(0,0,0,0)
        self.node = QColor(100,100,100)
        self.nodeborder = QColor(0,0,0,0)
        self.button = QColor(100,100,100)
        self.text = QColor(222,222,222,255)
        self.beg_color=(200,50,0)
        self.end_color=(0,50,200)
        self.current_color=(255,255,0)

style = WindowStyle()

class GridMap(QPolygon):
    def __init__(self, rect, mapsize, node_radius, qp: QPainter):
        super().__init__()
        self.rect = np.asarray(rect, dtype=int)
        self.mapsize = np.asarray(mapsize, dtype=int)
        self.node_radius = node_radius
        self.grid_size = self.rect[2:]/(self.mapsize+1) #grid size
        self.__rgb_vec = np.asarray(((np.cos(0.0), np.sin(0.0)), (np.cos(2.094395), np.sin(2.094395)),(np.cos(4.18879), np.sin(4.18879))), dtype=float)

        # scales in Q objects
        self.gs_Q = QPoint(self.grid_size[0], self.grid_size[1]) #grid size
        self.rect_Q = QRect(self.rect[0],self.rect[1],self.rect[2],self.rect[3])

        self.qp = qp

        self.__init_map()

    def __init_map(self):
        w,h = self.mapsize
        ox,oy,gw,gh = self.rect

        self.x_mapping = np.linspace(ox, ox+gw, num=w+2, dtype=int)[1:-1]
        self.y_mapping = np.linspace(oy, oy+gh, num=h+2, dtype=int)[1:-1]
        
        for x in self.x_mapping:
            for y in self.y_mapping:
                self << QPoint(int(x), int(y))

    def __rainbow(self, num):
        angle = np.linspace(0.0, 3.14159*2.0*((num-1)/num), num=num)[:,np.newaxis]
        vec = np.concatenate((np.cos(angle), np.sin(angle)), axis=1)
        rgb = np.einsum('ij,kj -> ik', vec,self.__rgb_vec)
        exp = np.exp(rgb)
        rgb = exp/np.sum(exp,axis=1, keepdims=True)*255.0
        return np.clip(rgb.astype(int), 0, 255)

    def drawBackground(self):
        self.qp.setPen(style.border)
        self.qp.setBrush(style.backgroundcolor)
        self.qp.drawRect(self.rect_Q)

    def drawGridMap(self):

        self.drawBackground()
        for i in range(self.count()):
            self.drawNode(point = self.at(i), color=(100,100,100))

    def getFocus(self, e: QMouseEvent):
        mouse = np.asarray((e.pos().x(), e.pos().y()), dtype=float)

        mouse = (mouse - self.rect[:2] - self.grid_size)    #mouse position in subcoordinate
        closest_target = np.round(mouse/self.grid_size).astype(int)
        relative_position = closest_target*self.grid_size - mouse

        if np.any(closest_target < 0) or np.any(closest_target > self.mapsize):
            return None
        if np.linalg.norm(relative_position) < self.node_radius:
            return closest_target
        return None

    def drawNode(self, x=0, y=0, color=(0,0,0), point:QPoint = None, str=None):

        self.qp.setPen(style.nodeborder)
        self.qp.setBrush(QColor(color[0],color[1],color[2]))

        p = QPoint(x,y) if point is None else point
        self.qp.drawEllipse( p, self.node_radius, self.node_radius)
        
        if str is not None:
            self.qp.setPen(style.text)
            self.qp.drawText(QRectF(p - self.gs_Q, p + self.gs_Q), Qt.AlignCenter, str)

    def drawPath(self, path: Path, status: Status = None, beg_color=style.beg_color, end_color=style.end_color):

        shift = self.grid_size + self.rect[:2]

        i,l = -1,len(path.path)
        beg_color = np.array(beg_color)
        end_color = np.array(end_color)

        for p in path.path:
            i+=1
            fuse_color = (beg_color*(1.0-i/l) + end_color*i/l).astype(int)
            pos = np.asarray((p.x,p.y), dtype=int)*self.grid_size + shift
            self.drawNode(x=pos[0], y=pos[1], color=fuse_color, str=f'{p.id}')

        if status is not None:
            pos = np.asarray((status.current.x,status.current.y), dtype=int)*self.grid_size + shift
            self.drawNode(x=pos[0], y=pos[1], color=style.current_color)
            
    def drawMultiplePaths(self, paths, allstatus):

        shift = self.grid_size + self.rect[:2]

        valid_cars = allstatus.keys()
        paths = paths.values()
        rainbow = self.__rainbow(len(paths))
        for color,path in zip(rainbow,paths):
            if path.target in valid_cars:
                status = allstatus[path.target]
                near_plan = path.path[status.current.id: status.current.id+3]
                if len(near_plan)==1:
                    pos = np.asarray((status.current.x,status.current.y), dtype=int)*self.grid_size + shift
                    self.drawNode(x=pos[0], y=pos[1], color=color)
                else:
                    for p in near_plan:
                        pos = np.asarray((p.x,p.y), dtype=int)*self.grid_size + shift
                        self.drawNode(x=pos[0], y=pos[1], color=color, str=f'{p.id}')

class PanelUI(QMainWindow):
    '''
    The panel UI implemented with PyQT5
    '''
    def __init__(self, sendWaypointF, resetWaypointF, gridClickedF, sizeChangedF, changeTargetF, sendPlaneF):
        super().__init__()
        self.resize(800, 600)

        self.qp = QPainter(self)
        self.draw_multi_targets = None
        self.draw_target = None
        self.setStyleSheet("background-color: rgb(50, 50, 50); color: rgb(222,222,222)")

        self.planButtons = QWidget(self)
        self.planButtons.setGeometry(QRect(400, 490, 300, 60))
        self.planButtons.setObjectName("planButtons")
        self.planButtons.setContentsMargins(0, 0, 0, 0)
        self.planButtonLayout = QHBoxLayout(self.planButtons)
        self.planButtonLayout.setObjectName("planButtonLayout")

        self.planeButton = QPushButton(self.planButtons)
        self.planeButton.setObjectName("planeButton")
        self.planeButton.setText('Planning')
        self.planButtonLayout.addWidget(self.planeButton)

        self.resetButton = QPushButton(self.planButtons)
        self.resetButton.setObjectName("resetPathButton")
        self.resetButton.setText('Reset')
        self.planButtonLayout.addWidget(self.resetButton)

        self.sendButton = QPushButton(self.planButtons)
        self.sendButton.setObjectName("sendPlan")
        self.sendButton.setText('Send Plan')
        self.planButtonLayout.addWidget(self.sendButton)

        self.horizontalLayoutWidget = QWidget(self)
        self.horizontalLayoutWidget.setGeometry(QRect(400, 450, 200, 50))
        self.horizontalLayoutWidget.setObjectName("horizontalLayoutWidget")
        self.horizontalLayoutWidget.setContentsMargins(0, 0, 0, 0)
        self.MapSize = QHBoxLayout(self.horizontalLayoutWidget)
        self.MapSize.setObjectName("MapSize")
        self.width = QLineEdit(self.horizontalLayoutWidget)
        self.width.setObjectName("width")
        self.MapSize.addWidget(self.width)
        self.height = QLineEdit(self.horizontalLayoutWidget)
        self.height.setObjectName("height")
        self.MapSize.addWidget(self.height)

        self.setMapButton = QPushButton(self.horizontalLayoutWidget)
        self.setMapButton.setGeometry(QRect(520, 450, 80, 25))
        self.setMapButton.setObjectName("setSize")
        self.setMapButton.setText('Set Map')
        self.MapSize.addWidget(self.setMapButton)

        self.targetlist = QListWidget(self)
        self.targetlist.setGeometry(QRect(70, 450, 200, 100))
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
        self.setMapButton.clicked.connect(self.__changeSize)

        QMetaObject.connectSlotsByName(self)
        self.gridMap = GridMap((50, 30, 700, 400), (10,10), 15, qp=self.qp)

        self.show()

    def mousePressEvent(self, e):

        if self.isDashBoardMode():
            rospy.logdebug('It\'s dashboard mode. No opreation allowed.')
            return

        target=self.gridMap.getFocus(e)
        if target is not None:
            self.gridClickedF(target[0], target[1], self.getTarget())
            self.update()
        
    def paintEvent(self, ev):

        if not self.qp.isActive():
            self.qp.begin(self)

        self.qp.setRenderHint(QPainter.Antialiasing, True)
        self.qp.setRenderHint(QPainter.HighQualityAntialiasing, True)
        self.qp.setRenderHint(QPainter.SmoothPixmapTransform, True)

        self.gridMap.drawGridMap()
        if self.draw_target is not None:
            self.gridMap.drawPath(self.draw_target[0], self.draw_target[1])
        if self.draw_multi_targets is not None:
            self.gridMap.drawMultiplePaths(self.draw_multi_targets[0], self.draw_multi_targets[1])

        if self.qp.isActive():
            self.qp.end()
    '''
    Private functions
    '''
               
    def __retranslateUi(self, Dialog):
        _translate = QCoreApplication.translate
        Dialog.setWindowTitle(_translate("Dialog", "Dialog"))

    # callbacks of weidgets
    def __changeSize(self):
        w,h = self.getMapSize()
        self.gridMap = GridMap((50, 30, 700, 400), (w,h), 15, qp=self.qp)
        self.update()
        self.sizeChangedF(w,h)

    # clicked events of weidgets
    def __sendPlane(self):

        if self.isDashBoardMode():
            self.sendPlaneF()
        else:
            self.sendPlaneF(self.getTarget())

    def __sendWaypoints(self):

        if self.isDashBoardMode():
            rospy.logdebug('It\'s dashboard mode. No opreation allowed.')
            return

        self.sendWaypointF(self.getTarget())

    def __resetWaypoints(self):

        if self.isDashBoardMode():
            rospy.logdebug('It\'s dashboard mode. No opreation allowed.')
            return

        self.resetWaypointF(self.getTarget())
        self.update()

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
            rospy.logfatal(e.args)
            return -1,-1

    # fucntions used from outside
    def showPath(self, path, status=None):
        self.draw_multi_targets = None
        self.draw_target = (path, status)
        self.update()

    # fucntions used from outside
    def showDashBoard(self, paths, allstatus):
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

        self.__init_targets()

    def __init_targets(self):
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
    # The console does not necessarily have the status of a target, and if not, return None
    def __status(self, target):
        if target in self.status.keys():
            return self.status[target]
        return None 

    def __path_cb(self, msg: Path):
        self.path[msg.target] = msg
        self.ui.showPath(msg, self.__status(msg.target))
        rospy.loginfo(f'Receive a path of target {msg.target}.')

    def __fleet_cb(self, msg: FleetStatus):
        for s in msg.fleet:
            self.status[s.target] = s
            self.__isFinished(s)

        if self.ui.isDashBoardMode():
            self.ui.showDashBoard(self.path, self.status)
        else:
            target = self.ui.getTarget()
            self.ui.showPath(self.path[target], self.__status(target))

    def __isFinished(self, status):
        if status is None:
            return False
        
        target = status.target
        if len(self.path[target].path)<=1:
            return False

        if status.next == self.path[target].path[-1]:
            rospy.loginfo(f'Target {target} just finish the task!')
            self.resetWaypoint(target)

    def __isNeighbor(self, a: Point, b : Point):
        return abs(a.x-b.x)+abs(a.y-b.y) <= 1 

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
                    map[p.y, p.x] = p.id
        costmap.data = map.reshape(-1)
        return costmap

    '''
    injection callback for ui trigger
    '''
    def sizeChanged(self, w, h):
        rospy.loginfo(f'Set the map size to {w} by {h}')
        self.__init_targets()

    def sendOnePlane(self, target):
        path = self.path[target]

        if not self.__isContinuous(path):
            rospy.logdebug(f'Failed to assign a task with length:{len(path.path)} to target {target}.')
            return False

        if len(path.path)<2:
            rospy.logdebug(f'Failed! The task does not have enough points.')
            return False

        status = self.__status(target)
        if status is not None:
            if not (path.path[0].x == status.current.x and path.path[0].y == status.current.y):
                rospy.logerr('Warning: The starting point of the path is not where the target is!')
                return False
        
        self.plan_pub.publish(path)
        rospy.loginfo(f'Assign a task with length:{len(path.path)} to target {target}.')

    def sendPlane(self, target=None):
        if target is None:
            for target in self.path.keys():
                self.sendOnePlane(target)
        else:
            self.sendOnePlane(target)

    def appendStatusInfo(self, req: PlanRequest):
        target = req.target
        status = self.__status(target)
        if status is not None:
            if not (status.current.x == req.waypoints.path[0].x and status.current.y == req.waypoints.path[0].y):
                req.waypoints.path.insert(0, status.current)
                rospy.loginfo(f'Automatically fill current position as the first waypoint.')
            req.heading = status.heading

    def sendWaypoint(self, target):
        planRequest = PlanRequest()

        planRequest.target = target
        planRequest.costmap = self.__generateCostMap(target)
        planRequest.waypoints = self.path[target]
        self.appendStatusInfo(planRequest)

        rospy.loginfo(f'Send a task for {target} with {len(planRequest.waypoints.path)} waypoints.')
        self.waypoint_pub.publish(planRequest)
        self.resetWaypoint(target)

    def resetWaypoint(self, target):
        if target in self.path.keys():
            self.path[target].path = []

    def gridClicked(self, x, y, target):
        path_length = len(self.path[target].path)
        pos = Point(x=x, y=y, id=path_length)
        self.path[target].path.append(pos)
        self.ui.showPath(self.path[target], self.__status(target))
        rospy.loginfo(f'Set {path_length}th waypoint as ({x},{y})')

    def changeTarget(self, target):
        if target == 'dashboard':
            self.ui.showDashBoard(self.path, self.status)
            rospy.loginfo(f'Switch to all target.')
        else:
            self.ui.showPath(self.path[target], self.__status(target))
            rospy.loginfo(f'Switch to target {target}.')

if __name__ == "__main__":
    import sys
    rospy.init_node('Console', anonymous=True, log_level=rospy.INFO)
    app = QApplication(sys.argv)
    Console = Console()
    sys.exit(app.exec_())
