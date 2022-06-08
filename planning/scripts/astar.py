#! /usr/bin/env python3
'''
IOT AGV Fleet project - AStar planning Node

Author: RTU
Data: 2022/06/04
Ver: 1.0

Description:
    The A* algorithm in Python responsible for receiving costmap and waypoints.
'''

import cv2
import rospy
import numpy as np

from fleet_msgs.msg import PlanRequest, Path, Point

class AStar():
    def __init__(self, map):
        self.map = map
        self.shape = map.shape

        # hyperparameters
        self.turn_cost = 50
        self.neighbors = np.asarray(
            ((1,0),
            (0,1),
            (-1,0),
            (0,-1))
            , dtype=int)

    '''
    Functional 
    '''
    def __getResizedShape(self, size, ratio):
        return (np.asarray(size[::-1], dtype=float)*ratio).astype(int)

    def __inMap(self, p):   # input shape ((x),(y))
        return not(p[0]<0 or p[1]<0 or p[0]>=self.shape[1] or p[1]>=self.shape[0])

    def __pose2np(self, p: Point):
        return np.asarray((p.x, p.y), dtype=int)

    def __getPath(self, parent, start, end):
  
        path = Path()
        p = end
        while p is not None:
            pos = Point(x=p[0], y=p[1])
            path.path.insert(0,pos)
            p = parent[p[1], p[0]] if not np.all(p==start) else None

        return path

    def __headingCost(self, a, b):
        if a is None or b is None:
            return 0

        cosine_similarity = np.dot(a,b)/np.linalg.norm(a)/np.linalg.norm(b)
        return self.turn_cost*(1-0.5*cosine_similarity)

    '''
    Visualization
    '''
    def __visualizePath(self, name, path, size, ratio, wait=1, waypoints=None):

        map = np.zeros(size, dtype=np.uint8)
        for p in path.path:
            map[p.y,p.x]=255
        if waypoints is not None:
            for p in waypoints.path:
                map[p.y,p.x]=125
                
        resize = self.__getResizedShape(size, ratio)
        rmap = cv2.resize(map, resize, interpolation=cv2.INTER_NEAREST)
        cv2.imshow(name, rmap)
        cv2.waitKey(wait)

    def __visualizeCost(self, name, map, ratio, wait=10, start=None, end=None, explor=None):

        map = map.astype(np.uint8)
        if start is not None:
            map[start[1], start[0]] = (0,0,255)
        if end is not None:
            map[end[1], end[0]] = (0,255,255)
        if explor is not None:
            for p in explor:
                map[p[1],p[0]] = (125,125,125)

        resize = self.__getResizedShape(map.shape[:2], ratio)
        rmap = cv2.resize(map, resize, interpolation=cv2.INTER_NEAREST)
        cv2.imshow(name, rmap)
        cv2.waitKey(wait)

    '''
    A* planning algorithm
    '''
    def __planning(self, start, end, heading=None, visualization = False):
        
        cf = np.zeros((self.shape[0], self.shape[1], 3), dtype=int) # (h, g, c)
        o = np.zeros(self.shape, dtype=int)
        parent = np.zeros((self.shape[0], self.shape[1], 2), dtype=int)

        # initialization
        explor = []
        o[start[1], start[0]] = 2
        for n in self.neighbors:
            p = start + n
            if self.__inMap(p):
                explor.append(p)

                cf[p[1],p[0]] = (
                    np.sum(np.abs(end-p)),
                    self.map[p[1],p[0]]+1,
                    self.__headingCost(n,heading))

                parent[p[1],p[0]] = start


        #planning
        while len(explor)>0:

            explor.sort(key = lambda x:np.sum(cf[x[1],x[0]]), reverse=True)
            n = explor.pop()
            o[n[1],n[0]] = 2

            if np.all(n==end):
                return self.__getPath(parent, start, end)

            for neighbor in self.neighbors:
                
                p = n + neighbor
                if self.__inMap(p) and o[p[1],p[0]]<2:

                    if o[p[1],p[0]] == 0:
                        o[p[1],p[0]] = 1
                        explor.append(p)

                    pheading = n - parent[n[1],n[0]]
                    new_config = np.asarray((
                        np.sum(np.abs(end-p)),
                        cf[n[1],n[0],1]+self.map[p[1],p[0]]+1,
                        self.__headingCost(neighbor,pheading)),
                        dtype=int)

                    if cf[p[1],p[0],1] == 0 or np.sum(new_config)<np.sum(cf[p[1],p[0]]):
                        cf[p[1],p[0]] = new_config
                        parent[p[1],p[0]] = n

                if visualization:
                    self.__visualizeCost('cost', cf, 5, start=start, end=end, explor=explor)

        return None

    '''
    Public calls
    '''
    def PlanePath(self, start, end, heading, visualization=False):

        if np.all(start == end):
            return Path(path=[Point(x=start[0], y=start[1]), Point(x=end[0], y=end[1])]), heading

        path = self.__planning(start, end, heading, visualization=visualization)
        if path is None:
            return None,None

        heading = self.__pose2np(path.path[-1]) - self.__pose2np(path.path[-2])
        return path, heading

    def PathFromWaypoints(self, waypoints: Path, heading=None, visualize_result=False, visualize_process=False):
        path = Path()
        path.target = waypoints.target

        for i in range(1, len(waypoints.path)):

            if len(path.path)>0:
                path.path.pop()    # pop one last to prevent repeat node

            start = self.__pose2np(waypoints.path[i-1])
            end = self.__pose2np(waypoints.path[i])
            subpath, heading = self.PlanePath(start, end, heading, visualization=visualize_process)
            path.path.extend(subpath.path)

        id=0
        for p in path.path:
            p.id = id
            id+=1

        if visualize_result:
            self.__visualizePath('path', path, self.shape[:2], 5, waypoints=waypoints)

        return path
        

class PlannerNode():
  def __init__(self, node_name):
    rospy.init_node(node_name, anonymous=True)
    
    self.planner = None
    rospy.Subscriber('planner/plan', PlanRequest, callback=self.plan_cb, queue_size=1)
    self.path_pub = rospy.Publisher('console/path', Path, queue_size=10)

  def plan_cb(self, msg: PlanRequest):
        w,h = msg.costmap.info.width, msg.costmap.info.height
        rospy.loginfo(f'Planner: receive a map with size:{(w, h)}')
        map = np.asarray(msg.costmap.data).reshape(h,w)
        self.planner = AStar(map)

        rospy.loginfo(f'Planner: receive a task with {len(msg.waypoints.path)} waypoints')
        path = self.planner.PathFromWaypoints(msg.waypoints)
        self.path_pub.publish(path)
            
if __name__ == '__main__':
    planner = PlannerNode('planner')
    rospy.spin()



