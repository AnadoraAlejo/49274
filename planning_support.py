"""Map-aware support behaviours for missing Planning 1/2 modules.

Use Nav2 for actual collision checking/path planning. This grid check is only
candidate screening; it is not a replacement for Nav2's costmaps.
"""
import math
import cv2
import numpy as np


class GridPlanner:
    def __init__(self):
        self.grid = None
        self.history = []

    def update(self, data, width, height, resolution, origin):
        self.grid = np.asarray(data, dtype=np.int16).reshape(height,width)
        self.res = resolution
        self.ox,self.oy,self.yaw = origin
        self.c,self.s = math.cos(self.yaw),math.sin(self.yaw)
        free = ((self.grid >= 0) & (self.grid < 30)).astype(np.uint8)
        # Pad borders with blocked cells: unknown/outside map is not free.
        padded = np.pad(free,1)
        self.clearance = cv2.distanceTransform(padded,cv2.DIST_L2,5)[1:-1,1:-1]*resolution
        self.safe = (self.clearance >= 0.75).astype(np.uint8) # robot radius .5 + margin
        _,self.components = cv2.connectedComponents(self.safe, connectivity=4)

    def cell(self, xy):
        x,y = xy[0]-self.ox,xy[1]-self.oy
        return int(math.floor((self.c*x+self.s*y)/self.res)), int(math.floor((-self.s*x+self.c*y)/self.res))

    def world(self, col, row):
        x,y=(col+0.5)*self.res,(row+0.5)*self.res
        return self.ox+self.c*x-self.s*y,self.oy+self.s*x+self.c*y

    def inside(self, col, row):
        return 0<=row<self.grid.shape[0] and 0<=col<self.grid.shape[1]

    def safe_goal(self, xy, robot):
        if self.grid is None:
            return False
        c,r=self.cell(xy); rc,rr=self.cell(robot)
        return (self.inside(c,r) and self.inside(rc,rr) and self.safe[r,c] != 0
                and self.components[rr,rc] != 0 and self.components[r,c] == self.components[rr,rc])

    def line_of_sight(self, viewpoint, target):
        # Exclude the final 0.45 m, where the artefact itself may occupy the map.
        dist=math.dist(viewpoint[:2],target[:2])
        for t in np.linspace(0,max(0,1-0.45/max(dist,0.01)),max(2,int(dist/self.res))):
            xy=np.asarray(viewpoint[:2])*(1-t)+np.asarray(target[:2])*t
            c,r=self.cell(xy)
            if not self.inside(c,r) or self.grid[r,c]<0 or self.grid[r,c]>=50:
                return False
        return True

    def approach(self, target, robot, attempt=0, standoff=2.0):
        theta=math.atan2(robot[1]-target[1],robot[0]-target[0])
        offsets = [0,0.35,-0.35,0.7,-0.7,1.1,-1.1] if attempt==0 else [0.7,-0.7,1.1,-1.1,0.35,-0.35,0]
        for offset in offsets:
            angle=theta+offset
            x,y=target[0]+standoff*math.cos(angle),target[1]+standoff*math.sin(angle)
            if self.safe_goal((x,y),robot) and self.line_of_sight((x,y),target):
                return (x,y,math.atan2(target[1]-y,target[0]-x))
        return None

    def explore(self, robot):
        if self.grid is None:
            return None
        c,r=self.cell(robot)
        if not self.inside(c,r) or self.components[r,c]==0:
            return None
        reachable=self.components==self.components[r,c]
        unknown=(self.grid<0).astype(np.uint8)
        # Look for safe cells within ~1.5 m of unknown space.
        size=max(3,int(3/self.res)|1)
        near_unknown=cv2.dilate(unknown,np.ones((size,size),np.uint8))>0
        rows,cols=np.nonzero(reachable & near_unknown)
        if not len(rows):
            rows,cols=np.nonzero(reachable)
        step=max(1,int(0.8/self.res))
        choices=[]
        for row,col in zip(rows[::step],cols[::step]):
            x,y=self.world(col,row)
            distance=math.dist((x,y),robot[:2])
            if distance<1.2:
                continue
            if any(math.dist((x,y),p)<1.5 for p in self.history[-80:]):
                continue
            choices.append((distance,(x,y,math.atan2(y-robot[1],x-robot[0]))))
        return min(choices,key=lambda pair:pair[0])[1] if choices else None

    def remember(self, goal):
        self.history.append(goal[:2])
