import numpy as np
from  GridWorld import GridWorld
class MiniGrid(GridWorld):
    def __init__(self, rows, cols, height, start, Kd, Ks, Kt, Kg):
         super().__init__(rows, cols, height, start, Kd, Ks, Kt, Kg)
         self.initGrid=(0,0,0)
         self.endGrid=(rows-1,cols-1,height-1)   
    def setPathGlobal(self, pathGlobal):
        self.pathGlobal = pathGlobal
    
    def setGridSize(self, localSize):
        self.localSize = localSize
    
    def setGoal(self, goal):
        self.goal = goal
    
    def setMiniGrid(self, initGrid, endGrid):
        self.initGrid = initGrid
        self.endGrid = endGrid
    
    def setObstacles(self, obstacles):
        self.obstacles = obstacles

    def setPosition(self, cartesian_position):
        self.i, self.j, self.k = cartesian_position

    def updateObstacles(self, obstacle):
        if obstacle not in (self.obstacles):
            self.obstacles.append(obstacle)
        #self.actionSpace()
    
    def is_onboard(self, cartesian_position):
        '''
        checks if the agent is in the environment and return true or false
        '''
        x, y, z = cartesian_position
        if x < self.initGrid[0]  or x > self.endGrid[0] or y < self.initGrid[1]  or y > self.endGrid[1]:
            return 0
        return 1



