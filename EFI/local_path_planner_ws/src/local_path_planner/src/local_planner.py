#! /usr/bin/env python3
from openinstance import OpenInstance
from gridworld import GridWorld
from qlAgentCursed import qlAgent as qla

from sys import platform
opr=platform

import time
import rospy
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Point
import numpy as np



def opt2position(x_max :float, y_max:float, size:float, position:tuple)->tuple:
    n_cols = int(x_max / size)
    n_rows = int(y_max / size)
    
    current_10x = position[0] * 10
    current_10y = position[1] * 10
    size_10 = size * 10

    x, y = int(current_10x/size_10), int(current_10y/size_10)
    cell = (n_cols) * int(current_10y/size_10) + int(current_10x/size_10)

    return x, y, cell

def cell2opt(x_max:float, y_max:float, size:float, cell:int):
    n_cols = int(x_max / size)
    y_position, x_position  = divmod(cell, n_cols)
    return (size * x_position + size/2, size * y_position + size/2)



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

    def updateObstacles(self, obstacle:list):
        if obstacle not in (self.obstacles):
            self.obstacles.append(obstacle)
    
    def is_onboard(self, cartesian_position):
        '''
        checks if the agent is in the environment and return true or false
        '''
        x, y, z = cartesian_position
        if x < self.initGrid[0]  or x > self.endGrid[0] or y < self.initGrid[1]  or y > self.endGrid[1]:
            return 0
        return 1
    
    def set_bounds(self, startPosition, row, col, grid_size):
        x, y, z = startPosition

        new_x, new_y, new_z = x - int(grid_size/2), y - int(grid_size/2), 0
        if grid_size % 2 == 0: #If grid size is a even number
            end_x, end_y, end_z = x + int(grid_size/2) - 1, y + int(grid_size/2) - 1, 0
        else:
            end_x, end_y, end_z = x + int(grid_size/2) - 0 , y + int(grid_size/2)-0 , 0

        if new_x < 0:
            new_x = 0
            end_x = x + grid_size - 1
        
        if new_y < 0:
            new_y = 0
            end_y = y + grid_size - 1

        if end_x > row -1:
            new_x = row - grid_size
            end_x = row - 1

        if end_y > col -1:
            new_y = col - grid_size
            end_y = col - 1

        self.setMiniGrid((new_x, new_y, new_z), (end_x, end_y, end_z))

        return [(new_x, new_y, new_z), (end_x, end_y, end_z)]
    
    def set_mindist_goal(self, start_grid:tuple, end_grid:tuple, goal_position:int, minigrid_size:int):
        xi, yi, zi = start_grid
        xe, ye, ze = end_grid
        xg, yg, zg = goal_position
        print('minigrid', goal_position)
        min_dist = 10000
        for i in range(xi, xe + 1):
            for j in range(yi, ye + 1):
                if abs(i - xg) + abs(j - yg) < min_dist:
                    if self.cart2s((i, j, 0)) not in self.get_obstacles():
                        min_dist = abs(i - xg) + abs(j - yg)
                        goal = (i, j, 0)
        self.setGoal(goal=goal)
        print('GOAL: ', goal)
        return goal

class local_planner:
    def __init__(self, index:int, kd:float, ks:float, kt:float, kg:float, startPos=(0,0,0), grid_size=7, numEpisode=1000,
                 alpha = .2):
         #Creating our node,publisher and subscriber
        rospy.init_node('topic_publisher', anonymous=True)
        self.global_path = rospy.Publisher('/GlobalPath', Float64MultiArray, queue_size=1)
        self.opt_info = rospy.Subscriber('/B1/ObstaclePosition', Float64MultiArray, self.callback)
        self.uav_position = rospy.Subscriber('/B1/UAVposition', Point, self.callbackpos)
        self.opt_pos = rospy.Subscriber('/OptPos', Point, self.callbackpos)
        self.flag = 0

        self.multi_array = Float64MultiArray()
        self.__current_position = Point()
        self.rate = rospy.Rate(10)
        self.best_path = Float64MultiArray()

        self.__index = index
        self.__ks = ks
        self.__kd = kd
        self.__kt = kt
        self.__kg = kg
        self.__startPos = list(startPos)
        self.__grid_size = grid_size
        self.__numEpisode = numEpisode
        self.__alpha = alpha

        


        

        self.__load_our_map()
        
        self.__create_our_grid_world()
        self.__create_our_mini_grid()
        self.__create_our_agent()
        self.__train_agent()
    
    def callbackpos(self, msg:Point):
        self.__current_position = msg
       # 
    
    def callback(self, data:Float64MultiArray):
        if self.flag == 0:
            self.__load_our_map()
            self.__create_our_grid_world()
            self.__create_our_mini_grid()
            self.__create_our_agent()
            self.__train_agent()

        x_max = 7.35
        y_max = 4.8
        cell_size = .6
        self.multi_array = data
        state_list = list()

       # teste = [3.503674030303955, 3.932750940322876, 0.9141186475753784, 5.610137462615967, 3.3981847763061523,
       #  0.8829743266105652, 6.169996738433838, 0.8000350594520569, 0.9061737656593323, 2.9267265796661377, 0.7945838570594788, 0.9559198021888733]
       # print(len(self.multi_array.data))
        for i in range(0, len(self.multi_array.data), 3):
            _, _, cell = opt2position(x_max, y_max, cell_size, self.multi_array.data[i:i+3])
        #    print(cell)
            state_list.append(cell)
        
        


       
            
        
        self.multi_array.data = state_list


        y, x, current_position = opt2position(x_max, y_max, cell_size, (self.__current_position.x, self.__current_position.y))#self.__grid_world.s2cart(self.multi_array.data[0])
        self.__startPos[0] = x
        self.__startPos[1] = y
        obs_in_path = set(self.best_path.data) & set(self.multi_array.data) # Interseção entre dois conjuntos
        #print(self.best_path.data)
        #print(obs_in_path)
        print(f"obs: {state_list}, drone:{current_position} {y, x}", end='\r')
        

        if len(obs_in_path) > 0:
         #   print(np.where(np.array(self.best_path.data) == current_position))
            index_current_position = np.where(np.array(self.best_path.data) == current_position)[0][0]
            obs_forward_the_agent = [True for obs in obs_in_path if np.where(np.array(self.best_path.data) == obs)[0][0] > index_current_position]
            if True in obs_forward_the_agent:
                self.__obstacles = sorted(self.multi_array.data)
                self.__create_our_grid_world()
                self.__create_our_mini_grid()
                self.__create_our_agent()
                self.__train_agent()
        

        try:
            if self.best_path.data[-1] == current_position:
                #print('entrei aqui')
                self.__obstacles = sorted(self.multi_array.data)
                self.__create_our_grid_world()
                self.__create_our_mini_grid()
                self.__create_our_agent()
                self.__train_agent()
        except ValueError:
            pass


    def set_goal_position(self, goal_position:tuple) -> None:
        self.__goal_position = goal_position
    
    def get_goal_position(self)-> tuple:
        return self.__goal_position
    
    def set_minigrid_size(self, grid_size:int)->None:
        self.__grid_size = grid_size

    def __load_our_map(self):
        if opr!='linux':
            print('We are working on a Windows system')
            path = f"mapasICUAS\mapaEscolhido{self.__index}.txt"
            
        else:
            print('We are working on a Linux system')
            path = f"mapasICUAS/mapaEscolhido{self.__index}.txt"
            
        self.__maps = OpenInstance(path)
        self.__header, self.__numObstacle, self.__obstacles = self.__maps.run()
        

        self.__row = self.__header[0]
        self.__col = self.__header[1]
        try:
            self.__height = self.__header[2]
        except:
            self.__height = 1

    def __create_our_grid_world(self):
        try:
            self.get_goal_position()
        except AttributeError:
              self.set_goal_position((self.__row - 1, self.__col - 1, 0))# Objetivo global do agente

        self.__grid_world = GridWorld(self.__row, self.__col, self.__height, self.__startPos,
                                self.__kd, self.__ks, self.__kt, self.__kg)
        
        self.__grid_world.set_obstacles(self.__obstacles)
        self.__grid_world.get_reward_safety()

    def __create_our_mini_grid(self):
        # Configurar Minigrid
        self.__mini_grid = MiniGrid(self.__row, self.__col, self.__height, self.__startPos, self.__kd, self.__ks, self.__kt, self.__kg)
        self.__mini_grid.setObstacles(self.__grid_world.get_obstacles())
        init_grid, end_grid = self.__mini_grid.set_bounds(self.__startPos, self.__row, self.__col, self.__grid_size)
        self.__mini_grid.setPosition(self.__startPos)
        self.__mini_grid.set_mindist_goal(init_grid, end_grid, self.__goal_position, self.__grid_size)
        self.__mini_grid.actionSpace()
        self.__mini_grid.get_reward_safety()
    
    def __create_our_agent(self):
        self.__localAgent=qla(alpha=self.__alpha)
        self.__localAgent.setEnviroment(self.__mini_grid)
        self.__localAgent.setQtable(self.__row * self.__col, 8)
        self.__localAgent.setEpsilon(1, [1, .1, self.__numEpisode])
        self.__localAgent.setAlpha(0, [self.__alpha, .1, self.__numEpisode])
        self.__localAgent.exploringStarts(1, 1, self.__startPos)
    
    def __train_agent(self):
        init=time.time() * 1000
        self.__localAgent.train(self.__numEpisode, 10, plot=0)
        fim=time.time() * 1000 - init
        print(fim)
        #print(self.__localAgent.getBestPath(self.__startPos))

        #teste = [3.449720859527588, 3.9794921875, 0.9147161245346069, 5.6061530113220215, 3.4011101722717285, 0.8833327889442444, 6.160008430480957,
        # 0.7659165859222412, 0.9063773155212402, 2.6224820613861084, 0.34377673268318176, 0.9608232975006104]

        #for i in range(0, len(teste), 3):
        #    _, _, cell = opt2position(7.5, 4.73, .6, teste[i:i+3])
        #    print(cell)
        #    print(cell2opt(7.35, 4.8, .6, cell))

        self.__grid_world.PrintBestPath(self.__localAgent.getQtable(), 0, self.__localAgent.getBestPath(self.__startPos))
        self.best_path.data = self.__localAgent.getBestPath(self.__startPos)
        print(f'startPos:{self.__startPos}, Best path:{self.best_path.data}')
        self.global_path.publish(self.best_path)
        self.flag = 1


        
        #for data in self.best_path.data:
            #print(cell2opt(1, 1, .1, data))
    


        
if __name__ == '__main__':
    try:
        a1 = local_planner(24, 0, -1, -1, 100, alpha=.1, grid_size=5)
        rospy.spin()


    except rospy.ROSInterruptException: 
        pass
