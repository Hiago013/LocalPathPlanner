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

    xcell, ycell = int(current_10x/size_10), int(current_10y/size_10)
    cell = (n_cols) * int(current_10y/size_10) + int(current_10x/size_10)

    return xcell, ycell, cell

def cell2opt(x_max:float, y_max:float, cell_size:float, cell:int):
    n_cols = int(x_max / cell_size)
      
    i,j= divmod(cell, n_cols)
    y_cell, x_cell=i,j
    return (cell_size * x_cell + cell_size/2, cell_size * y_cell + cell_size/2)



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
        init grid and endgrid (i,j)
        '''
        i, j, k = cartesian_position
        if i < self.initGrid[0]  or i > self.endGrid[0] or j < self.initGrid[1]  or j > self.endGrid[1]:
            return 0
        return 1
    
    def actionSpace(self):
        '''
        This function set the possible action space for each state removing actions that
        would leave out of the board or towards a obstacle
        '''
        self.actions = {}

        for state in range(0, self.cart2s((self.rows-1, self.cols-1, self.height-1))+1):
            self.actions[state] = list((0,1,2,3,4,5,6,7))
            action=0
            while action<=7: 
                ni,nj,nk = self.test_move(action,state)
                if not self.is_onboard((ni,nj,nk)) or not self.is_onboard(self.s2cart(state)):
                    self.removeAction(action,state)
                action+=1

    
    def set_bounds(self, startPosition, row, col, grid_size):
        ##Se for usar, consertar pra i,j 
        #
        #
        #
        #

        i, j, k = startPosition

        new_i, new_j, new_k = i - int(grid_size/2), j - int(grid_size/2), 0
        if grid_size % 2 == 0: #If grid size is a even number
            end_i, end_j, end_k = i + int(grid_size/2) - 1, j + int(grid_size/2) - 1, 0
        else:
            end_i, end_j, end_k = i + int(grid_size/2) - 0 , j + int(grid_size/2)- 0 , 0

        if new_i < 0:
            new_i = 0
            end_i = i + (grid_size - 1)
        
        if new_j < 0:
            new_j = 0
            end_j = j + (grid_size - 1)

        if end_i > row -1:
            new_i = row - grid_size
            end_i = row - 1

        if end_j > col -1:
            new_j = col - grid_size
            end_j = col - 1

        self.setMiniGrid((new_i, new_j, new_k), (end_i, end_j, end_k))

        return [(new_i, new_j, new_k), (end_i, end_j, end_k)]
    
    def get_empty_cell_minigrid(self):
        ii, ji, _ = self.initGrid
        ie, je, _ = self.endGrid
        nonobstacles = []
        for i in range(ii, ie + 1):
            for j in range(ji, je + 1):
                if self.cart2s((i, j, 0)) not in self.get_obstacles():
                    nonobstacles.append(self.cart2s((i, j, 0)))
        return nonobstacles
    
    def get_empty_cell_minigrid2(self, init:tuple, end:tuple):
        ii, ji, _ = init
        ie, je, _ = end
        nonobstacles = []
        for i in range(ii, ie + 1):
            for j in range(ji, je + 1):
                if self.cart2s((i, j, 0)) not in self.get_obstacles():
                    nonobstacles.append(self.cart2s((i, j, 0)))
        return nonobstacles
        
    
    def set_mindist_goal(self, start_grid:tuple, end_grid:tuple, goal_position:int, minigrid_size:int):
        ii, ji, ki = start_grid
        ie, je, ke = end_grid
        ig, jg, kg = goal_position
        print('minigrid', goal_position)
        min_dist = 10000
        for i in range(ii, ie + 1):
            for j in range(ji, je + 1):
                if abs(i - ig) + abs(j - jg) < min_dist:
                    if self.cart2s((i, j, 0)) not in self.get_obstacles():
                        min_dist = abs(i - ig) + abs(j - jg)
                        goal = (i, j, 0)
        self.setGoal(goal=goal)
        print('GOAL: ', goal)
        return goal

    def get_bounds(self, start_position:tuple, row:int, col:int, action:int, grid_size:int) -> list:
        i, j, k = start_position
        
        if action == 0: # If action equals zero, so the agente must go down
            start_i, start_j, start_k = i , j- int(grid_size/2) , 0
            end_i, end_j, end_k = i + (grid_size-1), j + int(grid_size/2), 0

          #  return [(start_i, start_j, start_k), (end_i, end_j, end_k)]

        elif action == 1: # If action equals one, so the agente must go up
            start_i, start_j, start_k = i - (grid_size-1), j- int(grid_size/2) , 0
            end_i, end_j, end_k = i , j + int(grid_size/2), 0

          #  return [(start_i, start_j, start_k), (end_i, end_j, end_k)]
        
        elif action == 2: # If action equals two, so the agent must go right
            start_i, start_j, start_k = i-int(grid_size/2), j , 0
            end_i, end_j, end_k = i +  int(grid_size/2),  j+(grid_size-1), 0

           # return [(start_i, start_j, start_k), (end_i, end_j, end_k)]
        
        elif action == 3: # If action equals three, so the agent must go left
            start_i, start_j, start_k = i-int(grid_size/2), j - (grid_size-1), 0
            end_i, end_j, end_k = i+int(grid_size/2), j , 0

          #  return [(start_i, start_j, start_k), (end_i, end_j, end_k)]
        
        elif action == 4: # If action equals four, so the agent must go down-right
            start_i, start_j, start_k = i , j , 0
            end_i, end_j, end_k = i + grid_size - 1, j + grid_size - 1, 0

           # return [(start_i, start_j, start_k), (end_i, end_j, end_k)]
        
        elif action == 5: # If action equals five, so the agent must go down-left
            start_i, start_j, start_k = i, j-(grid_size-1), 0
            end_i, end_j, end_k = i+(grid_size-1), j, 0

          #  return [(start_i, start_j, start_k), (end_i, end_j, end_k)]
        
        elif action == 6: # If action equals six, so the agent must go up-right
            start_i, start_j, start_k = i-(grid_size-1) , j , 0
            end_i, end_j, end_k = i ,j+(grid_size-1) , 0

           # return [(start_i, start_j, start_k), (end_i, end_j, end_k)]
        
        elif action == 7: # If action equals seven, so the agent must go up-left
            start_i, start_j, start_k = i - (grid_size-1), j - (grid_size-1), 0
            end_i, end_j, end_k = i, j, 0

            #return [(start_i, start_j, start_k), (end_i, end_j, end_k)]

        else: # Otherwise the agent must be in the center of grid
            start_i, start_j, start_k = i - int(grid_size/2), j - int(grid_size/2), 0
            end_i, end_j, end_k = i + int(grid_size/2), j + int(grid_size/2), 0
            
        return [(max(0, start_i), max(0, start_j), 0), (min(row - 1, end_i), min(col - 1, end_j), 0)]
    
    def get_last_action(self, previous_state:tuple, current_state:tuple) -> int:
        i_p, j_p, k_p = previous_state
        i_c, j_c, k_c = current_state

        if (i_c == i_p + 1) and (j_c == j_p):
            return 0
        
        elif (i_c == i_p - 1) and (j_c == j_p):
            return 1
        
        elif (i_c == i_p) and (j_c == j_p + 1):
            return 2
        
        elif (i_c == i_p) and (j_c == j_p - 1):
            return 3
        
        elif (i_c == i_p + 1) and (j_c == j_p + 1):
            return 4
        
        elif (i_c == i_p + 1) and (j_c == j_p - 1):
            return 5
        
        elif (i_c == i_p - 1) and (j_c == j_p + 1):
            return 6
        
        elif (i_c == i_p - 1) and (j_c == j_p - 1):
            return 7


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

        self._visited_states = list()
        self._last_action = None
        self.__index = index
        self.__ks = ks
        self.__kd = kd
        self.__kt = kt
        self.__kg = kg
        self.__startPos = list(startPos)
        self.__grid_size = grid_size
        self.__numEpisode = numEpisode
        self.__alpha = alpha
        self.__current_state = 0
        self._empty_cell_minigrid = list()


        self.__load_our_map()
        self.__create_our_grid_world()
        self.__create_our_mini_grid()
        self.__create_our_agent()
        self.__train_agent()
    
    def callbackpos(self, msg:Point):
        self.__current_position = msg

    
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

        for i in range(0, len(self.multi_array.data), 3):
            _, _, cell = opt2position(x_max, y_max, cell_size, self.multi_array.data[i:i+3])
            state_list.append(cell)
        self.multi_array.data = state_list


        xcell, ycell, current_position = opt2position(x_max, y_max, cell_size, (self.__current_position.x, self.__current_position.y))
        i_c, j_c = ycell, xcell
        self.__current_state = current_position
        self.__startPos[0] = i_c
        self.__startPos[1] = j_c

        if current_position not in self._visited_states:
            self._visited_states.append(current_position)
        
        if len(self._visited_states) == 2:
            self._last_action = self.__mini_grid.get_last_action(self.__mini_grid.s2cart(self._visited_states[0]), self.__mini_grid.s2cart(self._visited_states[1]))
            print('\n', self._last_action, '\n')
            del self._visited_states[0]
        
        action = self.__localAgent.chooseBestAction(self.__current_state)
        init, end = self.__mini_grid.get_bounds(self.__startPos, self.__row, self.__col, action, self.__grid_size)
        previous_empty_cells = self.__mini_grid.get_empty_cell_minigrid2(init, end)
        

        obs_in_path = set(self.best_path.data) & set(self.multi_array.data)           # Interseção entre dois conjuntos
        obs_in_minigrid = set(previous_empty_cells) & set(self.multi_array.data) # Interseção entre dois conjunto
        #obs_in_minigrid = set(self._empty_cell_minigrid) & set(self.multi_array.data) # Interseção entre dois conjunto
    
        

        
        #print(self.best_path.data)
        #print(obs_in_path)
        print(f"obs: {state_list}, drone:{current_position} {i_c, j_c}, obsminigrid: {obs_in_minigrid}", end='\r')

        

        try:
            if current_position in self.best_path.data[-2:]:
                self.__obstacles = sorted(self.multi_array.data)
                self.__create_our_grid_world()
                self.__create_our_mini_grid()
                self.__create_our_agent()
                self.__train_agent()
        except ValueError:
            pass

        if (len(obs_in_minigrid) > 0):
            self.__obstacles = sorted(self.multi_array.data)
            self.__create_our_grid_world()
            self.__create_our_mini_grid()
            self.__create_our_agent()
            self.__train_agent()
        
        self.global_path.publish(self.best_path)



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
        
        # Modificacao novo minigrid
        try:
            if self._last_action != None:
                action = self._last_action
            else:
                action = self.__localAgent.chooseBestAction(self.__current_state)
            init_grid, end_grid = self.__mini_grid.get_bounds(self.__startPos, self.__row, self.__col, action, self.__grid_size)
            print(f'\ncurrent action: {action}')
            print(f'\ninit: {init_grid}\nend: {end_grid}\n')
            self.__mini_grid.setMiniGrid(init_grid, end_grid)
        except AttributeError:
            init_grid, end_grid = self.__mini_grid.set_bounds(self.__startPos, self.__row, self.__col, self.__grid_size)
        self.__mini_grid.setPosition(self.__startPos)
        self.__mini_grid.set_mindist_goal(init_grid, end_grid, self.__goal_position, self.__grid_size)
        self.__mini_grid.actionSpace()
        if self._last_action != None:
            self.__mini_grid.last_action = self._last_action
            self.__mini_grid.get_energyCost()
        self.__mini_grid.get_reward_safety()

        #print(self.__mini_grid.get_bounds((0,0,0), 10, 10, 4, 5))
    
    def __create_our_agent(self):
        self.__localAgent=qla(alpha=self.__alpha, epsilon=.3)
        self.__localAgent.setEnviroment(self.__mini_grid)
        self.__localAgent.setQtable(self.__row * self.__col, 8)
        self.__localAgent.setEpsilon(1, [.8, .1, self.__numEpisode])
        self.__localAgent.setAlpha(0, [self.__alpha, .1, self.__numEpisode])

        length_i = self.__mini_grid.endGrid[0] - self.__mini_grid.initGrid[0] + 1
        length_j = self.__mini_grid.endGrid[1] - self.__mini_grid.initGrid[1] + 1

        #self.__localAgent.exploringStarts(length_i, length_j, self.__mini_grid.initGrid)
        self.__localAgent.exploringStarts(1, 1, self.__startPos)
    
    def __train_agent(self):
        init=time.time() * 1000
        self.__localAgent.train(self.__numEpisode, 1, plot=0)
        self._empty_cell_minigrid = self.__mini_grid.get_empty_cell_minigrid()
        fim=time.time() * 1000 - init
        print(fim)


        self.__grid_world.PrintBestPath(self.__localAgent.getQtable(), 0, self.__localAgent.getBestPath(self.__startPos))
        self.best_path.data = self.__localAgent.getBestPath(self.__startPos)
        print(f'Dijkstra Path: {self.__localAgent.getDijkstraPath(self.__startPos)}')
        print(f'startPos:{self.__startPos}, Best path:{self.best_path.data}')
        self.global_path.publish(self.best_path)
        self.flag = 1
        #dijsktra = list(self.__localAgent.getDijkstraPath(self.__startPos))
        #print('\n\n')
        #self.__grid_world.PrintBestPath(self.__localAgent.getQtable(), 0, dijsktra)
        #self.__mini_grid.printMiniGrid(self.__mini_grid.initGrid, self.__mini_grid.endGrid, self.best_path.data, self.best_path.data, self.__localAgent.getQtable())


    def _check_diagonal_obstacles(self, obstacles_list:list)-> list:
        new_list_obstacles = obstacles_list.copy()
        for obstacle in obstacles_list:
            if (obstacle + 1 - self.__col in obstacles_list) and (obstacle % self.__col < self.__col):
                new_list_obstacles.append(obstacle + 1)
                new_list_obstacles.append(obstacle + self.__col)
            if (obstacle - 1 - self.__col in obstacles_list) and (obstacle % self.__col > 0):
                new_list_obstacles.append(obstacle - 1)
                new_list_obstacles.append(obstacle - self.__col)
        
        return list(set(new_list_obstacles))

        
if __name__ == '__main__':
    try:
        a1 = local_planner(24, -.4, -.1, -.4, 100, alpha=.1, grid_size=5, numEpisode=1000)
        rospy.spin()


    except rospy.ROSInterruptException: 
        pass
