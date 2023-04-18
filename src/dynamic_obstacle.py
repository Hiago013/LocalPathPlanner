from GridWorld import GridWorld
import numpy as np
class dynamic_obstacle:
    def __init__(self, grid_world:GridWorld, current_position:tuple, static_obstacles:list):
        self.__grid_world = grid_world
        self.__current_position = current_position
        self._rows = grid_world.rows
        self._cols = grid_world.cols
        self.__static_obstacles = static_obstacles
    
    def set_current_position(self, position):
        self.__current_position = position
    
    def set_grid_world(self, grid_world):
        self.__grid_world = grid_world
    
    def set_static_obstacles(self, static_obstacle):
        self.__static_obstacles = static_obstacle
    
    def get_current_position(self):
        return self.__current_position
    
    def get_grid_world(self):
        return self.__grid_world
    
    def get_static_obstacles(self):
        return self.__static_obstacles
        
    def random_move(self):
        '''
        This function random move the obstacle to a new possible state.
        The possible state is a free cell, therefore that cell don't there
        agent or obstacles there.
        '''
        xo, yo, zo = self.get_current_position()
        possible_obstacles = []
        for i in range(xo-1, xo+2):
            for j in range(yo-1, yo+2):
                # Check if obstacle is into the environment
                if i >= 0 and j>= 0:
                    if i < self._rows and j < self._cols:
                        if (i,j) != (xo, yo):
                            # Check if obstacle chosen already there
                            # Check if the new obstacle postion is the current agent position
                            if (not self.__grid_world.cart2s((i,j,zo)) in self.get_static_obstacles()) and \
                            (not (i,j,zo) == self.__grid_world.get_current_position()):
                                possible_obstacles.append(self.__grid_world.cart2s((i,j,zo)))

        self.set_current_position(self.__grid_world.s2cart(np.random.choice(possible_obstacles)))
