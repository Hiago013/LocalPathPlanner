import numpy as np

class GridWorld:
    def __init__(self, rows, cols, height, start, Kd, Ks, Kt, Kg):
        '''
        Defines Rewards according to 3 parameters
            Kd --> Constant reward for obstacle proximity
            Ks --> Constant reward  for each step
            Kt --> Constant reward for changing routes this should encourage straigh lines
            Kg --> Reward for reaching the goal
            start --> Initial position (x, y, z)
        '''
        #Define reward
        self.Ks = Ks
        self.Kd = Kd
        self.Kt = Kt
        self.Kg = Kg
        self.reward = 0
        self.reward_safety = []

        # set grid world size
        self.rows = rows #number of rows, so if rows=4, we will have rows labeld from 0-3
        self.cols = cols #number of columns, so if cols=4, we will have columns labeld from 0-3
        self.height = height #number of columns, so if cols=4, we will have columns labeld from 0-3

        # set initial position
        self.start = start
        self.i = start[0] #starting i
        self.j = start[1] #starting j
        self.k = 0

        # set goal position
        self.goal = (self.rows-1,self.cols-1, self.height-1)

        # set done verify
        self.done = False # Means the episode hasn't ended

        # set obtacles position
        self.obstacles=[]

        # set last action
        self.last_action = 0

        # list of actions taken
        self.current_action = 0

        # action space
        self.actions = {}
        self.get_energyCost()
        self.QTable=[]
    
    def get_obstacles(self):
        return self.obstacles
    
    def set_obstacles(self, obstacles):
        self.obstacles = obstacles
        self.get_reward_safety()
    
    def get_current_position(self):
        return self.i, self.j, self.k
    
    def set_current_position(self, position:tuple):
        self.i, self.j, self.k = position
    
    def refine_reward(self, action:int, previous_position:tuple, current_position:tuple):
        i_p, j_p, k_p = previous_position
        i_c, j_c, k_c = current_position
        k = 20
        if action == 4: ## ARRUMAR j_c -1 > 0
            
            if (self.cart2s((i_c, j_c - 1, k_c)) in self.get_obstacles()) and (j_c > 0) or \
            (self.cart2s((i_c - 1, j_c, k_c)) in self.get_obstacles()) and (i_c > 0):
              #  print('entrei4', self.cart2s(current_position))
              #  print(self.cart2s((i_c -1, j_c, k_c)), self.cart2s((i_c , j_c - 1, k_c)))
                return 1#self.Kd * k
            
        elif action == 5:
            if (self.cart2s((i_c, j_c + 1, k_c)) in self.get_obstacles()) and (j_c < self.cols - 1)or \
            (self.cart2s((i_c - 1, j_c, k_c)) in self.get_obstacles()) and (i_c > 0):
               # print('entrei5')
                return 1#self.Kd * k

        elif action == 6:
            if (self.cart2s((i_c, j_c - 1, k_c)) in self.get_obstacles()) and (j_c > 0) or \
            (self.cart2s((i_c + 1, j_c, k_c)) in self.get_obstacles()) and (i_c < self.rows- 1):
                #print('entrei6')
                return 1#self.Kd * k
        

        elif action == 7:
            if (self.cart2s((i_c +1, j_c, k_c)) in self.get_obstacles()) and (i_c < self.rows - 1) or \
            (self.cart2s((i_c , j_c + 1, k_c)) in self.get_obstacles()) and (j_c < self.cols - 1):
                #print('entrei7')
                return 1#self.Kd * k

        return 0

        
    def get_energyCost(self):
        '''
            this function return the energy cost of the current state
            0:Down
            1:Up
            2:Right
            3:Left
            4:Down and Right
            5:Down and Left
            6:Up and Right
            7:Up and Left
            8:Upside
            9:Downside
        '''

        action_vector = {0: np.array([0, -1]),
                        1: np.array([0, 1]),
                        2: np.array([1, 0]),
                        3: np.array([-1, 0]),
                        4: np.array([1, -1])/np.sqrt(2),
                        5: np.array([-1, -1])/np.sqrt(2),
                        6: np.array([1, 1])/np.sqrt(2),
                        7: np.array([-1, 1])/np.sqrt(2)}

        self.energyCost = {}
        for i in range(8):
            for j in range(i+1, 8):
                self.energyCost[(i, j)] = self.Kt * np.arccos(np.dot(action_vector[i], action_vector[j])) / np.pi
    
    def cart2s(self, cartesian_position):
        ''' returns the position in the cell given the 3D Cartesian coordinate 
        (i, j, k) -> s
        '''
        i, j, k = cartesian_position
        s = self.cols * i + j + self.cols * self.rows * k
        return s
    
    def s2cart(self, s_position):
        ''' returns the position in the 3D Cartesian coordinate given the cell
        s -> (i, j, k) 
        '''
        numCelInCrossSection = self.rows * self.cols
        i = int(s_position /(self.cols))
        j = s_position % (self.cols)
        k = 0

        if s_position < numCelInCrossSection:
            return (i, j, k)

        else:
            while s_position >= numCelInCrossSection:
                s_position -= numCelInCrossSection
                k += 1
            i = int(s_position /(self.cols))
            j = s_position % (self.cols)
            return (i, j, k) 

    def test_move(self,action,state):
        i, j, k =self.s2cart(state)

        ai = self.i
        aj = self.j
        ak = self.k

        self.i = i
        self.j = j
        self.k = k

        if action == 0:  
            new_i = self.i+1
            new_j = self.j
            new_k = self.k
        elif action == 1:
            new_i = self.i-1
            new_j = self.j
            new_k = self.k
        elif action == 2:
            new_i = self.i
            new_j = self.j+1
            new_k = self.k
        elif action == 3:
            new_i = self.i
            new_j = self.j-1
            new_k = self.k
        elif action == 4:
            new_i = self.i+1
            new_j = self.j+1
            new_k = self.k
        elif action == 5:
            new_i = self.i+1
            new_j = self.j-1
            new_k = self.k
        elif action == 6:
            new_i = self.i-1
            new_j = self.j+1
            new_k = self.k
        elif action == 7:
            new_i = self.i-1
            new_j = self.j-1
            new_k = self.k
        elif action == 8:
            new_i = self.i
            new_j = self.j
            new_k = self.k + 1
        elif action == 9:
            new_i = self.i
            new_j = self.j
            new_k = self.k - 1
        
        else:
            print("Ação inválida.")
            self.i=ai
            self.j=aj
            self.k=ak
            return -1
        self.i=ai
        self.j=aj
        self.k=ak
        return (new_i,new_j,new_k)
        
    def step(self, action):
        ''' return new state, reward and boolean value to verify if the new state = goal 
            0:Down
            1:Up
            2:Right
            3:Left
            4:Down and Right
            5:Down and Left
            6:Up and Right
            7:Up and Left
            8:Upside
            9:Downside
        '''
        reward=0
        if action == 0:  
            new_i = self.i+1
            new_j = self.j
            new_k = self.k
        elif action == 1:
            new_i = self.i-1
            new_j = self.j
            new_k = self.k
        elif action == 2:
            new_i = self.i
            new_j = self.j+1
            new_k = self.k
        elif action == 3:
            new_i = self.i
            new_j = self.j-1
            new_k = self.k
        elif action == 4:
            new_i = self.i+1
            new_j = self.j+1
            new_k = self.k
        elif action == 5:
            new_i = self.i+1
            new_j = self.j-1
            new_k = self.k
        elif action == 6:
            new_i = self.i-1
            new_j = self.j+1
            new_k = self.k
        elif action == 7:
            new_i = self.i-1
            new_j = self.j-1
            new_k = self.k
        elif action == 8:
            new_i = self.i
            new_j = self.j
            new_k = self.k + 1
        elif action == 9:
            new_i = self.i
            new_j = self.j
            new_k = self.k - 1
        else:
            print("Ação inválida.")
        
        if not self.is_onboard((new_i, new_j, new_k)):
            # Cancel action
            new_i = self.i
            new_j = self.j
            new_k = self.k
        if self.cart2s((new_i, new_j, new_k)) in self.obstacles:
            # Cancel action
            new_i = self.i
            new_j = self.j
            new_k = self.k
            reward=-self.Kg
            self.done=True
        
        reward += self.reward_safety[(new_i, new_j, new_k)] # reward of safety
        reward += self.get_reward() # reward of distance and energy
        if self.refine_reward(action, (self.i, self.j, self.k), (new_i, new_j, new_k)):
            reward -= self.Kg
            self.done = True
        #reward += self.refine_reward(action, (self.i, self.j, self.k), (new_i, new_j, new_k)) ## LINHA DO "TIRA QUINA"
        new_state = (new_i, new_j, new_k)

        if new_state == self.goal:
            self.done = True
        return (new_state), reward, self.done
    
    def get_reward(self):
        '''
        this function return a reward for each step
        '''
        reward = 0
        reward = 0
        if self.current_action != self.last_action:
            if self.current_action <= 7 and self.last_action <= 7:
                try:
                    reward = self.energyCost[(self.last_action, self.current_action)]
                except:
                    reward = self.energyCost[(self.current_action, self.last_action)]
            
        
        if self.current_action > 3 and self.current_action <= 7:
            reward += self.Ks * 1.4
        elif self.current_action < 4:
            reward += self.Ks
        elif self.current_action == 8:
            reward += self.Ks * 1
        elif self.current_action == 9:
            reward += self.Ks * 1
        return reward

    
    def get_reward_safety(self):
        '''
        this function create safety reward
        '''
        reward_step = np.zeros([self.rows, self.cols, self.height])
        reward_step[self.goal] = self.Kg

        reward_safety = np.ones([self.rows, self.cols, self.height])

        for i in range(self.rows):
            for j in range(self.cols):
                for k in range(self.height):
                    obstacleList = []
                    for obstacle in self.obstacles:
                        oi, oj, ok = self.s2cart(obstacle)
                        #oz=oz/2
                        distance = np.sqrt((i - oi)**2 + (j - oj)**2 + (k - ok)**2)
                        obstacleList.append(distance)
                    obstacleList.sort()
                    obstacleList = obstacleList[0:10]
                    penalty = 0
                    

                    for it in range(len(obstacleList)):
                        penalty += min(0, (3 - obstacleList[it]) * self.Kd)
                                       
                    reward_safety[i][j][k] = penalty * reward_safety[i][j][k]
        self.reward_safety = reward_safety + reward_step

    def actionSpace(self):
        '''
        This function set the possible action space for each state removing actions that
        would leave out of the board or towards a obstacle
        '''
        self.actions = {}

        for state in range(0, self.cart2s((self.rows-1, self.cols-1, self.height-1))+1):
            self.actions[state] = list((0,1,2,3,4,5,6,7))
            #print("Actions for state ",state,' before')
            #print(self.actions[state])
            action=0
            while action<=7: 
                ni,nj,nk = self.test_move(action,state)
                #print(nx,ny,nz, ' For action',action, 'in state',state)
                if self.is_onboard((ni,nj,nk)):
                    pass
                else:
                    #print('Removing Action',action,' From state',state)
                    #input('Press to continue')
                    self.removeAction(action,state)
                #print("Actions for state ",state,' after removal')
                #print(self.actions[state])
                action+=1
        
        # for obstacle in self.obstacles:
        #     '''
        #     x -> raw
        #     y -> cols
        #     z -> height
        #     '''
        #     x, y, z = self.s2cart(obstacle)

        #     if self.is_onboard((x-1, y, z)): # remove down action (0)
        #         self.removeAction(0, self.cart2s((x-1, y, z)))
        #     if self.is_onboard((x+1, y, z)): # remove up action (1)
        #         self.removeAction(1, self.cart2s((x+1, y, z)))
        #     if self.is_onboard((x, y-1, z)): # remove right action (2)
        #         self.removeAction(2, self.cart2s((x, y-1, z)))
        #     if self.is_onboard((x, y+1, z)): # remove left action (3)
        #         self.removeAction(3, self.cart2s((x, y+1, z)))
        #     if self.is_onboard((x+1, y+1, z)): # remove up and left action (7)
        #         self.removeAction(7, self.cart2s((x+1, y+1, z)))
        #     if self.is_onboard((x-1, y+1, z)): # remove down and left action (5)
        #         self.removeAction(5, self.cart2s((x-1, y+1, z)))
        #     if self.is_onboard((x+1, y-1, z)): # remove up and right action (6)
        #         self.removeAction(6, self.cart2s((x+1, y-1, z)))
        #     if self.is_onboard((x-1, y-1, z)): # remove down and right action (4)
        #         self.removeAction(4, self.cart2s((x-1, y-1, z)))
            
            #print('ok2')


    def removeAction(self, index, state):
        '''
        this function remove actions
        '''
        if index in self.actions[state]:
            self.actions[state].remove(index)


    
    def is_onboard(self, cartesian_position):
        '''
        checks if the agent is in the environment and return true or false
        '''
        i, j, k = cartesian_position
        if i < 0 or i >= self.rows or j < 0 or j >= self.cols or k < 0 or k >= self.height:
            return 0
        return 1
    
    def reset(self, start=0):
        if start==0:
            self.i = self.start[0]
            self.j = self.start[1]
            self.k = self.start[2]
        else:
            self.i = start[0]
            self.j = start[1]
            self.k = start[2]
        self.done = False

    def PrintBestAction(self, Q, k):
            for i in range(0, self.rows):
                print(72*'-')
                for j in range(0, self.cols):
                    if self.cart2s((i,j,k)) in self.obstacles:
                        print('  X  ',end='|')
                    else:
                        if np.argmax(Q[self.cart2s((i,j,k)),:])==0:
                            print('  D  ',end='|')
                        elif np.argmax(Q[self.cart2s((i,j,k)),:])==1:
                            print('  U  ',end='|')
                        elif np.argmax(Q[self.cart2s((i,j,k)),:])==2:
                            print('  R  ',end='|')
                        elif np.argmax(Q[self.cart2s((i, j, k)),:])==3:
                            print('  L  ',end='|')
                        elif np.argmax(Q[self.cart2s((i, j, k)),:])==4:
                            print('  DR ',end='|')
                        elif np.argmax(Q[self.cart2s((i, j, k)),:])==5:
                            print('  DL ',end='|')
                        elif np.argmax(Q[self.cart2s((i, j, k)),:])==6:
                            print('  UR ',end='|')
                        elif np.argmax(Q[self.cart2s((i, j, k)),:])==7:
                            print('  UL ',end='|')
                        elif np.argmax(Q[self.cart2s((i, j, k)),:])==8:
                            print('  S  ',end='|')
                        elif np.argmax(Q[self.cart2s((i, j, k)),:])==9:
                            print('  B  ',end='|')
                    
                    if j==(self.cols- 1):
                        print()
            print()
    
    def debug(self, q_table, origin = (0,0,0), reset = False):
        actions = {0:'Down',
                    1:'Up',
                    2:'Right',
                    3:'Left',
                    4:'Down and Right',
                    5:'Down and Left',
                    6:'Up and Right',
                    7:'Up and Left',
                    8:'Upside',
                    9:'Downside'}
        if reset == True:
            
            self.i, self.j, self.k = origin
            self.done = False
        else:
            best_action = np.argmax(q_table[self.cart2s((self.i, self.j, self.k))])
            print(f'action taken: {actions[best_action]}')
            state, reward, done = self.step(best_action)
            self.i, self.j, self.k = state

    def get_distance(self,Path):
        dist=[]
        for i in range(0,len(Path)):
            try:
                cart1=np.array(self.s2cart(Path[i]))
                #cart1[2]/=2
                cart2=np.array(self.s2cart(Path[i+1]))
                #cart2[2]/=2
                dist.append(np.linalg.norm(cart1-cart2))
            except:
                pass
        return sum(dist)

    def get_meanDist(self,Path):
        #get mean of minimum distance
        dist=[]

        
        for cell in Path:
            obstacleList = []
            i,j,k=self.s2cart(cell)
            k/=2
            for obstacle in self.obstacles:
                oi, oj, ok = self.s2cart(obstacle)
                #oz=oz/2
                distance = np.sqrt((i - oi)**2 + (j - oj)**2 + (k - ok)**2)
                obstacleList.append(distance)
            obstacleList.sort()
            dist.append(sum(obstacleList[0:10]))
        return np.mean(dist)
        
        

    def onMap(self):
        k = self.k
        for i in range(self.rows):
            print(72 * '-')
            for j in range(self.cols):
                if self.cart2s((i, j, k)) in self.obstacles:
                    print('  X  ', end = '|')
                elif (i, j, k) == (self.i, self.j, self.k):
                    print('  A  ', end = '|')
                else:
                    print('     ', end='|')
            print('')
    
    def getGrafo(self):
        row = self.rows
        col = self.cols
        height = self.height
        grafo = {}
        for k in range(height):
            for i in range(row):
                for j in range(col):
                    # if k > 0 and k < height-1:
                        if i > 0 and i < row-1 and j>0 and j < col-1:
                            grafo[(i, j, k)] = {(i+1,j,k) : 1,
                                                (i-1,j,k): 1,
                                                (i,j+1,k): 1,
                                                (i,j-1,k): 1,
                                                (i+1,j+1,k): np.sqrt(2),
                                                (i+1,j-1,k): np.sqrt(2),
                                                (i-1,j-1,k): np.sqrt(2),
                                                (i-1,j+1,k): np.sqrt(2),
                                                (i,j,k+1): .5,
                                                (i,j,k-1): .5
                                                }
                        elif (i == 0) and (j > 0 and j < col-1):
                            grafo[(i, j, k)] = {(i+1,j,k): 1,
                                                (i, j-1, k): 1,
                                                (i, j+1, k): 1,
                                                (i+1, j+1, k): np.sqrt(2),
                                                (i+1, j-1, k): np.sqrt(2),
                                                (i, j, k+1): .5,
                                                (i, j, k-1): .5}
                        elif (i == row-1) and (j > 0 and j < col-1):
                            grafo[(i, j, k)] = {(i-1,j,k): 1,
                                                (i, j-1, k): 1,
                                                (i, j+1, k): 1,
                                                (i-1, j+1, k): np.sqrt(2),
                                                (i-1, j-1, k): np.sqrt(2),
                                                (i, j, k+1): .5,
                                                (i, j, k-1): .5}
                        elif (j == 0) and (i>0 and i < row-1):
                            grafo[(i, j, k)] = {(i+1,j,k):1,
                                                (i-1,j,k): 1,
                                                (i,j+1,k): 1,
                                                (i+1,j+1,k): np.sqrt(2),
                                                (i-1,j+1,k): np.sqrt(2),
                                                (i,j,k+1): .5,
                                                (i,j,k-1): .5}
                        elif (j == col-1) and (i>0 and i < row-1):
                            grafo[(i, j, k)] = {(i+1,j,k): 1,
                                                (i-1,j,k): 1,
                                                (i,j-1,k): 1,
                                                (i+1,j-1,k): np.sqrt(2),
                                                (i-1,j-1,k): np.sqrt(2),
                                                (i,j,k+1): .5,
                                                (i,j,k-1): .5}
                        elif (i == 0 and j == 0):
                            grafo[(i, j, k)] = {(i+1,j,k): 1,
                                                (i+1,j+1,k): np.sqrt(2),
                                                (i,j+1,k): 1,
                                                (i,j,k+1): .5,
                                                (i,j,k-1): .5}
                        elif (i == 0 and j == col-1):
                            grafo[(i, j, k)] = {(i+1,j,k): 1,
                                                (i+1,j-1,k): np.sqrt(2),
                                                (i,j-1,k): 1,
                                                (i,j,k+1): .5,
                                                (i,j,k-1): .5}
                        elif (i == row -1 and j == 0):
                            grafo[(i, j, k)] = {(i-1,j,k): 1,
                                                (i-1,j+1,k): np.sqrt(2),
                                                (i,j+1,k): 1,
                                                (i,j,k+1): .5,
                                                (i,j,k-1): .5}
                        elif (i == row-1 and j == col -1):
                            grafo[(i, j, k)] = {(i-1,j,k): 1,
                                                (i-1,j-1,k): np.sqrt(2),
                                                (i,j-1,k): 1,
                                                (i,j,k+1): .5,
                                                (i,j,k-1): .5}
                        if k == 0:
                            del grafo[(i, j, k)][(i,j,k-1)]
                        if k == height-1:
                            del grafo[(i, j, k)][(i,j,k+1)]
        new_grafo = {}
        for key in grafo.keys():
            auxState = []
            auxDistance = []
            for item in grafo[key].items():
                if self.cart2s(item[0]) not in self.obstacles:
                    auxState.append(self.cart2s(item[0]))
                    auxDistance.append(item[1])
            new_grafo[self.cart2s(key)] = dict(zip(auxState, auxDistance))
        return new_grafo
    
    def getBestPath(self, initial, Q_table):
        '''
        this function return the best path for the robot to follow using Q-learning
        '''
        bestPath = []

        # get the best path
        self.debug(Q_table, initial, reset=True)
        while not self.done:
            state = self.cart2s((self.i, self.j, self.k))
            best_action = np.argmax(Q_table[state])
            newState, reward, done = self.step(best_action)
            self.i, self.j, self.k = newState
            bestPath.append(self.cart2s(newState))
        return bestPath
    
    def testConvergence(self, Q_table):
        '''
        this function test if the robot has converged to a stable state
        '''

        Fail = 0
        maxSteps = self.rows * self.cols * self.height
        countStep = 0
        countStates = 0
        
        for i in range(self.rows):
            for j in range(self.cols):
                for k in range(self.height):
                    if self.cart2s((i,j,k)) not in self.obstacles and self.is_onboard((i, j, k)):
                        countStates += 1
                        countStep = 0
                        self.debug(Q_table, (i, j, k), reset=True)
                        while (not self.done) and (maxSteps > countStep):
                            state = self.cart2s((self.i, self.j, self.k))
                            best_action = np.argmax(Q_table[state])
                            newState, reward, done = self.step(best_action)
                            self.i, self.j, self.k = newState
                            countStep += 1
                            self.done = done
                        if self.done == False:
                            Fail += 1
                            #print("Fail: ", i,j,k)
        return np.round(100 * Fail / countStates, 2)

    def PrintBestPath(self, Q, k, path):
            print(self.cols*'---', end='')
            for i in range(0, self.rows):
                print('')
                for j in range(0, self.cols):
                    if self.cart2s((i,j,k)) in self.obstacles:
                        print(' X ',end='')
                    else:
                        if self.cart2s((i,j,k)) in path:
                            if np.argmax(Q[self.cart2s((i,j,k)),:])==0:
                                print(' v ',end='')
                            elif np.argmax(Q[self.cart2s((i,j,k)),:])==1:
                                print(' ^ ',end='')
                            elif np.argmax(Q[self.cart2s((i,j,k)),:])==2:
                                print(' > ',end='')
                            elif np.argmax(Q[self.cart2s((i, j, k)),:])==3:
                                print(' < ',end='')
                            elif np.argmax(Q[self.cart2s((i, j, k)),:])==4:
                                print(' \ ',end='')
                            elif np.argmax(Q[self.cart2s((i, j, k)),:])==5:
                                print(' / ',end='')
                            elif np.argmax(Q[self.cart2s((i, j, k)),:])==6:
                                print(' / ',end='')
                            elif np.argmax(Q[self.cart2s((i, j, k)),:])==7:
                                print(' \ ',end='')
                        else:
                            print('   ', end='')
                    
                    if j==(self.cols- 1):
                        print("|")
            print(self.cols*'---')

    def printMiniGrid(self, initialPosition, endPosition, globalPath = None, localPath = None, qtableLocal = None):
        '''
        @param: initialPosition e.g. (0,0,0)
        @param: endPosition e.g. (9,9,0)
        @param: globalPath e.g. [0, 9, 18, 28, 38, 47, 57, 58, 68, 78, 79, 80]
        @param: localPath e.g. [0, 9, 17, 18]
        @param: qtableLocal
        @return None
        '''
        grid = np.zeros((self.rows+2, self.cols+2))
        verticalEsquerda = [(row, initialPosition[1]) for row in range(initialPosition[0], endPosition[0] + 1)]
        verticalDireita = [(row, endPosition[1]) for row in range(initialPosition[0], endPosition[0] + 1)]
        horizontalSuperior = [(initialPosition[0], col) for col in range(initialPosition[1], endPosition[1] + 1)]
        horizontalInferior = [(endPosition[0], col) for col in range(initialPosition[1], endPosition[1] + 1)]

        horizontal = set()
        horizontal.update(horizontalInferior + horizontalSuperior)
        vertical = set()
        vertical.update(verticalDireita + verticalEsquerda)
        
        Q = self.QTable

        verifyGlobal = True if globalPath != None else False
        verifyLocal = True if localPath != None else False
        for row in range(self.rows):
            print('')
            for col in range(self.cols):
                cell = self.cart2s((row, col, 0))
                if (verifyLocal) and (cell in localPath):
                        if np.argmax(qtableLocal[self.cart2s((row, col, 0)),:])==0:
                            print(' v"',end='')
                        elif np.argmax(qtableLocal[self.cart2s((row, col, 0)),:])==1:
                            print(' ^"',end='')
                        elif np.argmax(qtableLocal[self.cart2s((row, col, 0)),:])==2:
                            print(' >"',end='')
                        elif np.argmax(qtableLocal[self.cart2s((row, col, 0)),:])==3:
                            print(' <"',end='')
                        elif np.argmax(qtableLocal[self.cart2s((row, col, 0)),:])==4:
                            print(' \\"',end='')
                        elif np.argmax(qtableLocal[self.cart2s((row, col, 0)),:])==5:
                            print(' /"',end='')
                        elif np.argmax(qtableLocal[self.cart2s((row, col, 0)),:])==6:
                            print(' /"',end='')
                        elif np.argmax(qtableLocal[self.cart2s((row, col, 0)),:])==7:
                            print(' \\"',end='')

                elif (verifyGlobal) and (cell in globalPath):
                        if np.argmax(Q[self.cart2s((row, col, 0)),:])==0:
                            print(' v ',end='')
                        elif np.argmax(Q[self.cart2s((row, col, 0)),:])==1:
                            print(' ^ ',end='')
                        elif np.argmax(Q[self.cart2s((row, col, 0)),:])==2:
                            print(' > ',end='')
                        elif np.argmax(Q[self.cart2s((row, col, 0)),:])==3:
                            print(' < ',end='')
                        elif np.argmax(Q[self.cart2s((row, col, 0)),:])==4:
                            print(' \ ',end='')
                        elif np.argmax(Q[self.cart2s((row, col, 0)),:])==5:
                            print(' / ',end='')
                        elif np.argmax(Q[self.cart2s((row, col, 0)),:])==6:
                            print(' / ',end='')
                        elif np.argmax(Q[self.cart2s((row, col, 0)),:])==7:
                            print(' \ ',end='')
                elif cell in self.obstacles:
                    print(' O ', end='')

                elif (row, col) in horizontal:
                    print(' = ', end='')
                elif (row, col) in vertical:
                    print(' | ', end='')
                else:
                    print('   ', end='')
            if col == (self.cols - 1):
                print("|", end='')
        print('\n',self.cols*'---')

    def  demolish(self,states:list):
        #returns a list of obstacles that surrounds the list of states
        EitObs=[]
        for s in states:
            i,j,jk=self.s2cart(s)
            if self.cart2s((i+1,j,jk)) not in EitObs and self.is_onboard((i+1,j,jk)) and self.cart2s((i+1,j,jk)) not in self.obstacles:
                EitObs.append(self.cart2s((i+1,j,jk)))
            if self.cart2s((i+1,j+1,jk)) not in EitObs and self.is_onboard((i+1,j+1,jk)) and self.cart2s((i+1,j+1,jk)) not in self.obstacles:
                EitObs.append(self.cart2s((i+1,j+1,jk)))
            if self.cart2s((i+1,j-1,jk)) not in EitObs and self.is_onboard((i+1,j-1,jk)) and self.cart2s((i+1,j-1,jk)) not in self.obstacles:
                EitObs.append(self.cart2s((i+1,j-1,jk)))
            if self.cart2s((i,j,jk)) not in EitObs and self.is_onboard((i,j,jk)) and self.cart2s((i,j,jk)) not in self.obstacles:
                EitObs.append(self.cart2s((i,j,jk)))
            if self.cart2s((i,j+1,jk)) not in EitObs and self.is_onboard((i,j+1,jk)) and self.cart2s((i,j+1,jk)) not in self.obstacles:
                EitObs.append(self.cart2s((i,j+1,jk)))
            if self.cart2s((i,j-1,jk)) not in EitObs and self.is_onboard((i,j-1,jk)) and self.cart2s((i,j-1,jk)) not in self.obstacles:
                EitObs.append(self.cart2s((i,j-1,jk)))
            if self.cart2s((i-1,j,jk)) not in EitObs and self.is_onboard((i-1,j,jk)) and self.cart2s((i-1,j,jk)) not in self.obstacles:
                EitObs.append(self.cart2s((i+1,j,jk)))
            if self.cart2s((i-1,j+1,jk)) not in EitObs and self.is_onboard((i-1,j+1,jk)) and self.cart2s((i-1,j+1,jk)) not in self.obstacles:
                EitObs.append(self.cart2s((i-1,j+1,jk)))
            if self.cart2s((i-1,j-1,jk)) not in EitObs and self.is_onboard((i-1,j-1,jk)) and self.cart2s((i-1,j-1,jk)) not in self.obstacles:
                EitObs.append(self.cart2s((i-1,j-1,jk)))

        return EitObs