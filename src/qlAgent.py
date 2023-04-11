import numpy as np
from GridWorld import GridWorld
from dijkstra import Dijkstra
class qlAgent:
    def __init__(self, alpha=.1, gamma=.99, epsilon=.1,Double=0):
        self.alpha = alpha
        self.gamma = gamma
        self.epsilon = epsilon
        self.EXP = 2.7183
        self.Double=Double

    def setEnvironment(self, environment):
        self.environment = environment
        self.setPossibleStates()
        
    def setPossibleStates(self):
        self.states_ = np.arange(self.environment.rows * self.environment.cols)
        self.states_ = np.delete(self.states_, self.environment.obstacles, axis=0)
    
    def removeStates(self, states):
        self.states_ = np.setdiff1d(self.states_, states)
    
    def setEpsilon(self, mode = 0, intervals = [1, .1, 500]):
        '''
        @param: mode: 0, 1 or 2
        @param intervals: [max, min, totalEpisodes]
        '''
        if mode == 0:
            self.epsilonFunction = lambda episode: self.epsilon
        elif mode == 1:
            self.epsilonFunction = lambda episode: (max(intervals[0] - intervals[0]/intervals[2] * episode, intervals[1]))
        elif mode == 2:
            # y = ae^(-bx)
            a = intervals[0]
            b =  - np.log(intervals[1]) / intervals[2]
            self.epsilonFunction = lambda episode: (max(a * self.EXP**(-b * episode), intervals[1]))
        else:
            self.epsilonFunction = lambda episode: self.epsilon

    
    def setAlpha(self, mode = 0, intervals = [1, .1, 500]):
        '''
        @param: mode: 0, 1 or 2
        @param intervals: [max, min, totalEpisodes]
        '''
        if mode == 0:
            self.alphaFunction = lambda episode: self.alpha
        elif mode == 1:
            self.alphaFunction = lambda episode: (max(intervals[0] - intervals[0]/intervals[2] * episode, intervals[1]))
        elif mode == 2:
            # y = ae^(-bx)
            a = intervals[0]
            b =  - np.log(intervals[1]) / intervals[2]
            self.alphaFunction = lambda episode: (max(a * self.EXP**(-b * episode), intervals[1]))
        else:
            self.alphaFunction = lambda episode: self.alpha
    
    def setQtable(self, numTotalStates, numActions):
        '''
        Create q table
        '''
        self.Q = np.zeros((numTotalStates, numActions))
    
    def exploringStarts(self, rows, cols, origin):
        '''
        Set of possible states given the initial exploration constraints
        '''
        xo, yo, zo = origin
        x = np.arange(xo, xo+rows)
        y = np.arange(yo, yo+cols)
        totalStates = len(x) * len(y)
        self.states_ = np.zeros(totalStates, dtype=np.ushort)
        step = 0
        for row in x:
            for col in y:
                self.states_[step] = self.environment.cart2s((row, col, 0))
                step += 1
        self.removeStates(self.environment.obstacles)

        
    
    def chooseAction(self, state, episode):
        '''
        chooses a action-state based on possible action-states
        '''
        
        if np.random.rand() < self.epsilonFunction(episode):
            
                action = np.random.choice(self.environment.actions[state])
                return action
          
        else:
            action = np.argmax(self.Q[state,:])
            return action
    
    def chooseInitialState(self):
        '''
        chooses a random initial state based on possible states
        '''
        initialState = np.random.choice(self.states_)
        self.updateEnvironment(initialState)
        return initialState


    def updateAction(self, action):
        '''
        Update action in grid worldmini_grid.updateObstacles(1)
        '''
        self.environment.current_action = action

    
    def move(self, action):
        '''
        Move agent in grid world
        '''
        newState, reward, done = self.environment.step(action)
        return newState, reward, done
    
    def updateEnvironment(self, state):
        '''
        Update position of the agent in grid world
        '''
        x, y, z = self.environment.s2cart(state)
        self.environment.i = x
        self.environment.j = y
        self.environment.k = z
        self.environment.last_action = self.environment.current_action


    def updateQTable(self, state, action, reward, newState,done, episode):
        '''
        Update q-table
        '''
        self.Q[state, action] = (1 - self.alphaFunction(episode)) * self.Q[state, action] + self.alphaFunction(episode) * (reward + (1-done)*self.gamma * max(self.Q[newState, :]))
    
    def updateQTableTransfer(self,Q:np.ndarray,thresh):
        #updates Q table using a new Q table as input using TLusing tresh as treshold level
        if self.Double==0:
            for s in range(0,Q.shape[0]):
                for a in range(0,Q.shape[1]):
                    # Validate Treshold
                    limit=np.max(Q)*thresh
                    if Q[s][a]>=limit:
                        self.Q[s][a]=Q[s][a]*thresh
                    else:
                        self.Q[s,a]=0
        else:
            pass 
            #print('This function only works for regular Q-Learning, the Double-QL flag is set to 1')


    def reset(self,start=0):
        '''
        reset environment
        '''
        self.environment.reset(start)
    
    def plot(self, scorePerEpisode, stepsPerEpisode, TDerrorPerEpisode):
        '''
        this method plots to statistics during training, like: score per episode, steps per episode and 50 periods moving average
        '''
        import matplotlib.pyplot as plt

        moving_average = lambda data, periods: np.convolve(data, np.ones(periods), 'valid') / periods
    

        fig, (ax1, ax2, ax3) = plt.subplots(3, 1, sharex=True, figsize=(12, 8))
        ax1.plot(moving_average(TDerrorPerEpisode, 50))
        ax1.set_title("TD error per episode")
        ax2.plot(moving_average(stepsPerEpisode, 1))
        ax2.set_title("Steps per episode")
        ax3.plot(moving_average(scorePerEpisode, 50))
        ax3.set_title("Reward per episode")
        fig.tight_layout()
        ax1.grid()
        ax2.grid()
        ax3.grid()
        #plt.draw()
        plt.show()


    def train(self, numEpisodes, frequency, plot = False):
        '''
        This method train the agent
        '''
        scorePerEpisode = np.zeros(numEpisodes)
        stepsPerEpisode = np.zeros(numEpisodes)
        TDerrorPerEpisode = np.zeros(numEpisodes)
        #startfrom=(self.environment.i, self.environment.j, self.environment.k)
        #print(self.environment.i, self.environment.j, self.environment.k)
        for episode in range(numEpisodes):
            step = 0
            score = 0
            done = False
            TDerror = 0
            
            self.reset()
            #print(self.environment.cart2s((self.environment.i, self.environment.j, self.environment.k)))
            
            if episode % frequency == 0:
                self.chooseInitialState()
                print('Episode:', episode, end='\r')
            while done == False:
                           
                state = self.environment.cart2s((self.environment.i, self.environment.j, self.environment.k))
                #print('\n',state) 
                action = self.chooseAction(state, episode)
                self.updateAction(action)
                newState, reward, done = self.move(action)
                newState = self.environment.cart2s(newState)
                TDerror += self.getTDerror(state, action, reward, newState, done, plot)
                self.updateQTable(state, action, reward, newState, done, episode)
                state = newState
                self.updateEnvironment(state)
                step += 1
                score += reward
            scorePerEpisode[episode] = score
            stepsPerEpisode[episode] = step
            TDerrorPerEpisode[episode] = TDerror
        if plot == True:
            self.plot(scorePerEpisode, stepsPerEpisode, TDerrorPerEpisode)
    def FindPathsLocal(self,goals,InitPos,Q=None,TL=0,NumEp=300,freq=10,plot=0):
        #returns paths for each of the goals 
        #Assumes that obstacles and minigrid have been updated accordingly
        Paths=[]
        self.environment.i,self.environment.j,self.environment.k=InitPos
        for goal in goals:
            #print(goal)
            #print(self.environment.i,self.environment.j,self.environment.k)
            self.environment.setGoal(goal)
            self.environment.get_reward_safety()
            self.setQtable(self.environment.rows*self.environment.cols,8)
            if TL:
                self.updateQTableTransfer(Q,0.6)
            self.train(NumEp,freq)
            path=self.getBestPath(InitPos)
            Paths.append(path)
            
            if plot:
                self.environment.PrintBestPath(self.Q,0,path)
        return Paths
    def getTDerror(self, state, action, reward, newState, done, plot):
        if plot == False:
            return 0
        return reward + (1 - done) * self.gamma * max(self.Q[newState, :]) - self.Q[state, action]
    def getQtable(self)->np.array:
        '''
        Return q table
        '''
        return self.Q
    
    def getBestPath(self, startPos):
        path = self.environment.getBestPath(startPos, self.Q)
        path.insert(0, self.environment.cart2s(startPos))
        return path
    
    def getDijkstraPath(self, startPos):
        grafo = self.environment.getGrafo()
        dijkstra = Dijkstra(grafo, self.environment.cart2s(startPos))
        distancias, bestPath, pais = dijkstra.run()
        dijkstraPath = dijkstra.getPath(self.environment.cart2s(self.environment.goal))
        return dijkstraPath
    
    def getStats(self, startPos):
        print(f'Fail rate: {self.environment.testConvergence(self.Q)}%' )
        print('\nQ learning stats:')
        print('Lenght: %.2f' %self.environment.get_distance(self.getBestPath(startPos)))
        print('Mean Distance: %.2f ' % self.environment.get_meanDist(self.getBestPath(startPos)))


        print('\nDijkstra stats:')
        print('Lenght: %.2f' % self.environment.get_distance(self.getDijkstraPath(startPos)))
        print('Mean Distance: %.2f' % self.environment.get_meanDist(self.getDijkstraPath(startPos)))
    def chooseBestAction(self, state):
        return np.argmax(self.Q[state,:])
    def getRank(self, path):
        distanceRank = self.getRankDist(path)
        energyRank = self.getRankEnergy(path)
        proximityRank = self.getRankObstaclesProximity(path)
        return distanceRank, energyRank, proximityRank

    def getRankDist(self, path):
        xo, yo, zo = self.environment.s2cart(path[0])
        xf, yf, zf = self.environment.s2cart(path[-1])
        lengthPath = self.environment.get_distance(path)
        euclideanDistance = np.linalg.norm([xf - xo, yf - yo])
        return euclideanDistance/lengthPath

    def getRankEnergy(self, path):
        last_action = self.chooseBestAction(path[0])
        current_action = 0
        energyCost = 0
        for index_state in range(1, len(path) - 1):
            current_action = self.chooseBestAction(path[index_state])
            energyCost += self.getEnergyCost(last_action, current_action)
            last_action = current_action
        return energyCost
    
    def getRankObstaclesProximity(self, path):
        return self.environment.get_meanDist(path)
    
    def getEnergyCost(self, last_action, current_action):
        energyCost = 0
        if last_action != current_action:
            try:
                energyCost += self.environment.energyCost[(last_action, current_action)]
            except:
                energyCost += self.environment.energyCost[(current_action, last_action)]
        return energyCost

    def FindInPolicy(self, position : tuple, size : int = 1) -> dict:
        x, y, z = position
        x_possibles = np.arange(x - size, x + size + 1)
        y_possibles = np.arange(y - size, y + size + 1)
        z_possibles = np.arange(z - size, z + size + 1)
        possibles_states = []
        for x in x_possibles:
            for y in y_possibles:
                if x >= 0 and y >=0:
                    possibles_states.append(self.environment.cart2s((x, y, 0)))

        for state in possibles_states:
            if state in self.environment.obstacles:
                possibles_states = np.setdiff1d(possibles_states, state)

        possibles_states = np.setdiff1d(possibles_states, self.environment.cart2s(position))

        states = []
        step = 0
        for state in possibles_states:
            current_path = self.__OneStep(state)
            if current_path != []:
                states.append(current_path)
        return self.__rankPath(states)
        
    def __OneStep(self, initialState: int) -> list:
        maxsteps = 100
        step = 0
        state = initialState
        path = [initialState]
        last_state = 66
        done = False
        self.environment.reset()
        while done == False and last_state != state:
            last_state = state
            self.updateEnvironment(state)
            action = self.chooseBestAction(state)
            self.updateAction(action)
            newState, reward, done = self.move(action)
            newState = self.environment.cart2s(newState)
            state = newState
            self.updateEnvironment(state)
            step += 1
            if last_state != newState:
                path.append(state)
            else:
                return []
        return path

    def __rankPath(self, paths : list) -> dict:
        rank_path = dict()
        for idx, path in enumerate(paths):
            rank_path[idx] = {'path': path,
                              'rank': self.getRank(path)}
        return rank_path