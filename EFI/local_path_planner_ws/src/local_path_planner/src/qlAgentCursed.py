import numpy as np
from dijkstra import *


class qlAgent:
    def __init__(self, alpha=.1, gamma=.99, epsilon=.1, double = False):
        self.alpha = alpha
        self.gamma = gamma
        self.epsilon = epsilon
        self.EXP = 2.7183
        self.double = double
        if double == True:
            self.__currentQTable = 1
            self.__best_action = 0
        self.__first_action = None # The first action that was taken before the episode start
        

    def setEnviroment(self, enviroment):
        self.enviroment = enviroment
        self.__first_action = self.enviroment.last_action
        self.setPossibleStates()
    
    def setPossibleStates(self):
        self.states_ = np.arange(self.enviroment.rows * self.enviroment.cols)
        self.states_ = np.delete(self.states_, self.enviroment.obstacles, axis=0)
    
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
            self.epsilonFunction = lambda episode: max(intervals[0] - intervals[0]/intervals[2] * episode, intervals[1])
        elif mode == 2:
            # y = ae^(-bx)
            a = intervals[0]
            b =  - np.log(intervals[1]) / intervals[2]
            self.epsilonFunction = lambda episode: max(a * self.EXP**(-b * episode), intervals[1])
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
            self.alphaFunction = lambda episode: max(intervals[0] - intervals[0]/intervals[2] * episode, intervals[1])
        elif mode == 2:
            # y = ae^(-bx)
            a = intervals[0]
            b =  - np.log(intervals[1]) / intervals[2]
            self.alphaFunction = lambda episode: max(a * self.EXP**(-b * episode), intervals[1])
        else:
            self.alphaFunction = lambda episode: self.alpha
    
    def setQtable(self, numTotalStates, numActions):
        '''
        Create q table
        '''
        if self.double == False:
            self.Q = np.zeros((numTotalStates, numActions))
        else:
            self.Q1 = np.zeros((numTotalStates, numActions))
            self.Q2 = np.zeros((numTotalStates, numActions))
    
    def exploringStarts(self, rows, cols, origin):
        '''
        Set of possible states given the initial exploration constraints
        '''
        io, jo, ko = origin
        i = np.arange(io, io+rows)
        j = np.arange(jo, jo+cols)
        totalStates = len(i) * len(j)
        self.states_ = np.zeros(totalStates, dtype=np.ushort)
        step = 0
        for row in i:
            for col in j:
                self.states_[step] = self.enviroment.cart2s((row, col, 0))
                step += 1
        self.removeStates(self.enviroment.obstacles)

        
    
    def chooseAction(self, state, episode):
        '''
        chooses a action-state based on possible action-states
        '''
        if self.double == True:
            self.Q = self.Q1 + self.Q2

        if np.random.rand() < self.epsilonFunction(episode):
            action = np.random.choice(self.enviroment.actions[state])
            return action
        else:
            action = np.argmax(self.Q[state,:])
            return action
    
    def chooseBestAction(self, state):
        if self.double == True:
            self.Q = self.Q1 + self.Q2
        return np.argmax(self.Q[state,:])
    
    def chooseInitialState(self):
        '''
        chooses a random initial state based on possible states
        '''
        initialState = np.random.choice(self.states_)
        self.updateEnviroment(initialState)
        return initialState


    def updateAction(self, action):
        '''
        Update action in grid world
        '''
        self.enviroment.current_action = action

    
    def move(self, action):
        '''
        Move agent in grid world
        '''
        newState, reward, done = self.enviroment.step(action)
        return newState, reward, done
    
    def updateEnviroment(self, state):
        '''
        Update position of the agent in grid world
        '''
        i, j, k = self.enviroment.s2cart(state)
        self.enviroment.i = i
        self.enviroment.j = j
        self.enviroment.k = k
        self.enviroment.last_action = self.enviroment.current_action


    def updateQTable(self, state, action, reward, newState,done, episode):
        '''
        Update q-table
        '''
        self.Q[state, action] = (1 - self.alphaFunction(episode)) * self.Q[state, action] + self.alphaFunction(episode) * (reward + (1-done)*self.gamma * max(self.Q[newState, :]))
    
    def updateDQtable(self, state, action, reward, newState, done, episode):

        if np.random.rand() <= .5:
            self.__currentQTable = 1
            self.__best_action = np.argmax(self.Q1[newState, :])
            self.Q1[state, action] = (1 - self.alphaFunction(episode)) * self.Q1[state, action] + self.alphaFunction(episode) * (reward + (1-done)*self.gamma * self.Q2[newState, self.__best_action])
        else:
            self.__currentQTable = 2
            self.__best_action = np.argmax(self.Q2[state, :])
            self.Q2[state, action] = (1 - self.alphaFunction(episode)) * self.Q2[state, action] + self.alphaFunction(episode) * (reward + (1-done)*self.gamma * self.Q1[newState, self.__best_action])



    def reset(self,start=0):
        '''
        reset enviroment
        '''
        self.enviroment.reset(start)
    
    def plot(self, scorePerEpisode, stepsPerEpisode, TDerrorPerEpisode):
        '''
        this method plots to statistics during training, like: score per episode, steps per episode and 50 periods moving average
        '''
        import matplotlib.pyplot as plt

        moving_average = lambda data, periods: np.convolve(data, np.ones(periods), 'valid') / periods
    

        fig, (ax1, ax2, ax3) = plt.subplots(3, 1, sharex=True, figsize=(12, 8))
        ax1.plot(moving_average(TDerrorPerEpisode, 50))
        ax1.set_title("TD error per episode")
        ax2.plot(moving_average(stepsPerEpisode, 50))
        ax2.set_title("Steps per episode")
        ax3.plot(moving_average(scorePerEpisode, 50))
        ax3.set_title("Reward per episode")
        fig.tight_layout()
        ax1.grid()
        ax2.grid()
        ax3.grid()
        plt.show()



    def train(self, numEpisodes, frequency, plot = False):
        '''
        This method train the agent
        '''
        if self.double == 0:
            update = self.updateQTable
        else:
            update = self.updateDQtable
        scorePerEpisode = np.zeros(numEpisodes)
        stepsPerEpisode = np.zeros(numEpisodes)
        TDerrorPerEpisode = np.zeros(numEpisodes)
        for episode in range(numEpisodes):
            step = 0
            score = 0
            done = False
            TDerror = 0
            self.enviroment.last_action = self.__first_action
            self.reset()
            if episode % frequency == 0:
                self.chooseInitialState()
            while done == False:
                state = self.enviroment.cart2s((self.enviroment.i, self.enviroment.j, self.enviroment.k))
                try:
                    action = self.chooseAction(state, episode)
                except:
                   # print('Estado:',state, 'Dict:',self.enviroment.actions[state],'Posição do Agente:',self.enviroment.i  ,self.enviroment.j,'Lista',self.states_, end='\r')
                    #input('Vai')
                    pass
                self.updateAction(action)
                newState, reward, done = self.move(action)
                newState = self.enviroment.cart2s(newState)
                TDerror += self.__getTDerror(state, action, reward, newState, done, plot)
                update(state, action, reward, newState, done, episode)
                state = newState
                self.updateEnviroment(state)
                step += 1
                score += reward
            scorePerEpisode[episode] = score
            stepsPerEpisode[episode] = step
            TDerrorPerEpisode[episode] = TDerror
        if self.double == True:
            self.Q = self.Q1 + self.Q2
        if plot == True:
            self.plot(scorePerEpisode, stepsPerEpisode, TDerrorPerEpisode)
    
    def __getTDerror(self, state, action, reward, newState, done, plot):
        if plot == False:
            return 0
        if self.double == True:
            if self.__currentQTable == 1:
                return reward + (1 - done) * self.gamma * self.Q2[newState, self.__best_action] - self.Q1[state, action]
            else:
                return reward + (1 - done) * self.gamma * self.Q1[newState, self.__best_action] - self.Q2[state, action]

        return reward + (1 - done) * self.gamma * max(self.Q[newState, :]) - self.Q[state, action]

    def getQtable(self):
        '''
        Return q table
        '''
        return self.Q
    
    def getBestPath(self, startPos):
        path = self.enviroment.getBestPath(startPos, self.Q)
        path.insert(0, self.enviroment.cart2s(startPos))
        return path
    
    def getDijkstraPath(self, startPos):
        grafo = self.enviroment.getGrafo()
        dijkstra = Dijkstra(grafo, self.enviroment.cart2s(startPos))
        distancias, bestPath, pais = dijkstra.run()
        dijkstraPath = dijkstra.getPath(self.enviroment.cart2s(self.enviroment.goal))
        return dijkstraPath

    def getStats(self, startPos):
        print(f'Fail rate: {self.enviroment.testConvergence(self.Q)}%' )
        print('\nQ learning stats:')
        print('Lenght: %.2f' %self.enviroment.get_distance(self.getBestPath(startPos)))
        print('Mean Distance: %.2f ' % self.enviroment.get_meanDist(self.getBestPath(startPos)))


        print('\nDijkstra stats:')
        print('Lenght: %.2f' % self.enviroment.get_distance(self.getDijkstraPath(startPos)))
        print('Mean Distance: %.2f' % self.enviroment.get_meanDist(self.getDijkstraPath(startPos)))
    
    def getRank(self, path):
        distanceRank = self.getRankDist(path)
        energyRank = self.getRankEnergy(path)
        proximityRank = self.getRankObstaclesProximity(path)
        return distanceRank, energyRank, proximityRank

    def getRankDist(self, path):
        io, jo, ko = self.enviroment.s2cart(path[0])
        ie, je, ke = self.enviroment.s2cart(path[-1])
        lengthPath = self.enviroment.get_distance(path)
        euclideanDistance = np.linalg.norm([ie - io, je - jo])
        return euclideanDistance/lengthPath

    def getRankEnergy(self, path):
        last_action = self.chooseBestAction(path[0])
        current_action = 0
        energyCost = 0
        for index_state in range(1, len(path) - 1):
            current_action = self.chooseBestAction(path[index_state])
            energyCost += self.__getEnergyCost(last_action, current_action)
            last_action = current_action
        return energyCost
    
    def getRankObstaclesProximity(self, path):
        return self.enviroment.get_meanDist(path)
    
    def __getEnergyCost(self, last_action, current_action):
        energyCost = 0
        if last_action != current_action:
            try:
                energyCost += self.enviroment.energyCost[(last_action, current_action)]
            except:
                energyCost += self.enviroment.energyCost[(current_action, last_action)]
        return energyCost
    
    def __OneStep(self, initialState: int) -> list:
        maxsteps = 100
        step = 0
        state = initialState
        path = [initialState]
        last_state = 66
        done = False
        self.enviroment.reset()
        while done == False and last_state != state:
            last_state = state
            self.updateEnviroment(state)
            action = self.chooseBestAction(state)
            self.updateAction(action)
            newState, reward, done = self.move(action)
            newState = self.enviroment.cart2s(newState)
            state = newState
            self.updateEnviroment(state)
            step += 1
            if last_state != newState:
                path.append(state)
            else:
                return []
        return path

    def FindInPolicy(self, position : tuple, size : int = 1) -> dict:
        i, j, k = position
        i_possibles = np.arange(i - size, i + size + 1)
        j_possibles = np.arange(j - size, j + size + 1)
        k_possibles = np.arange(k - size, k + size + 1)
        possibles_states = []
        for i in i_possibles:
            for j in j_possibles:
                if i >= 0 and j >=0:
                    possibles_states.append(self.enviroment.cart2s((i, j, 0)))

        for state in possibles_states:
            if state in self.enviroment.obstacles:
                possibles_states = np.setdiff1d(possibles_states, state)

        possibles_states = np.setdiff1d(possibles_states, self.enviroment.cart2s(position))

        states = []
        step = 0
        for state in possibles_states:
            if np.array(self.__OneStep(state)) != []:
                states.append(np.array(self.__OneStep(state)))
        return self.__rankPath(states)

    def __rankPath(self, paths : list) -> dict:
        rank_path = dict()
        for i, path in enumerate(paths):
            rank_path[i] = {path: self.getRank(path)}
