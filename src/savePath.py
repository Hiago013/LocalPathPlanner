
from sys import platform

class savePath():
    def __init__(self, dijkstraPath, QLearningPath):
        self.dijkstraPath = dijkstraPath
        self.QLearningPath = QLearningPath
        
    def save(self, name):
        test=platform
        if test=='linux':
            with open("Paths//" + name + 'Dijkstra' + '.csv', 'w') as file:
                for i in range(len(self.dijkstraPath)):
                    if i == len(self.dijkstraPath) - 1:
                        file.write(str(self.dijkstraPath[i]))
                    else:
                        file.write(str(self.dijkstraPath[i]) + ',')
            file.close()
            with open("Paths//" + name + 'QLearning' + '.csv', 'w') as file:
                for i in range(len(self.QLearningPath)):
                    if i == len(self.QLearningPath) - 1:
                        file.write(str(self.QLearningPath[i]))
                    else:
                        file.write(str(self.QLearningPath[i]) + ',')
            file.close()
        else:
            with open("Paths\\" + name + 'Dijkstra' + '.csv', 'w') as file:
                for i in range(len(self.dijkstraPath)):
                    if i == len(self.dijkstraPath) - 1:
                        file.write(str(self.dijkstraPath[i]))
                    else:
                        file.write(str(self.dijkstraPath[i]) + ',')
            file.close()
            with open("Paths\\" + name + 'QLearning' + '.csv', 'w') as file:
                for i in range(len(self.QLearningPath)):
                    if i == len(self.QLearningPath) - 1:
                        file.write(str(self.QLearningPath[i]))
                    else:
                        file.write(str(self.QLearningPath[i]) + ',')
            file.close()
    
    def saveAlternativePath(self, name, path):
        numPaths = len(path)
        test=platform
        if test=='linux':
            with open("Paths//" + name + 'Alternative' + '.csv', 'w') as file:
                for route in range(numPaths):
                    for j in range(len(path[route])):
                        if j == len(path[route]) - 1:
                            file.write(str(path[route][j]))
                        else:
                            file.write(str(path[route][j]) + ',')
                    file.write('\n')
            file.close()
        else:
            with open("Paths\\" + name + 'Alternative' + '.csv', 'w') as file:
                for route in range(numPaths):
                    for j in range(len(path[route])):
                        if j == len(path[route]) - 1:
                            file.write(str(path[route][j]))
                        else:
                            file.write(str(path[route][j]) + ',')
                    file.write('\n')
            file.close()
            
