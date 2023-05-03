class OpenInstance:
    def __init__(self, instanceName):
        self.instanceName = instanceName

    
    def run(self):
        maps = open(self.instanceName, 'r')
        self.header = maps.readline().rstrip('\n').split(' ')
        self.header = tuple([int(num) for num in self.header])
        self.numObstacle = int(maps.readline().rstrip('\n'))
        self.walls = []
        line = 'aux'
        while line != '':
            line = maps.readline().rstrip('\n')
            try:
                self.walls.append(int(line))
            except:
                pass
        maps.close()

        return(self.header, self.numObstacle, list(self.walls))

