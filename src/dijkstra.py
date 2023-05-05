#criar uma função que recebe o grafo e o nó inicial e retorna o caminho mais curto

class Dijkstra:
    def __init__(self, grafo, inicial):
        self.distancias = {}
        self.pais = {}
        self.bestPath = {}
        self.grafo = grafo
        self.inicial = inicial

    def run(self):
        '''
        This method runs the Dijkstra algorithm and returns the best path from the initial node to all other nodes.
        '''
        distancias = self.distancias
        pais = self.pais
        bestPath = self.bestPath
        grafo = self.grafo
        inicial = self.inicial
        for no in grafo:
            distancias[no] = float('inf')
            pais[no] = None
            bestPath[no] = []
        distancias[inicial] = 0
        fila = [inicial]
        while len(fila) > 0:
            atual = fila.pop(0)
            for vizinho in grafo[atual]:
                if distancias[vizinho] == float('inf'):
                    distancias[vizinho] = distancias[atual] + grafo[atual][vizinho]
                    pais[vizinho] = atual
                    fila.append(vizinho)
        for no in grafo:
            atual = no
            while pais[atual] != None:
                bestPath[no].append(atual)
                atual = pais[atual]
            bestPath[no].append(atual)
        return distancias, bestPath, pais
    
    def getPath(self, no):
        '''
        This method returns the best path from the initial node to the given node.
        '''
        return self.bestPath[no][::-1]

# grafo = {'A': {'B': 5, 'C': 1},
#             'B': {'A': 5, 'C': 2, 'D': 1},
#             'C': {'A': 1, 'B': 2, 'D': 4, 'E': 2},
#             'D': {'B': 1, 'C': 4, 'E': 3, 'F': 1},
#             'E': {'C': 2, 'D': 3, 'F': 5, 'G': 1},
#             'F': {'D': 1, 'E': 5, 'G': 4, 'H': 5},
#             'G': {'E': 1, 'F': 4, 'H': 3},
#             'H': {'F': 5, 'G': 3}}


# distancias, bestPath, pais = dijkstra(grafo, 'A')
# #print(distancias)
# #print(pais)
# print(bestPath['H'])
# #print(distancias['D'])