import math
from Nodo import Nodo
from queue import Queue

class Grafo:

    def __init__(self, directed=False):
        self.lnodos = []
        self.directed = directed
        self.grafo = {}
        self.start = None

    def __str__(self):
        out = ""
        for key in self.grafo.keys():
            out = out + str(key) + ": " + str(self.grafo[key]) + "\n"
        return out

    # Adiciona arco
    def addEdge(self, node1,x1,y1, node2,x2,y2):
        n1 = Nodo(node1,x1,y1)
        n2 = Nodo(node2,x2,y2)
        peso = 1

        if node2 == "X":
            peso = 25

        if n1 not in self.lnodos:
            self.lnodos.append(n1)
            self.grafo[n1] = set()

        if n2 not in self.lnodos:
            self.lnodos.append(n2)
            self.grafo[n2] = set()

        self.grafo[n1].add((n2,peso))

        if not self.directed:
            if node1 == "X":
                peso = 25
            else:peso = 1
            self.grafo[n2].add((n1, peso))

    # Localiza o nodo inicial
    def getStart(self):
        for start in self.lnodos:
            if start.type == "P":
                return start
        return None

    # Retorna o custo do arco
    def getArcCost(self, node1, node2):
        custoT = math.inf
        a = self.grafo[node1]   
        for (node, cost) in a:
            if node == node2:
                custoT = cost

        return custoT

    # Calcula o custo do caminho encontrado
    def calculateCost(self, path):
        teste = path 
        cost = 0
        i = 0
        while i + 1 < len(teste):
             cost = cost + self.getArcCost(teste[i], teste[i+1])
             i = i + 1
        return cost    

    # Algoritmo de procura BFS
    def BFSSearch(self, start, end):
        q = Queue()
        visited = set()

        q.put(start)
        visited.add(start)
        p = {}
        p[start] = None

        pathFound = False
        while not q.empty() and pathFound == False:
            n = q.get()
            if n.gettype() == end:
                pathFound = True
            else:
                for (adjacent, cost) in self.grafo[n]:
                    if adjacent not in visited:
                        visited.add(adjacent)
                        q.put(adjacent)
                        p[adjacent] = n
                    if adjacent.gettype() == end:
                        endNode = adjacent
                        pathFound = True
        path = []
        if pathFound == True:
            path.append(endNode)
            while p[endNode] is not None:
                path.append(p[endNode])
                endNode = p[endNode]
            path.reverse()
            custoT = self.calculateCost(path)
            return (path, custoT)
