import math
from Nodo import Nodo
from queue import Queue

class Grafo:

    def __init__(self, directed=False):
        self.lnodos = []
        self.directed = directed
        self.grafo = {}
        self.start = "P"
        self.end = "F"

    def __str__(self):
        out = "\033[34mGrafo\033[m\n"
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
    def getStart(self, node):
        for start in self.lnodos:
            if start.type == node:
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

    # Algoritmo de procura DFS
    def DFSSearch(self, start, end, path=[], visited=set()):
        path.append(start)
        visited.add(start)

        if start.gettype() == end:
            custoT = self.calculateCost(path)
            return (path, custoT)
        if start.gettype() != 'X':
            for (adjacente, peso) in self.grafo[start]:
                if adjacente not in visited:
                    resultado = self.DFSSearch(adjacente, end, path, visited)
                    if resultado is not None:
                        return resultado
        path.pop()  
        return None

    # Algoritmo de procura BFS
    def BFSSearch(self, start, end):
        q = Queue()
        visited = set()

        q.put(start)
        visited.add(start)
        p = dict()
        p[start] = None

        pathFound = False
        while not q.empty() and pathFound == False:
            n = q.get()
            if n.gettype() == end:
                pathFound = True
            else:
                if n.gettype() != 'X':
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

    def Uniform(self, start, end):
        q = list()
        q.append((start,0))

        visited = list()
        visited.append(start)
        p = {}
        p[start] = None

        pathFound = False
        while len(q)>0 and pathFound == False:
            q.sort(key=lambda x: x[1])
            n = q.pop(0)
            if n[0].gettype() == end:
                pathFound = True
            else:
                if n[0].gettype() != 'X':
                    for (adjacent, cost) in self.grafo[n[0]]:
                        if adjacent not in visited:
                            visited.append(adjacent)
                            q.append((adjacent,n[1]+self.getArcCost(n[0],adjacent)))
                            p[adjacent] = n[0]
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

    # Criar ligações entre nodos
    def addEdges(self, maze):
        j = 0
        for line in maze:
            i = 0
            for char in line[:-1]:
                if i < len(line) - 2:
                    if not (char == "X" and maze[j][i+1] == "X"):
                        self.addEdge(maze[j][i], j, i, maze[j][i+1], j,i+1)
                if j < len(maze) - 1:
                    if i < len(line) - 2:
                        if not (char == "X" and maze[j+1][i + 1] == "X"):
                            self.addEdge(maze[j][i], j, i, maze[j+1][i + 1], j + 1, i+1)
                    if not (char == "X" and maze[j+1][i] == "X"):
                        self.addEdge(maze[j][i], j, i, maze[j+1][i], j+1, i)
                    if i>0 :
                        if not (char == "X" and maze[j + 1][i-1] == "X"):
                            self.addEdge(maze[j][i], j, i, maze[j + 1][i-1], j+1, i - 1)
                i = i + 1
            j = j + 1
