import math
from Nodo import Nodo
from queue import Queue
from Carro import Carro

class Grafo:

    def __init__(self, directed=False):
        self.lnodos = []
        self.directed = directed
        self.grafo = {}
        self.heuristicas = {}
        self.start = "P"
        self.end = "F"
        self.carros = list()

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

    def getPrimCarro(self):
        return self.carros[0]
    def setCarros(self):
        l = list()
        i=0
        for start in self.lnodos:
            if start.type == self.start:
                i+=1
                c=Carro(i,start.getX(),start.getY())
                l.append(c)
        self.carros=l


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

    def add_heuristica(self, n, heuristca):
        n1 = Nodo(n)
        if n1 in self.m_nodes:
            self.heuristicas[n] = heuristca

    def manhattan_dist(self,c1, c2):
        return math.fabs(c1[0] - c2[0]) + math.fabs(c1[1] - c2[1])

    def get_end_coords_list(self):
        end_nodes = []
        for c in self.lnodos:
            if (c.gettype() == "F"):
                end_nodes.append(c.getCord())
        return end_nodes

    def calculate_heuristic(self):
        end_coords_list = self.get_end_coords_list()
        for node in self.lnodos:
            node_c = node.getCord()
            for end_c in end_coords_list:
                dist = self.manhattan_dist(node_c, end_c)
                if node not in self.heuristicas:
                    self.heuristicas[node] = dist
                elif self.heuristicas[node] > dist:
                    self.heuristicas[node] = dist

    def getNeighbours(self, nodo):
        lista = []
        for (adjacente, peso) in self.grafo[nodo]:
            lista.append((adjacente, peso))
        return lista

    def addEdges(self, maze):
        j = 0
        for line in maze:
            i = 0
            for char in line[:-1]:
                if i < len(line) - 2:
                    if not (char == "X" and maze[j][i + 1] == "X"):
                        self.addEdge(maze[j][i], j, i, maze[j][i + 1], j, i + 1)
                if j < len(maze) - 1:
                    if i < len(line) - 2:
                        if not (char == "X" and maze[j + 1][i + 1] == "X"):
                            self.addEdge(maze[j][i], j, i, maze[j + 1][i + 1], j + 1, i + 1)
                    if not (char == "X" and maze[j + 1][i] == "X"):
                        self.addEdge(maze[j][i], j, i, maze[j + 1][i], j + 1, i)
                    if i > 0:
                        if not (char == "X" and maze[j + 1][i - 1] == "X"):
                            self.addEdge(maze[j][i], j, i, maze[j + 1][i - 1], j + 1, i - 1)
                i = i + 1
            j = j + 1

    # Algoritmo de procura DFS
    def DFSSearch(self, start, end, path=[],expansao=list(), visited=set()):
        path.append(start)
        expansao.append(start)
        visited.add(start)

        if start.gettype() == end:
            expansao.pop()
            custoT = self.calculateCost(path)
            return (expansao,(path, custoT))
        if start.gettype() != 'X':
            for (adjacente, peso) in self.grafo[start]:
                if adjacente not in visited:
                    resultado = self.DFSSearch(adjacente, end, path, expansao,visited)
                    if resultado is not None:
                        return resultado
        path.pop()  
        return None

    # Algoritmo de procura BFS
    def BFSSearch(self, start, end):
        q = Queue()
        visited = set()
        expansao = list()

        q.put(start)
        visited.add(start)
        p = dict()
        p[start] = None

        pathFound = False
        while not q.empty() and pathFound == False:
            n = q.get()
            expansao.append(n)
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
            return (expansao,(path, custoT))

    def Uniform(self, start, end):
        q = list()
        q.append((start,0))

        visited = list()
        visited.append(start)
        expansao = list()
        p = {}
        p[start] = None

        pathFound = False
        while len(q)>0 and pathFound == False:
            q.sort(key=lambda x: x[1])
            n = q.pop(0)
            expansao.append(n[0])
            if n[0].gettype() == end:
                expansao.pop()
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
            return (expansao,(path, custoT))

    def greedy(self, start, end_name):
        # open_list é uma lista de nodos visitados, mas com vizinhos
        # que ainda não foram todos visitados, começa com o  start
        # closed_list é uma lista de nodos visitados
        # e todos os seus vizinhos também já o foram
        open_list = set()
        closed_list = set()

        open_list.add(start)

        # parents é um dicionário que mantém o antecessor de um nodo
        # começa com start
        parents = dict()
        parents[start] = start

        while len(open_list) > 0:
            n = None

            # encontrado nodo com a menor heuristica
            for v in open_list:
                if n is None or self.heuristicas[v] < self.heuristicas[n]:
                    n = v

            if n is None:
                print('Path does not exist!')
                return None

            # se o nodo corrente é o destino
            # reconstruir o caminho a partir desse nodo até ao start
            # seguindo o antecessor
            if n.gettype() == end_name:
                reconst_path = []

                while parents[n] != n:
                    reconst_path.append(n)
                    n = parents[n]

                reconst_path.append(start)

                reconst_path.reverse()

                return (reconst_path, self.calculateCost(reconst_path))

            # para todos os vizinhos  do nodo corrente
            for (m, weight) in self.getNeighbours(n):
                # Se o nodo corrente nao esta na open nem na closed list
                # adiciona-lo à open_list e marcar o antecessor
                if m not in open_list and m not in closed_list:
                    open_list.add(m)
                    parents[m] = n

            # remover n da open_list e adiciona-lo à closed_list
            # porque todos os seus vizinhos foram inspecionados
            open_list.remove(n)
            closed_list.add(n)

        print('Path does not exist!')
        return None

    def AStar(self, start, end):
        # open_list é uma lista de nodos visitados, mas com vizinhos
        # que ainda não foram todos visitados, começa com o  start
        # closed_list é uma lista de nodos visitados
        # e todos os seus vizinhos também já o foram
        open_list = set()
        closed_list = set()

        open_list.add(start)

        # parents é um dicionário que mantém o antecessor de um nodo
        # começa com start
        parents = dict()
        parents[start] = start

        # cost é um dicionário que mantém o custo do caminho desde o ínicio do grafo até a um nodo
        # começa com 0
        cost = dict()
        cost[start] = 0

        while len(open_list) > 0:
            n = None

            # encontrar nodo com a menor heuristica
            for v in open_list:
                if n is None or self.heuristicas[v] + cost[v] < self.heuristicas[n] + cost[n]:
                    n = v

            if n is None:
                print('Path does not exist!')
                return None

            # se o nodo corrente é o destino
            # reconstruir o caminho a partir desse nodo até ao start
            # seguindo o antecessor
            if n.gettype() == end:
                reconst_path = []

                while parents[n] != n:
                    reconst_path.append(n)
                    n = parents[n]

                reconst_path.append(start)

                reconst_path.reverse()

                return (reconst_path, self.calculateCost(reconst_path))

            # para todos os vizinhos  do nodo corrente
            if n.gettype() != "X":
                for (m, weight) in self.getNeighbours(n):
                    # Se o nodo corrente nao esta na open nem na closed list
                    # adiciona-lo à open_list e marcar o antecessor
                    if m not in open_list and m not in closed_list:
                        open_list.add(m)
                        parents[m] = n
                        cost[m] = self.get_pathTotalCost(start, parents[m], parents)

            # remover n da open_list e adiciona-lo à closed_list
            # porque todos os seus vizinhos foram inspecionados
            open_list.remove(n)
            closed_list.add(n)

        print('Path does not exist!')
        return None

    def get_pathTotalCost(self, start, n, parents):
        cost = 0
        while n != start:
            cost += self.getArcCost(n, parents[n])
            n = parents[n]
        return cost

    def multiplayer (self):
        ncarros=len(self.carros)

        return None


