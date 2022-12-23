import math
import heapq
import os
import time
from Nodo import Nodo
from queue import Queue, PriorityQueue
from Carro import Carro
from Path import Path


class Grafo:

    def __init__(self, directed=False):
        self.lnodos = []
        self.startPos = set()
        self.directed = directed
        self.grafo = {}
        self.heuristicas = {}
        self.end = "F"
        self.carros = []

    def __str__(self):
        out = "\033[34mGrafo\033[m\n"
        for key in self.grafo.keys():
            out = out + str(key) + ": " + str(self.grafo[key]) + "\n"
        return out

    # Adiciona arco
    def addEdge(self, node1, x1, y1, node2, x2, y2):
        n1 = Nodo(node1, x1, y1)
        n2 = Nodo(node2, x2, y2)
        peso = 1

        if node2 == "X":
            peso = 25

        if n1 not in self.lnodos:
            self.lnodos.append(n1)
            self.grafo[n1] = set()

        if n2 not in self.lnodos:
            self.lnodos.append(n2)
            self.grafo[n2] = set()

        self.grafo[n1].add((n2, peso))

        if not self.directed:
            if node1 == "X":
                peso = 25
            else:
                peso = 1
            self.grafo[n2].add((n1, peso))

    def getStartPositions(self):
        for start in self.lnodos:
            if start.type == "P":
                self.startPos.add(start)

    # Localiza o nodo inicial
    def getStart(self):
        for start in self.startPos:
                pos = start
                self.startPos.remove(pos)
                return pos
        return None

    def getPrimCarro(self):
        return self.carros[0]

    def setCarros(self):
        i = 0
        for start in self.lnodos:
            if start.gettype() == "P":
                c = Carro(i, start)
                self.carros.append(c)
                i += 1

    # Retorna o custo do arco
    def getArcCost(self, node1, node2):
        custoT = math.inf
        a = self.grafo[node1]
        if node1 == node2:
            return 0
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
            cost = cost + self.getArcCost(teste[i], teste[i + 1])
            i = i + 1
        return cost

    def add_heuristica(self, n, heuristca):
        n1 = Nodo(n)
        if n1 in self.m_nodes:
            self.heuristicas[n] = heuristca

    def manhattan_dist(self, c1, c2):
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

    def getSmallest(self, nodes):
        for node in nodes:
            if node.gettype() == "-":
                return node

        return nodes[0]

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
    def DFSSearch(self, start, end, path=[], expansao=list(), visited=set()):
        path.append(start)
        expansao.append(start)
        visited.add(start)

        if start.gettype() == end:
            expansao.pop()
            custoT = self.calculateCost(path)
            return (expansao, (path, custoT))
        if start.gettype() != 'X':
            for (adjacente, peso) in self.grafo[start]:
                if adjacente not in visited:
                    resultado = self.DFSSearch(adjacente, end, path, expansao, visited)
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
            return (expansao, (path, custoT))

    def Uniform(self, start, end):
        q = list()
        q.append((start, 0))

        visited = list()
        visited.append(start)
        expansao = list()
        p = {}
        p[start] = None

        pathFound = False
        while len(q) > 0 and pathFound == False:
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
                            q.append((adjacent, n[1] + self.getArcCost(n[0], adjacent)))
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
            return (expansao, (path, custoT))

    
    def Dijkstra(self, start, end):
        # Inicializa a fila de prioridade com a distância e o nó inicial
        q = [(0, start)]

        # Inicializa os dicionários de distâncias e nós anteriores com entradas para o nó inicial  
        distances = {start: 0}  
        previous = {start: None}  

        # Inicializa a lista de nós visitados
        visited = []  

        # Enquanto houver nós na fila de prioridade
        while q:  
            # Seleciona o nó com a menor distância
            distance, node = heapq.heappop(q)

            # Pula o nó se ele já foi visitado
            if node in visited:
                continue  

            # Adiciona o nó à lista de nós visitados
            visited.append(node) 

            # Se o tipo do nó é igual ao tipo do destino, reconstrói o caminho mais curto e retorna a distância 
            if node.gettype() == end:  
                # Reconstrói o caminho mais curto
                path = []
                current = node
                while current is not None:
                    path.append(current)
                    current = previous[current]
                path = path[::-1]         

                # Calcula o custo total do caminho       
                cost = self.calculateCost(path)
                return (visited, (path, cost))
            
            # Atualiza as distâncias dos vizinhos
            for adjacent, cost in self.grafo[node]:
                if adjacent not in visited:
                    alt = distance + self.getArcCost(node, adjacent)
                    if alt < distances.get(adjacent, float('inf')):
                        distances[adjacent] = alt
                        previous[adjacent] = node
                        heapq.heappush(q, (alt, adjacent))

        # Se o destino não foi alcançado, retorna -1 para indicar que o destino não é alcançável
        return (visited, -1)




    def greedy(self, start, end_name, vel=(0, 0), acc=(0, 0)):
        # open_list é uma lista de nodos visitados, mas com vizinhos
        # que ainda não foram todos visitados, começa com o  start
        # closed_list é uma lista de nodos visitados
        # e todos os seus vizinhos também já o foram
        open_list = set()
        closed_list = set()

        open_list.add(start)
        expansao = list()
        # parents é um dicionário que mantém o antecessor de um nodo
        # começa com start
        parents = dict()
        parents[start] = start

        end_coords_list = self.get_end_coords_list()
        nant = start

        end_list = self.get_end_coords_list()
        tempheuristica=dict()
        velD=dict()
        accD=dict()

        while len(open_list) > 0:
            n = None

            # encontrado nodo com a menor heuristica
            for v in open_list:
                for end_c in end_coords_list:
                    if n is None or self.heuristic(v.getCord(), vel, acc, end_c) < self.heuristic(n.getCord(), vel, acc,
                                                                                                  end_c):
                        n = v
            expansao.append(n)
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

                return (expansao, (reconst_path, self.calculateCost(reconst_path)))

            if n is not start:
                acc = self.getValues(nant.getCord(), n.getCord())
                vel = (vel[0] + acc[0], vel[1] + acc[1])
                nant = n
            if n.gettype() != "X":
                # para todos os vizinhos  do nodo corrente
                for (m, weight) in self.getNeighbours(n):
                    # Se o nodo corrente nao esta na open nem na closed list
                    # adiciona-lo à open_list e marcar o antecessor
                    if m not in open_list and m not in closed_list:
                        open_list.add(m)

                        if n is not start:
                            acc = accD[n]
                            vel = velD[n]

                        mCord = m.getCord()
                        end_c = self.nearestEnd(mCord, end_list)
                        acc = self.getValues(n.getCord(), m.getCord())
                        h = self.heuristic(m.getCord(), vel, acc, end_c)
                        tempheuristica[m] = h[0]
                        velD[m] = h[1]
                        accD[m] = acc

                        parents[m] = n

            # remover n da open_list e adiciona-lo à closed_list
            # porque todos os seus vizinhos foram inspecionados
            open_list.remove(n)
            closed_list.add(n)

        print('Path does not exist!')
        return None

    def AStar(self, start, end, vel=(0, 0), acc=(0, 0)):
        # open_list é uma lista de nodos visitados, mas com vizinhos
        # que ainda não foram todos visitados, começa com o  start
        # closed_list é uma lista de nodos visitados
        # e todos os seus vizinhos também já o foram
        open_list = set()
        closed_list = set()

        open_list.add(start)
        expansao = list()
        # parents é um dicionário que mantém o antecessor de um nodo
        # começa com start
        parents = dict()
        parents[start] = start

        # cost é um dicionário que mantém o custo do caminho desde o ínicio do grafo até a um nodo
        # começa com 0
        cost = dict()
        cost[start] = 0

        end_list = self.get_end_coords_list()

        tempheuristica=dict()
        velD=dict()
        accD=dict()

        while len(open_list) > 0:
            n = None
            # encontrar nodo com a menor heuristica
            for v in open_list:
                if n is None or tempheuristica[v] + cost[v] < tempheuristica[n] + cost[n]:
                    n = v
            expansao.append(n)
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

                return (expansao, (reconst_path, self.calculateCost(reconst_path)))

            # para todos os vizinhos  do nodo corrente
            for (m, weight) in self.getNeighbours(n):
                # Se o nodo corrente nao esta na open nem na closed list
                # adiciona-lo à open_list e marcar o antecessor
                if m not in open_list and m not in closed_list:
                    open_list.add(m)

                    if n is not start:
                        acc = accD[n]
                        vel = velD[n]

                    mCord=m.getCord()
                    end_c = self.nearestEnd(mCord,end_list)
                    acc = self.getValues(n.getCord(), m.getCord())
                    h=self.heuristic(m.getCord(), vel, acc, end_c)
                    tempheuristica[m]=h[0]
                    velD[m]=h[1]
                    accD[m]=acc

                    parents[m] = n
                    cost[m] = self.get_pathTotalCost(start, m, parents)


            # remover n da open_list e adiciona-lo à closed_list
            # porque todos os seus vizinhos foram inspecionados
            open_list.remove(n)
            closed_list.add(n)

        print('Path does not exist!')
        return None

    def nearestEnd(self,coord, coords):
        closest_coord = None
        closest_distance = float("inf")  # Initialize the closest distance with infinity
        for c in coords:  # Iterate over the coordinates in the list
            # Calculate the distance between the given coordinate and the current coordinate
            distance = math.sqrt((coord[0] - c[0]) ** 2 + (coord[1] - c[1]) ** 2)
            if distance < closest_distance:  # Update the closest coordinate and distance if the current distance is smaller
                closest_coord = c
                closest_distance = distance
        return closest_coord
    def get_pathTotalCost(self, start, n, parents):
        cost = 0
        while n != start:
            cost += self.getArcCost(parents[n],n)
            n = parents[n]
        return cost

    def heuristic(self, pos, vel, acc, end):
        p1 = pos[0] + vel[0] + acc[0]
        p2 = pos[1] + vel[1] + acc[1]

        newPos = (p1, p2)

        v1 = vel[0] + acc[0]
        v2 = vel[1] + acc[1]

        newVel = (v1, v2)

        sqrDist = (newPos[0] - end[0]) ** 2 + (newPos[1] - end[1]) ** 2
        dist = math.sqrt(sqrDist)

        totVel = math.sqrt(newVel[0] ** 2 + newVel[1] ** 2)

        if (totVel == 0):
            time = math.inf

        else:
            time = (dist / totVel)

        return (time,newVel)

    def heuristicaTempo(self, vel, acc):
        end_coords_list = self.get_end_coords_list()
        for node in self.lnodos:
            node_c = node.getCord()
            for end_c in end_coords_list:
                time = self.heuristic(node_c, vel, acc, end_c)
                if node not in self.heuristicas:
                    self.heuristicas[node] = time
                elif self.heuristicas[node] > time:
                    self.heuristicas[node] = time

    def getValues(self, start, last):
        s = start
        l = last

        acc = (0, 0)

        if (l[0] > s[0]):
            if (l[1] > s[1]):
                acc = (1, 1)
            elif (l[1] == s[1]):
                acc = (1, 0)
            else:
                acc = (1, -1)
        elif (l[0] == s[0]):
            if (l[1] > s[1]):
                acc = (0, 1)
            elif (l[1] == s[1]):
                acc = (0, 0)
            else:
                acc = (0, -1)
        else:
            if (l[1] > s[1]):
                acc = (-1, 1)
            elif (l[1] == s[1]):
                acc = (-1, 0)
            else:
                acc = (-1, -1)

        return acc

    def escolhaAlgoritmo(self, pos, end, a):
        auxpath = list()
        if a == 6:
            answer1 = self.DFSSearch(pos, end, path=[], expansao=list(), visited=set())
            auxpath = answer1[1][0]
        elif a == 7:
            answer1 = self.BFSSearch(pos, end)
            auxpath = answer1[1][0]
        elif a == 8:
            answer1 = self.Uniform(pos, end)
            auxpath = answer1[1][0]
        elif a == 9:
            answer1 = self.Dijkstra(pos, end)
            auxpath = answer1[1][0]
        elif a == 10:
            answer1 = self.greedy(pos, end)
            auxpath = answer1[1][0]
        elif a == 11:
            answer1 = self.AStar(pos, end)
            auxpath = answer1[1][0]
        return auxpath

    def multiplayer(self, start, start2, end, maze, a1, a2, n1, n2):
        pos1 = start
        pos2 = start2

        path1 = list()
        path2 = list()
        flag1 = 0
        flag2 = 0
        acc1 = (0, 0)
        acc2 = (0, 0)
        vel1 = (0, 0)
        vel2 = (0, 0)
        win = ""

        path = Path()

        auxpath1 = self.escolhaAlgoritmo(pos1, end, a1)
        auxpath2 = self.escolhaAlgoritmo(pos2, end, a2)

        end_coords_list = self.get_end_coords_list()

        repetidos = set()

        for i in range(max(len(auxpath1), len(auxpath2))):
            if flag1 == 0 and flag2 == 0:
                if auxpath1[i] == auxpath2[i]:
                    repetidos.add(auxpath1[i])

                    if (vel2 > vel1):
                        del auxpath1[i:]
                        new = self.escolhaAlgoritmo(auxpath1[i - 1], end, a1)
                        auxpath1.extend(new)
                    else:
                        del auxpath2[i:]
                        new = self.escolhaAlgoritmo(auxpath2[i - 1], end, a2)
                        auxpath2.extend(new)

                path1.append(auxpath1[i])
                path2.append(auxpath2[i])

                if i != 0:
                    acc1 = self.getValues(auxpath1[i - 1].getCord(), auxpath1[i].getCord())
                    acc2 = self.getValues(auxpath2[i - 1].getCord(), auxpath2[i].getCord())
                vel1 = (vel1[0] + acc1[0], vel1[1] + acc1[1])
                vel2 = (vel2[0] + acc2[0], vel2[1] + acc2[1])

                if auxpath1[i].getCord() in end_coords_list and auxpath2[i].getCord() in end_coords_list:
                    flag1 = 1
                    flag2 = 1
                    win = "empate"
                elif auxpath1[i].getCord() in end_coords_list:
                    flag1 = 1
                    win = n1
                elif auxpath2[i].getCord() in end_coords_list:
                    flag2 = 1
                    win = n2
            elif flag2 == 0:
                path2.append(auxpath2[i])
            elif flag1 == 0:
                path1.append(auxpath1[i])
            os.system("clear")
            path.colorRace(path1, path2, maze)
            time.sleep(0.3)
        os.system("clear")
        print(repetidos)
        return ((auxpath1, auxpath2), (self.calculateCost(auxpath1), self.calculateCost(auxpath2)), win)