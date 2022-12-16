import math
import os
import time
from Nodo import Nodo
from queue import Queue, PriorityQueue
from Carro import Carro
from Path import Path

class Grafo:

    def __init__(self, directed=False):
        self.lnodos = []
        self.directed = directed
        self.grafo = {}
        self.heuristicas = {}
        self.start = "P"
        self.start2 = "2"
        self.end = "F"
        self.carros = []

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
        i = 0
        for start in self.lnodos:
            if start.gettype() == self.start:
                c = Carro(i,start)
                self.carros.append(c)
                i += 1

    # Retorna o custo do arco
    def getArcCost(self, node1, node2):
        custoT = math.inf
        a = self.grafo[node1]
        if node1==node2:
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

    def dijkstra_algorithm(self, start, end):
        q = list()
        q.append((start, 0))

        visited = list()
        visited.append(start)
        expansao = list()
        p = {}
        p[start] = None

        pathFound = False
        while len(q) > 0:
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

            if n.gettype() != "X":
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
            #if n.gettype() != "X":
            for (m, weight) in self.getNeighbours(n):
                    # Se o nodo corrente nao esta na open nem na closed list
                    # adiciona-lo à open_list e marcar o antecessor
                if m not in open_list and m not in closed_list:
                    open_list.add(m)
                    parents[m] = n
                    cost[m] = self.get_pathTotalCost(start, m, parents)

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
            time = dist

        else:
            time = dist / totVel

        return time


    def heuristicaTempo(self,vel,acc):
        end_coords_list = self.get_end_coords_list()
        for node in self.lnodos:
            node_c = node.getCord()
            for end_c in end_coords_list:
                time = self.heuristic(node_c,vel,acc,end_c)
                if node not in self.heuristicas:
                    self.heuristicas[node] = time
                elif self.heuristicas[node] > time:
                    self.heuristicas[node] = time

    def getValues(self, start, last):
        s = start.getCord()
        l = last.getCord()

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


    def multiplayer (self, start, start2, end, maze,a1,a2):
        print("criacarro")
        carro1 = Carro(1, start, "Hamilton")
        carro2 = Carro(2, start2, "Vettel")

        pos1 = start
        pos2 = start2

        path1 = [start]
        path2 = [start2]
        auxpath1 = list()
        auxpath2 = list()

        flag1 = 0
        flag2 = 0
        win = ""
        endcords=self.get_end_coords_list()

        path = Path()

        while flag1 or flag2 != 1:
            if pos1.getCord() and pos2.getCord() not in endcords:
                if a1 == 5:
                    answer1 = self.DFSSearch(pos1, end, path=[],expansao=list(),visited=set())
                    auxpath1 = answer1[1][0]
                elif a1 == 6:
                    answer1 = self.BFSSearch(pos1, end)
                    auxpath1 = answer1[1][0]
                elif a1 == 7:
                    answer1 = self.Uniform(pos1, end)
                    auxpath1 = answer1[1][0]
                elif a1 == 8:
                    answer1 = self.dijkstra_algorithm(pos1, end)
                    auxpath1 = answer1[1][0]
                elif a1 == 9:
                    self.heuristicaTempo(carro1.getVel(), carro1.getAcc())
                    answer1 = self.greedy(pos1, end)
                    auxpath1 = answer1[0]
                    #acc1 = self.getValues(pos1, proxpos1)
                    #carro1.setVel(carro1.getVelCol() + carro1.getAccCol(), carro1.getVelLine() + carro1.getAccLine())
                elif a1 == 10:
                    self.heuristicaTempo(carro1.getVel(),carro1.getAcc())
                    answer1=self.AStar(pos1,end)
                    auxpath1 = answer1[0]
                    #acc1 = self.getValues(pos1, proxpos1)
                    #carro1.setVel(carro1.getVelCol() + carro1.getAccCol(), carro1.getVelLine() + carro1.getAccLine())

                if a2 == 5:
                    answer2 = self.DFSSearch(pos2, end, path=[],expansao=list(),visited=set())
                    auxpath2 = answer2[1][0]
                elif a2 == 6:
                    answer2 = self.BFSSearch(pos2, end)
                    auxpath2 = answer2[1][0]
                elif a2 == 7:
                    answer2 = self.Uniform(pos2, end)
                    auxpath2 = answer2[1][0]
                elif a2 == 8:
                    answer2 = self.dijkstra_algorithm(pos2, end)
                    auxpath2 = answer2[1][0]
                elif a2 == 9:
                    self.heuristicaTempo(carro2.getVel(), carro2.getAcc())
                    answer2 = self.greedy(pos2, end)
                    auxpath2 = answer2[0]
                    #acc2 = self.getValues(pos2, proxpos2)
                    #carro2.setVel(carro2.getVelCol() + carro2.getAccCol(), carro2.getVelLine() + carro2.getAccLine())
                elif a2 == 10:
                    self.heuristicaTempo(carro2.getVel(),carro2.getAcc())
                    answer2=self.AStar(pos2,end)
                    auxpath2 = answer2[0]
                    #acc2 = self.getValues(pos2, proxpos2)
                    #carro2.setVel(carro2.getVelCol() + carro2.getAccCol(), carro2.getVelLine() + carro2.getAccLine())


                # Astar retorna um tuplo com o caminho e o custo respetivo
                # Pegamos no caminho só fazendo answer[0]
                #auxpath1 = answer1[0]
                #auxpath2 = answer2[1][0]
                # Depois pegamos no auxpath[1] que será a próxima posição, sendo auxpath[0] a posição atual
                proxpos1 = auxpath1[1]
                proxpos2 = auxpath2[1]

                acc1 = self.getValues(pos1, proxpos1)
                carro1.setAcc(acc1[0], acc1[1])
                carro1.setVel(carro1.getVelCol() + carro1.getAccCol(), carro1.getVelLine() + carro1.getAccLine())
                acc2 = self.getValues(pos2, proxpos2)
                carro2.setAcc(acc2[0],acc2[1])
                carro2.setVel(carro2.getVelCol() + carro2.getAccCol(), carro2.getVelLine() + carro2.getAccLine())
                
                if proxpos1 == proxpos2:
                    if carro1.getVel() > carro2.getVel():
                        proxpos2 = pos2
                        carro2.setAcc(0,0)
                        carro2.setVel(0,0)
                    else:
                        proxpos1 = pos1
                        carro1.setAcc(0,0)
                        carro1.setVel(0,0)


                path1.append(proxpos1)
                path2.append(proxpos2)

                pos1 = proxpos1
                pos2 = proxpos2

            elif pos1.getCord() in endcords:
                flag1 = 1
                win = carro1.name
            
                self.heuristicaTempo(carro2.getVel(),carro2.getAcc())
                answer2 = self.AStar(pos2, end)
                if len(answer2[0]) >1:
                    proxpos2 = answer2[0][1]
                
                    aux1 = self.getValues(pos2, proxpos2)
                    carro2.setAcc(aux1[0], aux1[1])
                    carro2.setVel(carro2.getVelCol() + carro2.getAccCOl(), carro2.getVelLine() + carro2.getAccLine())

                    path2.append(proxpos2)
                    pos2 = proxpos2
                else: flag2 = 1

            elif pos2.getCord() in endcords:
                flag2 = 1
                win = carro2.name

                if len(answer1[0]) >1:
                    self.heuristicaTempo(carro1.getVel(),carro1.getAcc())
                    answer1=self.AStar(pos1,end)
                    proxpos1 = answer1[0][1]

                    aux2 = self.getValues(pos1, proxpos1)
                    carro1.setAcc(aux2[0], aux2[1])
                    carro1.setVel(carro1.getVelCol() + carro1.getAccCol(), carro1.getVelLine() + carro1.getAccLine())

                    path1.append(proxpos1)
                    pos1 = proxpos1

                else: flag1 = 1

            os.system("clear")
            path.colorRace(path1, path2, maze)
            time.sleep(0.3)
        os.system("clear")
        return ((path1, path2) ,(self.calculateCost(path1),self.calculateCost(path2)), win)