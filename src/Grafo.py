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

    def add_edge(self, node1,x1,y1, node2,x2,y2):
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

    def getStart(self):
        for start in self.lnodos:
            if start.tipo == "P":
                return start
        return None

    def get_arc_cost(self, node1, node2):
        custoT=math.inf
        a=self.grafo[node1]   
        for (nodo,custo) in a:
            if nodo==node2:
                custoT=custo

        return custoT

    def calcula_custo(self, caminho):
        teste=caminho
        custo=0
        i=0
        while i+1 < len(teste):
             custo=custo + self.get_arc_cost(teste[i], teste[i+1])
             i=i+1
        return custo    

    def procura_BFS(self, start, end):
        q = Queue()
        visitado = set()

        q.put(start)
        visitado.add(start)
        p = {}
        p[start] = None

        path_found = False
        while not q.empty() and path_found == False:
            n = q.get()
            if n.getTipo() == end:
                path_found = True
            else:
                for (adjacente, peso) in self.grafo[n]:
                    if adjacente not in visitado:
                        visitado.add(adjacente)
                        q.put(adjacente)
                        p[adjacente] = n
                    if adjacente.getTipo() == end:
                        endNode = adjacente
                        path_found = True
        path = []
        if path_found == True:
            path.append(endNode)
            while p[endNode] is not None:
                path.append(p[endNode])
                endNode = p[endNode]
            path.reverse()
            custoT = self.calcula_custo(path)
            return (path, custoT)
