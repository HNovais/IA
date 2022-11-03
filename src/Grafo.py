from Nodo import Nodo


class Grafo:

    def __init__(self, directed=False):
        self.lnodos = []
        self.directed = directed
        self.grafo = {}

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
