class Nodo:
    def __init__(self,tipo,x=-1,y=-1):
        self.tipo = str(tipo)
        self.cord = (x,y)

    def __int__(self, nodo):
        self.tipo = nodo.getTipo()
        self.cord = nodo.getCord()

    def __str__(self):
        return "node " + self.tipo + ":" + str(self.cord)

    def getTipo(self):
        return self.tipo

    def setTipo(self,tipo):
        self.tipo=tipo

    def getCord(self):
        return self.cord

    def setCord(self,x,y):
        self.cord=(x,y)

    def __repr__(self):
        return "node " + self.tipo + ":" + str(self.cord)

    def __eq__(self, other):
        return self.tipo == other.tipo and self.cord == other.cord

    def __hash__(self):
        return hash(self.cord)