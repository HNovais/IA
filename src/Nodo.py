class Nodo:
    def __init__(self,type,x=-1,y=-1):
        self.type = str(type)
        self.cord = (x,y)

    def __int__(self, nodo):
        self.type = nodo.gettype()
        self.cord = nodo.getCord()

    def __str__(self):
        return self.type + ":" + str(self.cord)

    def gettype(self):
        return self.type

    def settype(self,type):
        self.type = type

    def getCord(self):
        return self.cord

    def getX(self):
        return self.cord[0]

    def getY(self):
        return self.cord[1]

    def setCord(self,x,y):
        self.cord = (x,y)

    def __repr__(self):
        return "node " + self.type + ":" + str(self.cord)

    def __eq__(self, other):
        return self.type == other.type and self.cord == other.cord

    def __hash__(self):
        return hash(self.cord)
