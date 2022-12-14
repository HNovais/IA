from Nodo import Nodo

class Carro:
    def __init__(self,id, node, name = ""):
        self.id = id
        self.node = node
        self.velCol = 0
        self.velLine = 0
        self.accCol = 0
        self.accLine = 0
        self.vel = (0, 0)
        self.acc = (0, 0)
        self.name = name

    def __str__(self):
        return self.id + " " + self.coord + ": Velocity (" + self.velCol + "," + self.velLine + "); Acceleration (" + self.accCol + "," + self.accCol + ")" 

    def getId(self):
        return self.id

    def setId(self, id):
        self.id = id
    
    def getNode(self):
        return self.node

    def setNode(self, node):
        self.node = node

    def getVelCol(self):
        return self.velCol

    def setVelCol(self, velCol):
        self.velCol = velCol

    def getVelLine(self):
        return self.velLine

    def setVelLine(self, velLine):
        self.velLine = velLine
    
    def getAccCol(self):
        return self.accCol

    def setAccCol(self, accCol):
        self.accCol = accCol
    
    def getAccLine(self):
        return self.accLine

    def setAccLine(self, accLine):
        self.accLine = accLine

    def getVel(self):
        return self.vel
    
    def setVel(self, velCol, velLine):
        self.vel = (velCol, velLine)

    def getAcc(self):
        return self.acc

    def setAcc(self, accCol, accLine):
        self.acc = (accCol, accLine)

    def getName(self):
        return self.name
    
    def setName(self, name):
        self.name = name

    
    
