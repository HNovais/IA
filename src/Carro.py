class Carro:
    def __init__(self,id,x=-1,y=-1):
        self.id = id
        self.coord = (x,y)
        self.velCol = 0
        self.velLine = 0
        self.accCol = 0
        self.accLine = 0

    def __str__(self):
        return self.id + " " + self.coord + ": Velocity (" + self.velCol + "," + self.velLine + "); Acceleration (" + self.accCol + "," + self.accCol + ")" 

    def getId(self):
        return self.id

    def setId(self, id):
        self.id = id
    
    def getCoord(self):
        return self.coord

    def setCoord(self, coord):
        self.coord = coord

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
    