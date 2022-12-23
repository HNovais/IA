import random

def movimento():
    n = random.randint(1,4)
    if n == 1:
        return "CIMA"
    elif n == 2:
        return "BAIXO"
    elif n == 3:
        return "ESQUERDA"
    elif n == 4:
        return "DIREITA"

def guardarEmFicheiro(matriz, file):
    fileaux = open(file, "w")
    for linha in matriz:
        for elem in linha:
            fileaux.write(elem)
        fileaux.write("\n")
    fileaux.close()
def criarpeca():
    n = random.randint(1,6)
    if n == 1 or n == 2 or n == 3 or n == 4:
        return '-'
    elif n == 5 or n == 6:
        return 'X'

def gerarMapa(file):
    largura = random.randint(10,30)
    comprimento = random.randint(20,50)

    mapa = []
    linha = []
    i=0
    j=0

    while i<largura:
        while j<comprimento:
            linha.append(criarpeca())
            j += 1
        mapa.append(linha)
        linha = []
        i += 1
        j = 0

    lpartida = random.randint(0, largura-1)
    cpartida = 0
    mapa[lpartida][cpartida] = 'P'

    lmeta = random.randint(0, largura-1)
    cmeta = comprimento - 1

    i = lpartida
    j = cpartida

    while (i,j) != (lmeta, cmeta):
        move = movimento()
        if move == "CIMA":
            if i-1 >= 0 and (i-1,j) != (lpartida, cpartida):
                i = i-1
                mapa[i][j] = '-'
        elif move == "BAIXO":
            if i+1 < largura and (i+1,j) != (lpartida, cpartida):
                i = i+1
                mapa[i][j] = '-'

        elif move == "ESQUERDA":
            if j+1 < comprimento and (i,j+1) != (lpartida, cpartida):
                j= j+1
                mapa[i][j] = '-'

        elif move == "DIREITA":
            if j-1 >= 0 and (i,j-1) != (lpartida, cpartida):
                j = j-1
                mapa[i][j] = '-'

    mapa[lmeta][cmeta] = 'F'

    guardarEmFicheiro(mapa, file)

if __name__ == "__main__":
    gerarMapa("../gerarMapa.txt")