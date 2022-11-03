from Grafo import Grafo

def main():
    # Criar instância de grafo
    g = Grafo()

    file = open("maze.txt", "r")
    maze = file.readlines()
    print(maze)
    file.close

    j=0

    while j<7:
        i=0
        while i<10:
            if i<9:
                g.add_edge(maze[j][i], i, j, maze[j][i+1], i+1, j)
            if j<6:
                g.add_edge(maze[j][i], i, j, maze[j+1][i], i, j+1)
            i=i+1
        j=j+1

    print(len(g.grafo.keys()))
    print(g)

if __name__ == "__main__":
    main()
