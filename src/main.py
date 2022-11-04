from Grafo import Grafo

def main():
    # Criar instância de grafo
    g = Grafo()

    # Abrir ficheiro .txt com a pista
    file = open(r"d:\Universidade\IA\Trabalho\IA-main\src\maze.txt","r")
    maze = file.readlines()

    # Criar ligações entre nodos
    j = 0
    for line in maze:
        i = 0
        for char in line[:-1]:
            if i < len(line) - 2:
                g.add_edge(maze[j][i], i, j, maze[j][i+1], i+1, j)
            if j < len(maze) - 1:
                g.add_edge(maze[j][i], i, j, maze[j+1][i], i, j+1)
            i = i + 1
        j = j + 1

    print(g.procura_BFS(g.getStart(), "F"))

    file.close()

if __name__ == "__main__":
    main()
