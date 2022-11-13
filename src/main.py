from Grafo import Grafo

def main():
    # Criar instância de grafo
    g = Grafo()

    # Abrir ficheiro .txt com a pista
    file = open(r"d:\Universidade\IA\Trabalho\IA-main\src\maze.txt","r")
    maze = file.readlines()

    g.addEdges(maze)

    saida = -1
    while saida != 0:
        print("1-Imprimir Grafo")
        print("2-Imprimir nodos de Grafo")
        print("3-DFS")
        print("4-BFS")
        print("0-Saír")

        saida = int(input("introduza a sua opcao-> "))
        if saida == 0:
            print("saindo.......")
        elif saida == 1:
            # Escrever o grafo como string
            print(g)
            l = input("prima enter para continuar")
        elif saida == 2:
            # Imprimir as chaves do dicionario que representa o grafo
            print(g.grafo.keys())
            l = input("prima enter para continuar")
        elif saida == 3:
            # Efetuar  pesquisa de caminho entre nodo inicial e final com DFS
            inicio = g.getStart(input("Nodo inicial->"))
            fim = input("Nodo final->")
            print(g.DFSSearch(inicio, fim, path=[], visited=set()))
            l = input("prima enter para continuar")
        elif saida == 4:
            # Efetuar  pesquisa de caminho entre nodo inicial e final com Gulosa
            inicio = g.getStart(input("Nodo inicial->"))
            fim = input("Nodo final->")
            print(g.BFSSearch(inicio, fim))
            l = input("prima enter para continuar")
        else:
            print("Opção inválida...")
            l = input("prima enter para continuar")

    file.close()

if __name__ == "__main__":
    main()
