import os

from Grafo import Grafo
from Path import Path
import Gerador


def main():
    # Criar instância de grafo
    g = Grafo()
    f = "pista.txt"

    # Abrir ficheiro .txt com a pista
    file = open(f, "r")
    maze = file.readlines()

    # Adiciona as ligações entre nodos
    g.addEdges(maze)
    g.calculate_heuristic()

    # GUI
    saida = -1
    while saida != 0:
        print("1-Imprimir Grafo")
        print("2-Imprimir nodos de Grafo")
        print("3-Imprimir Circuito")
        print("4.Gerar Circuito aleatorio")
        print("5-Alterar Circuito")
        print("6-DFS")
        print("7-BFS")
        print("8-Uniform")
        print("9-Dijkstra")
        print("10-Gulosa")
        print("11-AStar")
        print("12-Multijogador")
        print("0-Saír")

        saida = int(input("introduza a sua opcao-> "))
        if saida == 0:
            print("saindo.......")
        elif saida == 1:
            # Escrever o grafo como string
            print(g.__str__())
            l = input("prima enter para continuar")
            os.system("clear")
        elif saida == 2:
            # Imprimir as chaves do dicionario que representa o grafo
            for k in g.grafo.keys():
                print(k.__str__())
            l = input("prima enter para continuar")
            os.system("clear")
        elif saida == 3:
            # Imprimir o grafo presente no ficheiro track.txt
            print("\n")
            for line in maze:
                print(line, end='')
            print("\n")
            l = input("prima enter para continuar")
            os.system("clear")
        elif saida == 4:
            gerarfile = input("Nome do ficheiro: ")
            Gerador.gerarMapa(gerarfile)
        elif saida == 5:
            f = input("Novo ficheiro:")
            file = open(f, "r")
            g = Grafo()
            maze = file.readlines()
            g.addEdges(maze)
            g.setCarros()
            l = input("prima enter para continuar")
            os.system("clear")
        elif saida == 6:
            # Efetuar  pesquisa de caminho entre nodo inicial e final com DFS
            g.getStartPositions()
            inicio = g.getStart()
            fim = g.end
            path = Path()
            answer = (g.DFSSearch(inicio, fim, path=[], expansao=list(), visited=set()))
            print((answer[0], 0))
            path.colorPath((answer[0], 0), maze)
            print(answer[1])
            path.colorPath(answer[1], maze)
            l = input("prima enter para continuar")
            os.system("clear")
        elif saida == 7:
            # Efetuar  pesquisa de caminho entre nodo inicial e final com BFS
            g.getStartPositions()
            inicio = g.getStart()
            fim = g.end
            path = Path()
            answer = (g.BFSSearch(inicio, fim))
            print((answer[0], 0))
            path.colorPath((answer[0], 0), maze)
            print(answer[1])
            path.colorPath(answer[1], maze)
            l = input("prima enter para continuar")
            os.system("clear")
        elif saida == 8:
            # Efetuar pesquisa de caminho ente nodo inicial e final com Uniform
            g.getStartPositions()
            inicio = g.getStart()
            fim = g.end
            path = Path()
            answer = (g.Uniform(inicio, fim))
            print((answer[0], 0))
            path.colorPath((answer[0], 0), maze)
            print(answer[1])
            path.colorPath(answer[1], maze)
            l = input("prima enter para continuar")
            os.system("clear")
        elif saida == 9:
            # Efetuar pesquisa de caminho ente nodo inicial e final com Uniform~
            g.getStartPositions()
            inicio = g.getStart()
            fim = g.end
            path = Path()
            answer = (g.Dijkstra(inicio, fim))
            print((answer[0], 0))
            path.colorPath((answer[0], 0), maze)
            print(answer[1])
            path.colorPath(answer[1], maze)
            l = input("prima enter para continuar")
            os.system("clear")
        elif saida == 10:
            # Efetuar pesquisa de caminho ente nodo inicial e final com Uniform
            g.getStartPositions()
            inicio = g.getStart()
            fim = g.end
            path = Path()
            answer = (g.greedy(inicio, fim))
            print((answer[0], 0))
            path.colorPath((answer[0], 0), maze)
            print(answer[1])
            path.colorPath(answer[1], maze)
            l = input("prima enter para continuar")
            os.system("clear")
        elif saida == 11:
            # Efetuar pesquisa de caminho ente nodo inicial e final com Uniform
            g.getStartPositions()
            inicio = g.getStart()
            fim = g.end
            path = Path()
            answer = (g.AStar(inicio, fim))
            print((answer[0], 0))
            path.colorPath((answer[0], 0), maze)
            print(answer[1])
            path.colorPath(answer[1], maze)
            l = input("prima enter para continuar")
            os.system("clear")
        elif saida == 12:
            g.getStartPositions()
            inicio = g.getStart()
            inicio2 = g.getStart()
            fim = g.end
            path = Path()
            print("\n(Escolha segundo os valores acima)")
            a1 = int(input("Algoritmo para o primeiro carro: "))
            n1 = input("Nome do(a) piloto(a): ")
            a2 = int(input("Algoritmo para o segundo carro: "))
            n2 = input("Nome do(a) plioto(a): ")
            answer = (g.multiplayer(inicio, inicio2, fim, maze, a1, a2, n1, n2))

            print("O algoritmo do(a) "+n1+" teve um custo de "+str(answer[1][0]))
            print("O algoritmo do(a) "+n2+" teve um custo de "+str(answer[1][1]))

            path.colorRace(answer[0][0], answer[0][1], maze)
            print("O Vencedor da corrida é " + answer[2])

            l = input("\nprima enter para continuar")
            os.system("clear")
        else:
            print("\33[31mOpção inválida...\033[m")
            l = input("prima enter para continuar")
            os.system("clear")

    file.close()


if __name__ == "__main__":
    main()
