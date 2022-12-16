from termcolor import colored

class Path:

    def __init__(self):
        self.an = []
        self.mz = []

    # Print ao circuito com cores no terminal
    def colorPath(self, answer, maze):
        coords = []
        nodes = answer[0]
        for node in nodes:
            n = str(node)
            coord = n.split(':')
            coords.append(str(coord[1]))

        y = 0

        print()

        for line in maze:
            x = 0
            for char in line[:-1]:
                xy = str((y, x))
                if xy in coords:
                    print(colored(char, 'white', 'on_green', attrs=["bold"]), end='')
                else:
                    print(char, end='')
                x = x + 1
            print()
            y = y + 1

        print()  

    def colorRace(self, answer1, answer2, maze):


        coords1 = []
        coords2 = []

        for node in answer1:
            n = str(node)
            coord = n.split(':')
            coords1.append(str(coord[1]))

        for node in answer2:
            n = str(node)
            coord = n.split(':')
            coords2.append(str(coord[1]))

        y = 0

        print()

        coords = [value for value in coords1 if value in coords2]

        for line in maze:
            x = 0
            for char in line[:-1]:
                xy = str((y, x))
                if xy in coords:
                    print(colored(char, 'white', 'on_yellow', attrs=["bold"]), end='')
                elif xy in coords1:
                    print(colored(char, 'white', 'on_green', attrs=["bold"]), end='')
                elif xy in coords2:
                    print(colored(char, 'white', 'on_red', attrs=["bold"]), end='')
                else:
                    print(char, end='')
                x = x + 1
            print()
            y = y+1

        print()
