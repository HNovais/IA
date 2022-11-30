from termcolor import colored

class Path:

    def __init__(self):
        self.an = []
        self.mz = []

    # Print ao circuito com cores no terminal
    def colorPath(self, answer, maze):
        nodes = answer[0]
        coords = []

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
              



        

    
