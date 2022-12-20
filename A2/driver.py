import time, random
import argparse
import math
import sys
from checkers_game import *

from tkinter import *
from tkinter import scrolledtext

class CheckersGui(object):
    def __init__(self, player1, player2, start, player):

        self.board = start
        self.current_player = player
        self.players = [None, player1, player2]
        self.height = 8
        self.width = 8
        self.start = (-1, -1)
        self.toggle = True

        self.offset = 3
        self.cell_size = 50

        root = Tk()
        root.wm_title("Checkers")
        root.lift()
        root.attributes("-topmost", True)
        self.root = root
        self.canvas = Canvas(root, height=self.cell_size * self.height + self.offset,
                             width=self.cell_size * self.width + self.offset)
        self.move_label = Label(root)
        self.score_label = Label(root)
        self.text = scrolledtext.ScrolledText(root, width=70, height=10)
        self.move_label.pack(side="top")
        self.score_label.pack(side="top")
        self.canvas.pack()
        self.text.pack()
        self.draw_board()

    def update_state(self, state):
        self.board = state
        if self.current_player == 1:
            self.current_player = 2
        else:
            self.current_player = 1
        self.draw_board()
        self.canvas.update()

    def shutdown(self, text):
        self.move_label["text"] = text

    def run(self):
        self.draw_board()
        self.canvas.mainloop()

    def draw_board(self):
        self.draw_grid()
        self.draw_disks()
        player = "Red" if self.current_player == 1 else "Black"
        self.move_label["text"] = player
        self.score_label["text"] = "Black {} : {} Red".format(*get_score(self.board))

    def log(self, msg, newline=True):
        self.text.insert("end", "{}{}".format(msg, "\n" if newline else ""))
        self.text.see("end")

    def draw_grid(self):
        for i in range(self.height):
            for j in range(self.width):
                if (i % 2 == 0 and j % 2 == 1) or (i % 2 == 1 and j % 2 == 0):
                    self.canvas.create_rectangle(i * self.cell_size + self.offset, j * self.cell_size + self.offset,
                                                 (i + 1) * self.cell_size + self.offset,
                                                 (j + 1) * self.cell_size + self.offset, fill="dark green")
                else:
                    self.canvas.create_rectangle(i * self.cell_size + self.offset, j * self.cell_size + self.offset,
                                                 (i + 1) * self.cell_size + self.offset,
                                                 (j + 1) * self.cell_size + self.offset, fill="white")

    def draw_disk(self, i, j, color, queen):
        x = i * self.cell_size + self.offset
        y = j * self.cell_size + self.offset
        padding = 2
        if queen:
            self.canvas.create_oval(x + padding, y + padding, x + self.cell_size - padding,
                                    y + self.cell_size - padding, fill=color, outline="yellow", width=5)
        else:
            self.canvas.create_oval(x + padding, y + padding, x + self.cell_size - padding,
                                    y + self.cell_size - padding, fill=color)

    def draw_disks(self):
        for i in range(self.height):
            for j in range(self.width):
                if self.board[i][j] == 'r':
                    self.draw_disk(j, i, "red", False)
                elif self.board[i][j] == 'b':
                    self.draw_disk(j, i, "black", False)
                elif self.board[i][j] == 'R':
                    self.draw_disk(j, i, "red", True)
                elif self.board[i][j] == 'B':
                    self.draw_disk(j, i, "black", True)


def legal(move, state, player):
    if version == 0:
        return True
    else:
        if not move:
            return True
        else:
            new_state = doit(move, state)
            if new_state in successors(state, player):
                return True
            else:
                return 0

def BoardCopy(board):
    new_board = [[]]*8
    for i in range(8):
        new_board[i] = [] + board[i]
    return new_board


def doit(move, state):
    new_state = copy_board(state)
    # print("player",player)
    # Move one step
    if len(move) == 2 and abs(move[1][0] - move[0][0]) == 1:
        new_state[move[0][0]][move[0][1]] = '.'
        if state[move[0][0]][move[0][1]] == 'b' and move[1][0] == 7:
            new_state[move[1][0]][move[1][1]] = 'B'
        elif state[move[0][0]][move[0][1]] == 'r' and move[1][0] == 0:
            new_state[move[1][0]][move[1][1]] = 'R'
        else:
            new_state[move[1][0]][move[1][1]] = state[move[0][0]][move[0][1]]
    # Jump
    else:
        step = 0
        new_state[move[0][0]][move[0][1]] = '.'
        while step < len(move) - 1:
            new_state[int(math.floor((move[step][0] + move[step + 1][0]) / 2))][
                int(math.floor((move[step][1] + move[step + 1][1]) / 2))] = '.'
            step += 1
        if state[move[0][0]][move[0][1]] == 'b' and move[step][0] == 7:
            new_state[move[step][0]][move[step][1]] = 'B'
        elif state[move[0][0]][move[0][1]] == 'r' and move[step][0] == 0:
            new_state[move[step][0]][move[step][1]] = 'R'
        else:
            new_state[move[step][0]][move[step][1]] = state[move[0][0]][move[0][1]]
    return new_state


def get_score(board):
    p1_count = 0
    p2_count = 0
    for i in range(len(board)):
        for j in range(len(board)):
            if board[i][j] == 'b':
                p1_count += 1
            elif board[i][j] == 'r':
                p2_count += 1
            elif board[i][j] == 'B':
                p1_count += 2
            elif board[i][j] == 'R':
                p2_count += 2

    return p1_count, p2_count


Initial_Board = [ ['b','.','b','.','b','.','b','.'],
                  ['.','b','.','b','.','b','.','b'],
                  ['b','.','b','.','b','.','b','.'],
                  ['.','.','.','.','.','.','.','.'],
                  ['.','.','.','.','.','.','.','.'],
                  ['.','r','.','r','.','r','.','r'],
                  ['r','.','r','.','r','.','r','.'],
                  ['.','r','.','r','.','r','.','r']]


# this is the game manager
def play(file_A, file_B, alphabeta, limit, caching, ordering, start_state=Initial_Board):
    Aplayer = import_script(file_A)
    Bplayer = import_script(file_B)

    A = Aplayer.GameEngine('r')
    B = Bplayer.GameEngine('b')

    currPlayer = A
    state = start_state

    gui = CheckersGui(A, B, state, 1)

    previous_states_list = []
    draw_flag = 0
    board_num = 0
    n_repeat_allowed = 10;

    print("'r': ", file_A," is ready")
    print("'b': ", file_B," is ready")
    print()

    while True:
        print("It is ", currPlayer ,"'s turn")

        start = time.time()
        move = currPlayer.nextMove(state,alphabeta,limit,caching,ordering)
        elapse = time.time() - start

        if not move:
            break

        if currPlayer == A:
            player_name = 'r'
        else:
            player_name = 'b'

        state_with_player = [state, player_name]

        if state_with_player in previous_states_list:
            n_repeat_allowed -= 1

            ind = previous_states_list.index(state_with_player)
            ind = board_num - len(previous_states_list) + ind ;

            n_r = 0
            n_R = 0
            n_b = 0
            n_B = 0

            for i in [7,6,5,4,3,2,1,0]:
                for j in range(8):
                    if state[i][j] == 'r':
                        n_r += 1
                    elif state[i][j] == 'R':
                        n_R += 1
                    elif state[i][j] == 'b':
                        n_b += 1
                    elif state[i][j] == 'B':
                        n_B += 1

            print("** The current board is the same as the", ind, "th board!!")

            if n_r + n_R < n_b + n_B and currPlayer == A:
                if n_repeat_allowed <= 1:
                    print("** Both players are generating ineffective moves. The judgement is made by the number of remaining checkers.")
                    n_repeat_allowed = 0
            elif n_r + n_R > n_b + n_B and currPlayer == B:
                if n_repeat_allowed <= 1:
                    print("** Both players are generating ineffective moves. The judgement is made by the number of remaining checkers.")
                    n_repeat_allowed = 0
            elif n_r + n_R == n_b + n_B:
                draw_flag = 1

            if n_repeat_allowed == 0:
                break
        else:
            n_repeat_allowed = 10;

        previous_states_list.append(state_with_player);
        if(len(previous_states_list) > 100):
            previous_states_list = previous_states_list[1:]

        print("The move is : ",move, end="")
        print(" (in %.2f s)" % elapse, end="")
        if elapse > 60.0:
            print("\n** '", currPlayer, "' took more than one minute!!")
            break
        print()
        parent = BoardCopy(state);
        if legal(move, state, currPlayer.str):
            state = doit(move,state) #move takes place here

        board_num = board_num + 1
        gui.update_state(state)
        time.sleep(0.1) # Short delay

        if currPlayer == A:
            currPlayer = B
        else:
            currPlayer = A

    print("Game Over")
    if draw_flag:
        print("It is a DRAW; black wins")
        print("The Winner is:",file_B, 'b')
    elif currPlayer == A:
        print("The Winner is:",file_B, 'b')
    else:
        print("The Winner is:",file_A, 'r')


# import the user script
def import_script(name):
    module_path = name

    if module_path in sys.modules:
        return sys.modules[module_path]

    return __import__(module_path, fromlist=[name])


# main script
if __name__ == '__main__':

    parser = argparse.ArgumentParser(description='Arguments')
    parser.add_argument('player1', help='python file of first player')
    parser.add_argument('player2', help='python file of second player')
    parser.add_argument('-a', '--alphabeta', action="store_true")
    parser.add_argument('-d', '--depth', type=int)
    parser.add_argument('-c', '--caching', action="store_true")
    parser.add_argument('-o', '--ordering', action="store_true")
    args = parser.parse_args()

    a = "Minimax"
    c = d = "Off"

    Aplayer = args.player1
    Bplayer = args.player2
    alphabeta = 1 if args.alphabeta else 0
    if (alphabeta): a = "Alphabeta"
    limit = args.depth
    if limit is None: limit = 5 #default depth limit is 5!!
    caching = 1 if args.caching else 0
    if (caching): c = "On"
    ordering = 1 if args.ordering else 0
    if (ordering): d = "On"

    version = 0

    print("Playing agent {} against {}".format(Aplayer, Bplayer))
    print("Algorithm is {}, Depth Limit is {}, Caching is {} and Ordering is {}".format(a,limit,c,d))

    random.seed()
    p_coin = random.random()
    print("Determining the colors by coin flipping...[", ("%.1f" % p_coin).lstrip('0'), end="")

    game_start = time.time()
    if p_coin > 0.5:
        print(" > .5 ]: head")
        print()
        play(Aplayer, Bplayer, alphabeta, limit, caching, ordering)
    else:
        print(" < .5 ]: tail")
        print()
        play(Bplayer, Aplayer, alphabeta, limit, caching, ordering)

    game_elapse = time.time() - game_start
    print(" (Game took %.2f s)" % game_elapse)
