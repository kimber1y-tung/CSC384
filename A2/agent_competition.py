import random
import sys
import time

# You can use the functions to write your AI for competition
from __future__ import nested_scopes
from checkers_game import *

# If you choose to try MCTS, you can make use of the code below
class MCTS_state():
    """
            This sample code gives you a idea of how to store records for each node
            in the tree. However, you are welcome to modify this part or define your own
            class.
    """
    def __init__(self, ID, parent, child, reward, total, board):
        self.ID = ID
        self.parent = parent    # a list of states
        self.child = child      # a list of states
        self.reward = reward    # number of win
        self.total = total      # number of simulation for self and (grand*)children
        self.board = board
        self.visited = 0        # 0 -> not visited yet, 1 -> already visited


def select_move_MCTS(state, color):
    """
               You can add additional help functions as long as this function will return a position tuple
    """
    initial_state = MCTS_state(0, [], [], 0, 0, board) # this is just an example. delete it when you start to code.
    pass

# ======================== Class GameEngine =======================================
class GameEngine:
    def __init__(self, str_name):
        self.str = str_name

    def __str__(self):
        return self.str

    # The return value should be a move that is denoted by a list
    def nextMove(self, state, alphabeta, limit, caching, ordering):
        global PLAYER
        PLAYER = self.str
        result = select_move_MCTS(Board(state), PLAYER)

        return result