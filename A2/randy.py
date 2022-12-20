from checkers_game import *
import random

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
        succmoves = successors(Board(state), PLAYER)
        result = None
        if len(succmoves) > 0:
            #print(random.choice(succmoves))
            result = random.choice(succmoves).move
        return result
