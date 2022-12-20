from __future__ import nested_scopes
from checkers_game import *

cache = {}  # you can use this to implement state caching!


# Method to compute utility value of terminal state
def compute_utility(state, color):

    curr_color = 0
    curr_color_king = 0
    opp_color = 0
    opp_color_king = 0

    for row in state.board:
        for space in row:
            if space == color:
                curr_color += 1
            if space == PieceToKingDic[color]:
                curr_color_king += 1
            if space == OppDic1[color]:
                opp_color += 1
            if space == PieceToKingDic[OppDic1[color]]:
                opp_color_king += 1

    curr_color_point = curr_color_king * 2 + curr_color
    opp_color_point = opp_color_king * 2 + opp_color
    return curr_color_point - opp_color_point


# Better heuristic value of board
def compute_heuristic(state, color):
    # IMPLEMENT
    cur_board = state.board
    count = 0
    score = 0
    if color == 'r':
        for i in range(8):
            for j in range(8):
                if cur_board[i][j] == '.':
                    continue
                else:
                    if cur_board[i][j] in ['r', 'R', 'b', 'B']:
                        count += 1
                    if cur_board[i][j] == 'R':
                        score += 10
                    elif cur_board[i][j] == 'B':
                        score -= 10
                    elif cur_board[i][j] == 'r' and j < 4:
                        score += 3
                    elif cur_board[i][j] != 'r' and j < 4:
                        score -= 7
                    elif cur_board[i][j] == 'r' and j >= 4:
                        score += 7
                    elif cur_board[i][j] != 'r' and j >= 4:
                        score -= 3
    else:
        for i in range(8):
            for j in range(8):
                if cur_board[i][j] == '.':
                    continue
                else:
                    if cur_board[i][j] in ['r', 'R', 'b', 'B']:
                        count += 1
                    if cur_board[i][j] == 'B':
                        score += 10
                    elif cur_board[i][j] == 'R':
                        score -= 10
                    elif cur_board[i][j] == 'b' and j < 4:
                        score += 3
                    elif cur_board[i][j] != 'b' and j < 4:
                        score -= 7
                    elif cur_board[i][j] == 'b' and j >= 4:
                        score += 7
                    elif cur_board[i][j] != 'b' and j >= 4:
                        score -= 3

    return score / count


############ MINIMAX ###############################
def minimax_min_node(state, color, limit, caching=0):
    # IMPLEMENT
    if color == 'r':
        opponent = 'b'
    else:
        opponent = 'r'

    board = tuple(tuple(i) for i in state.board)
    if caching == 1 and (board, color) in cache.keys():
        return cache[(board, color)]

    successor = successors(state, color)
    # print("start")
    # for i in  successor:
    #     print(i.move)
    # print("end")
    if limit == 0:
        return compute_utility(state, opponent), None

    if not successor:
        if caching == 1:
            cache[(board, color)] = compute_utility(state, opponent), None
        return compute_utility(state, opponent), None

    min_utility = float("inf")
    best_move = None
    for child in successor:
        child_val, child_move = minimax_max_node(child, opponent, limit - 1,
                                                 caching)
        if child_val < min_utility:
            min_utility = child_val
            best_move = child

    if caching == 1:
        cache[(board, color)] = min_utility, tuple(tuple(i) for i in best_move.board)

    return min_utility, best_move


def minimax_max_node(state, color, limit, caching=0):
    # IMPLEMENT
    if color == 'r':
        opponent = 'b'
    else:
        opponent = 'r'

    board = tuple(tuple(i) for i in state.board)
    if caching == 1 and (board, color) in cache.keys():
        return cache[(board, color)]

    if limit == 0:
        return compute_utility(state, color), None

    successor = successors(state, color)
    if not successor:
        if caching == 1:
            cache[(board, color)] = compute_utility(state, color), None
        return compute_utility(state, color), None

    max_utility = float("-inf")
    best_move = None
    for child in successor:
        child_val, child_move = minimax_min_node(child, opponent, limit - 1,
                                                 caching)
        if child_val > max_utility:
            max_utility = child_val
            best_move = child

    if caching == 1:
        cache[(board, color)] = max_utility, tuple(tuple(i) for i in best_move.board)

    return max_utility, best_move


def select_move_minimax(state, color, limit, caching=0):
    """
        Given a state (of type Board) and a player color, decide on a move.
        The return value is a list of tuples [(i1,j1), (i2,j2)], where
        i1, j1 is the starting position of the piece to move
        and i2, j2 its destination.  Note that moves involving jumps will contain
        additional tuples.

        Note that other parameters are accepted by this function:
        If limit is a positive integer, your code should enforce a depth limit that is equal to the value of the parameter.
        Search only to nodes at a depth-limit equal to the limit.  If nodes at this level are non-terminal return a heuristic
        value (see compute_utility)
        If caching is ON (i.e. 1), use state caching to reduce the number of state evaluations.
        If caching is OFF (i.e. 0), do NOT use state caching to reduce the number of state evaluations.
    """
    # IMPLEMENT

    return minimax_max_node(state, color, limit, caching)[1].move


############ ALPHA-BETA PRUNING #####################
def alphabeta_min_node(state, color, alpha, beta, limit, caching=0, ordering=0):
    # IMPLEMENT
    if color == 'r':
        opponent = 'b'
    else:
        opponent = 'r'

    board = tuple(tuple(i) for i in state.board)
    if caching == 1 and (board, color) in cache.keys():
        return cache[(board, color)]

    successor = successors(state, color)
    if limit == 0:
        return compute_utility(state, opponent), None

    # print('start')
    # for i in successor:
    #     print(i.move)
    # print('end')
    if not successor:
        if caching == 1:
            cache[(board, color)] = compute_utility(state, opponent), None
        return compute_utility(state, opponent), None

    if ordering:
        successor.sort(key=lambda board: compute_heuristic(board, color),reverse=False)


    min_utility = float("inf")
    best_move = None
    for child in successor:
        child_val, child_move = alphabeta_max_node(child, opponent, alpha, beta,
                                                   limit - 1, caching, ordering)
        if child_val < min_utility:
            min_utility = child_val
            best_move = child
        beta = min(beta, min_utility)
        if beta <= alpha:
            break

    if caching == 1:
        cache[(board, color)] = min_utility, tuple(tuple(i) for i in best_move.board)

    return min_utility, best_move


def alphabeta_max_node(state, color, alpha, beta, limit, caching=0, ordering=0):
    # IMPLEMENT
    if color == 'r':
        opponent = 'b'
    else:
        opponent = 'r'
    successor = successors(state, color)

    board = tuple(tuple(i) for i in state.board)
    if caching == 1 and (board, color) in cache.keys():
        return cache[(board, color)]

    if limit == 0:
        return compute_utility(state, color), None

    if not successor:
        if caching == 1:
            cache[(board, color)] = compute_utility(state, color), None
        return compute_utility(state, color), None

    if ordering:
        successor.sort(key=lambda board: compute_heuristic(board, color), reverse=True)

    max_utility = float("-inf")
    best_move = None
    for child in successor:
        child_val, child_move = alphabeta_min_node(child, opponent, alpha, beta,
                                                   limit - 1, caching, ordering)
        if child_val > max_utility:
            max_utility = child_val
            best_move = child
        alpha = max(alpha, max_utility)
        if max_utility >= beta:
            break

    if caching == 1:
        cache[(board, color)] = max_utility, tuple(tuple(i) for i in best_move.board)

    return max_utility, best_move


def select_move_alphabeta(state, color, limit, caching=0, ordering=0):
    """
    Given a state (of type Board) and a player color, decide on a move.
    The return value is a list of tuples [(i1,j1), (i2,j2)], where
    i1, j1 is the starting position of the piece to move
    and i2, j2 its destination.  Note that moves involving jumps will contain
    additional tuples.

    Note that other parameters are accepted by this function:
    If limit is a positive integer, your code should enforce a depth limit that is equal to the value of the parameter.
    Search only to nodes at a depth-limit equal to the limit.  If nodes at this level are non-terminal return a heuristic
    value (see compute_utility)
    If caching is ON (i.e. 1), use state caching to reduce the number of state evaluations.
    If caching is OFF (i.e. 0), do NOT use state caching to reduce the number of state evaluations.
    If ordering is ON (i.e. 1), use node ordering to expedite pruning and reduce the number of state evaluations.
    If ordering is OFF (i.e. 0), do NOT use node ordering to expedite pruning and reduce the number of state evaluations.
    """
    # IMPLEMENT
    alpha = float("-inf")
    beta = float("inf")
    return \
        alphabeta_max_node(state, color, alpha, beta, limit, caching, ordering)[
            1].move


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
        if alphabeta:
            result = select_move_alphabeta(Board(state), PLAYER, limit, caching,
                                           ordering)
        else:
            result = select_move_minimax(Board(state), PLAYER, limit, caching)
        print(result)
        return result
