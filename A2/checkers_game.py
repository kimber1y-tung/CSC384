from __future__ import nested_scopes

#============== Class: Board ============================================
# This class is used to represent board and move
# Class Members:
# board : a list of lists that represents the 8*8 board
# move : is also a list, e.g move = [(1,1),(3,3),(5,5)]
class Board:
    def __init__(self,board,move=[]):
        self.board = board
        self.move = move

PieceToKingDic = {'r':'R', 'b':'B'}
OppDic = {'B':['r','R'],'R':['b','B'],'b':['r','R'],'r':['b','B']}
PlayerDic = {'r':['r','R'],'b':['b','B'],'R':['r','R'],'B':['b','B']}
OppDic1 = {'b':'r','r':'b'}


#This function will return move. It has not been tested and it is not used yet
def GetMoveList(cur_board,suc_board,player):
    for i in range(8):
        for j in range(8):
            if suc_board[i][j]  == '.' and cur_board[i][j] in PlayerDic[player]:
                s,r = i,j
            if cur_board[i][j]  == '.' and suc_board[i][j] in PlayerDic[player]:
                a,b = i,j

    if abs(s-a) == 1:
        move = [(s,r),(a,b)]
    else:
        move = [(s,r)]
        while s != a and r != b:
            if s >= 2 and r >= 2 and cur_board[s-1][r-1] in OppDic[player]  and suc_board[s-1][r-1] == '.':
                s,r = s-2,r-2
                move += [(s, r)]
            elif s >= 2 and r<= 5 and cur_board[s-1][r+1] in OppDic[player]  and suc_board[s-1][r+1] == '.':
                s,r = s-2,r+2
                move += [(s, r)]
            elif s <= 5 and r >= 2 and cur_board[s+1][r-1] in OppDic[player]  and suc_board[s+1][r-1] == '.':
                s,r = s+2,r-2
                move += [(s, r)]
            elif s <= 5 and r <= 5 and cur_board[s+1][r+1] in OppDic[player]  and suc_board[s+1][r+1] == '.':
                s,r = s+2,r+2
                move += [(s, r)]
    return move


def Jump(board, a,b, jstep, player):
    result = []
    if player == 'b':
        #Jump:  upper right
        if a <= 5 and b <= 5 and (board[a+1][b+1] in OppDic[player]) and board[a+2][b+2] == '.':
            new_board = copy_board(board)
            new_board[a][b], new_board[a+1][b+1] = '.','.'
            if a == 5:
                new_board[a+2][b+2] = 'B'
            else:
                new_board[a+2][b+2] = 'b'
            tlist  = Jump(new_board,a+2,b+2,jstep+1,'b')
            for state in tlist:
                state.move = [(a,b)]+ state.move
            result += tlist
        #Jump:  upper left
        if a <= 5 and b >= 2 and (board[a+1][b-1] in OppDic[player]) and board[a+2][b-2] == '.':
            new_board = copy_board(board)
            new_board[a][b], new_board[a+1][b-1] = '.','.'
            if a == 5:
                new_board[a+2][b-2] = 'B'
            else:
                new_board[a+2][b-2] = 'b'
            tlist  = Jump(new_board,a+2,b-2,jstep+1,'b')
            for state in tlist:
                state.move = [(a,b)]+ state.move
            result += tlist
        if not result and jstep >= 1:
            result = [Board(board,move = [(a,b)])]
    elif player == 'r':
        #Jump:  down right
        if a >= 2 and b <= 5 and (board[a-1][b+1] in OppDic[player]) and board[a-2][b+2] == '.':
            new_board = copy_board(board)
            new_board[a][b], new_board[a-1][b+1] = '.','.'
            if a == 2:
                new_board[a-2][b+2] = 'R'
            else:
                new_board[a-2][b+2] = 'r'
            tlist  = Jump(new_board,a-2,b+2,jstep+1,'r')
            for state in tlist:
                state.move = [(a,b)]+ state.move
            result += tlist
        #Jump:  down left
        if a >= 2 and b >= 2 and (board[a-1][b-1] in OppDic[player]) and board[a-2][b-2] == '.':
            new_board = copy_board(board)
            new_board[a][b], new_board[a-1][b-1] = '.','.'
            if a == 2:
                new_board[a-2][b-2] = 'R'
            else:
                new_board[a-2][b-2] = 'r'
            tlist  = Jump(new_board,a-2,b-2,jstep+1,'r')
            for state in tlist:
                state.move = [(a,b)]+ state.move
            result += tlist
        if not result and jstep >= 1:
            result = [Board(board,move = [(a,b)])]
    elif player == 'B' or player == 'R':
        #Jump:  upper right
        if a <= 5 and b <= 5 and (board[a+1][b+1] in OppDic[player]) and board[a+2][b+2] == '.':
            new_board = copy_board(board)
            new_board[a][b], new_board[a+1][b+1] = '.','.'
            new_board[a+2][b+2] = player
            tlist  = Jump(new_board,a+2,b+2,jstep+1,player)
            for state in tlist:
                state.move = [(a,b)]+ state.move
            result = result + tlist
        #Jump:  upper left
        if a <= 5 and b >= 2 and (board[a+1][b-1] in OppDic[player]) and board[a+2][b-2] == '.':
            new_board = copy_board(board)
            new_board[a][b], new_board[a+1][b-1] = '.','.'
            new_board[a+2][b-2] = player
            tlist  = Jump(new_board,a+2,b-2,jstep+1,player)
            for state in tlist:
                state.move = [(a,b)]+ state.move
            result = result + tlist
        #Jump: down right
        if a >= 2 and b <= 5 and (board[a-1][b+1] in OppDic[player]) and board[a-2][b+2] == '.':
            new_board = copy_board(board)
            new_board[a][b], new_board[a-1][b+1] = '.','.'
            new_board[a-2][b+2] = player
            tlist  = Jump(new_board,a-2,b+2,jstep+1,player)
            for state in tlist:
                state.move = [(a,b)]+ state.move
            result = result + tlist
        #Jump: down left
        if a >= 2 and b >= 2 and (board[a-1][b-1] in OppDic[player]) and board[a-2][b-2] == '.':
            new_board = copy_board(board)
            new_board[a][b], new_board[a-1][b-1] = '.','.'
            new_board[a-2][b-2] = player
            tlist  = Jump(new_board,a-2,b-2,jstep+1,player)
            for state in tlist:
                state.move = [(a,b)]+ state.move
            result = result + tlist
        if not result and jstep >= 1:
            result = [Board(board,move = [(a,b)])]
    return result


def copy_board(board):
    new_board = [[]]*8
    for i in range(8):
        new_board[i] = [] + board[i]
    return new_board


def successors(state,player):
    cur_board = state.board
    suc_result = []
    if player == 'b':
        #Test jump: If a piece can jump, it must jump
        piece_list = []
        for i in range(8):
            for j in range(8):
                if cur_board[i][j] == 'b' or cur_board[i][j] == 'B':
                    suc_result += Jump(cur_board, i, j, 0, cur_board[i][j])
                    piece_list += [[i, j]]
        #Move the piece one step
        if not suc_result:
            for x in piece_list:
                i,j = x[0],x[1]
                if cur_board[i][j] == 'b':
                    #(1)The piece is not in the rightmost column, move to upperright
                    if i <= 6 and j <= 6 and cur_board[i+1][j+1] == '.':
                        suc_board = copy_board(cur_board)
                        suc_board[i][j] = '.'
                        if i<=5:
                            suc_board[i+1][j+1] = 'b'
                        else:
                            suc_board[i+1][j+1] = 'B'
                        move = [(i,j),(i+1,j+1)]
                        suc_result += [Board(suc_board, move)]
                    #(2)The pice is not in the leftmost column, move to the upperleft
                    if i <= 6 and j >= 1 and cur_board[i+1][j-1] == '.':
                        suc_board = copy_board(cur_board)
                        suc_board[i][j] = '.'
                        if i<= 5:
                            suc_board[i+1][j-1] = 'b'
                        else:
                            suc_board[i+1][j-1] = 'B'
                        move = [(i,j),(i+1,j-1)]
                        suc_result += [Board(suc_board, move)]
                elif cur_board[i][j] == 'B':
                    #Move the king one step
                    #(1)The king is not in top and the rightmost column, move to upperright
                    if i <= 6 and j <= 6 and cur_board[i+1][j+1] == '.':
                        suc_board = copy_board(cur_board)
                        suc_board[i][j] = '.'
                        suc_board[i+1][j+1] = 'B'
                        move = [(i,j),(i+1,j+1)]
                        suc_result += [Board(suc_board, move)]
                    #(2)The king is not in top and the leftmost column, move to the upperleft
                    if i <= 6 and j >= 1 and cur_board[i+1][j-1] == '.':
                        suc_board = copy_board(cur_board)
                        suc_board[i][j] = '.'
                        suc_board[i+1][j-1] = 'B'
                        move = [(i,j),(i+1,j-1)]
                        suc_result += [Board(suc_board, move)]
                    #(3)The king is not in bottom and the rightmost column, move to the downright
                    if i >= 1 and j <= 6 and cur_board[i-1][j+1] == '.':
                        suc_board = copy_board(cur_board)
                        suc_board[i][j] = '.'
                        suc_board[i-1][j+1] = 'B'
                        move = [(i,j),(i-1,j+1)]
                        suc_result += [Board(suc_board, move)]
                    #(4)The king is not in the leftmost column, move to the downleft
                    if i >= 1 and j >= 1 and cur_board[i-1][j-1] == '.':
                        suc_board = copy_board(cur_board)
                        suc_board[i][j] = '.'
                        suc_board[i-1][j-1] = 'B'
                        move = [(i,j),(i-1,j-1)]
                        suc_result += [Board(suc_board, move)]
    else:
        #Test jump: If a piece can jump, it must jump
        piece_list = []
        for i in range(8):
            for j in range(8):
                if cur_board[i][j] == 'r' or cur_board[i][j] == 'R':
                    suc_result += Jump(cur_board, i, j, 0, cur_board[i][j])
                    piece_list += [[i, j]]
        #If jump is not available, move the piece one step
        if not suc_result:
            for x in piece_list:
                i,j = x[0],x[1]
                if cur_board[i][j] == 'r':
                    #move the piece one step
                    #(1)the piece is not in the rightmost column, move to downright
                    if j <= 6 and cur_board[i-1][j+1] == '.':
                        suc_board = copy_board(cur_board)
                        suc_board[i][j] = '.'
                        if i >= 2:
                            suc_board[i-1][j+1] = 'r'
                        else:
                            suc_board[i-1][j+1] = 'R'
                        move = [(i,j),(i-1,j+1)]
                        suc_result += [Board(suc_board, move)]
                    #(2)the pice is not in the leftmost column, move to the upperleft
                    if j >= 1 and cur_board[i-1][j-1] == '.':
                        suc_board = copy_board(cur_board)
                        suc_board[i][j] = '.'
                        if i >= 2:
                            suc_board[i-1][j-1] = 'r'
                        else:
                            suc_board[i-1][j-1] = 'R'
                        move = [(i,j),(i-1,j-1)]
                        suc_result += [Board(suc_board, move)]
                elif cur_board[i][j] == 'R':
                    #move the king one step
                    #(1)the king is not in top and the rightmost column, move to upperright
                    if i <= 6 and j <= 6 and cur_board[i+1][j+1] == '.':
                        suc_board = copy_board(cur_board)
                        suc_board[i][j] = '.'
                        suc_board[i+1][j+1] = 'R'
                        move = [(i,j),(i+1,j+1)]
                        suc_result += [Board(suc_board, move)]
                    #(2)the king is not in top and the leftmost column, move to the upperleft
                    if i <= 6 and j >= 1 and cur_board[i+1][j-1] == '.':
                        suc_board = copy_board(cur_board)
                        suc_board[i][j] = '.'
                        suc_board[i+1][j-1] = 'R'
                        move = [(i,j),(i+1,j-1)]
                        suc_result += [Board(suc_board, move)]
                    #(3)the king is not in bottom and the rightmost column, move to the downright
                    if i >= 1 and j <= 6 and cur_board[i-1][j+1] == '.':
                        suc_board = copy_board(cur_board)
                        suc_board[i][j] = '.'
                        suc_board[i-1][j+1] = 'R'
                        move = [(i,j),(i-1,j+1)]
                        suc_result += [Board(suc_board, move)]
                    #(4)the king is not in the leftmost column, move to the upperleft
                    if i >= 1 and j >= 1 and cur_board[i-1][j-1] == '.':
                        suc_board = copy_board(cur_board)
                        suc_board[i][j] = '.'
                        suc_board[i-1][j-1] = 'R'
                        move = [(i,j),(i-1,j-1)]
                        suc_result += [Board(suc_board, move)]
    return suc_result

