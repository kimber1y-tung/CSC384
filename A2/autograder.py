#!/usr/bin/env python
import os  # for time functions
import checkers_game

# import student's functions
from agent import *

checkersboards = [[ ['.','.','.','.','.','.','.','.'], \
                    ['b','.','.','.','b','.','.','r'], \
                    ['.','b','.','.','.','.','.','.'], \
                    ['.','.','b','.','.','.','b','.'], \
                    ['.','.','.','r','.','.','.','.'], \
                    ['r','.','.','.','r','.','.','.'], \
                    ['.','.','.','R','.','b','.','.'], \
                    ['.','.','r','.','.','.','r','.'] ],
                  [ ['.','.','.','.','.','.','.','.'], \
                    ['b','.','.','.','B','.','.','r'], \
                    ['.','b','.','.','.','.','.','.'], \
                    ['.','.','b','.','.','.','b','.'], \
                    ['.','.','.','r','.','.','.','.'], \
                    ['r','.','.','.','r','.','.','.'], \
                    ['.','b','.','r','.','r','.','.'], \
                    ['.','.','r','.','.','.','B','.'] ],
                  [ ['.','.','.','.','.','.','.','.'], \
                    ['b','.','.','.','.','.','.','.'], \
                    ['.','.','.','.','.','.','.','.'], \
                    ['.','.','b','.','B','.','b','.'], \
                    ['.','.','.','r','.','.','.','.'], \
                    ['r','.','.','.','r','.','.','.'], \
                    ['.','.','.','r','.','r','.','.'], \
                    ['.','.','r','.','.','.','B','.'] ],
                  [ ['.','.','.','.','.','.','.','.'], \
                    ['b','.','b','.','.','.','.','.'], \
                    ['.','.','.','.','.','.','.','.'], \
                    ['.','.','B','.','B','.','b','.'], \
                    ['.','.','.','r','.','.','.','.'], \
                    ['r','.','.','.','r','.','.','.'], \
                    ['.','.','.','r','.','r','.','.'], \
                    ['.','.','r','.','.','.','B','.'] ]
                  ]

#Select what to test
test_compute_utility = False
test_minimax_min_node_1 = False
test_minimax_max_node_1 = False

test_minimax_min_node_2 = False
test_minimax_max_node_2 = False

test_alphabeta_min_node_1 = False
test_alphabeta_max_node_1 = False
test_alphabeta_min_node_2 = False
test_alphabeta_max_node_2 = False
test_caching = False
test_ordering = True

test_select_move_minimax = False
test_select_move_alphabeta = False
test_select_move_equal = False

if test_compute_utility:

    ##############################################################
    print('Testing Utility')
    correctvalues = [2, -2, -1, -3]
    correct = 0
    for i in range(0,len(checkersboards)):
        board = Board(checkersboards[i],None)
        value1 = compute_utility(board, 'r')
        value2 = compute_utility(board, 'b')
        if value1 == correctvalues[i] and value2 == correctvalues[i]*-1:
            correct+=1

    print("You computed correct utilities for {} of {} small boards".format(correct, len(correctvalues)))

if test_select_move_minimax:
    correctmoves_1 = [[(1, 7), (0, 6)],[(1, 7), (0, 6)],[(4, 3), (2, 5)],[(4, 3), (2, 1), (0, 3)]]
    correctmoves_2 = [[(6, 3), (7, 4)],[(1, 7), (0, 6)],[(4, 3), (2, 5)],[(4, 3), (2, 1), (0, 3)]]

    correct = 0
    colors = ['r','r','r','r']
    for i in range(0,len(checkersboards)):
        board = Board(checkersboards[i],None)
        value1 = select_move_minimax(board, colors[i], 2)
        #print("minimax move is {}".format(value1))
        if (value1 == correctmoves_1[i] or value1 == correctmoves_2[i]):
            correct+=1
    print('Testing Minimax (with Depth Limit of 2)')
    print("You computed correct minimax moves for {} of {} small boards".format(correct, len(correctmoves_1)))

if test_select_move_alphabeta:
    correctmoves_1 = [[(1, 7), (0, 6)],[(1, 7), (0, 6)],[(4, 3), (2, 5)],[(4, 3), (2, 1), (0, 3)]]
    correctmoves_2 = [[(6, 3), (7, 4)],[(1, 7), (0, 6)],[(4, 3), (2, 5)],[(4, 3), (2, 1), (0, 3)]]
    correct = 0
    colors = ['r','r','r','r']
    for i in range(0,len(checkersboards)):
        board = Board(checkersboards[i], None)
        value1 = select_move_alphabeta(board, colors[i], 1)
        #print("alphabeta move is {}".format(value1))
        if (value1 == correctmoves_1[i] or value1 == correctmoves_2[i]):
            correct+=1
    print('Testing Alphabeta (with Depth Limit of 1)')
    print("You computed correct alphabeta moves for {} of {} small boards".format(correct, len(correctmoves_1)))

if test_select_move_equal:
    correctmoves_1 = [[(1, 7), (0, 6)],[(1, 7), (0, 6)],[(4, 3), (2, 5)],[(4, 3), (2, 1), (0, 3)]]
    correctmoves_2 = [[(6, 3), (7, 4)],[(1, 7), (0, 6)],[(4, 3), (2, 5)],[(4, 3), (2, 1), (0, 3)]]
    correct = 0
    for i in range(0,len(checkersboards)):
        board = Board(checkersboards[i], None)
        value1_minimax = select_move_minimax(board, 'r', 2)
        value1_ab = select_move_alphabeta(board, 'r', 2)
        if (value1_minimax == value1_ab == correctmoves_1[i]) or (value1_minimax == value1_ab == correctmoves_2[i]):
            correct+=1

    print('Testing Minimax and Alphabeta Moves Equality (with Depth Limit of 2)')
    print("You computed correct moves for {} of {} tests".format(correct, len(correctmoves_1)))

if test_caching:

    print('Testing Caching')
    check_1 = 0
    check_2 = 0
    for i in range(0,len(checkersboards)):

        start_time_1 = os.times()[0]
        no_cache = select_move_alphabeta(Board(checkersboards[i],None), 'r', 8)
        end_time_1 = os.times()[0]

        start_time_2 = os.times()[0]
        with_cache = select_move_alphabeta(Board(checkersboards[i],None), 'r', 8, 1)
        end_time_2 = os.times()[0]

        if (end_time_1 - start_time_1) >= (end_time_2 - start_time_2) and with_cache is not None:
            check_1 += 1

        if (with_cache == no_cache) and with_cache is not None:
            check_2 += 1

    print("Caching improved the time of your alpha-beta for {} of {} boards".format(check_1, len(checkersboards)))
    print("Move choice with and without caching is the same for {} of {} boards".format(check_2, len(checkersboards)))
for x in range(10):
    if test_ordering:

        print('Testing Ordering')
        check_1 = 0
        check_2 = 0
        for i in range(0,len(checkersboards)):

            start_time_1 = os.times()[0]
            no_order = select_move_alphabeta(Board(checkersboards[i],None), 'r', 6, 0, 0)
            end_time_1 = os.times()[0]

            start_time_2 = os.times()[0]
            with_order = select_move_alphabeta(Board(checkersboards[i],None), 'r', 6, 0, 1)
            end_time_2 = os.times()[0]

            if (end_time_1 - start_time_1) >= (end_time_2 - start_time_2) and with_order is not None:
                check_1 += 1

            if (with_order == no_order):
                check_2 += 1

        print("Node ordering improved the time of your alpha-beta for {} of {} boards".format(check_1, len(checkersboards)))
    x+= 1

if test_ordering:

    print('Testing Ordering')
    check_1 = 0
    check_2 = 0
    for i in range(0,len(checkersboards)):

        start_time_1 = os.times()[0]
        no_order = select_move_alphabeta(Board(checkersboards[i],None), 'r', 6, 0, 0)
        end_time_1 = os.times()[0]

        start_time_2 = os.times()[0]
        with_order = select_move_alphabeta(Board(checkersboards[i],None), 'r', 6, 0, 1)
        end_time_2 = os.times()[0]

        if (end_time_1 - start_time_1) >= (end_time_2 - start_time_2) and with_order is not None:
            check_1 += 1

        if (with_order == no_order):
            check_2 += 1

    print("Node ordering improved the time of your alpha-beta for {} of {} boards".format(check_1, len(checkersboards)))


if test_alphabeta_min_node_1:

    print('Testing Alpha Beta Min Node - Player 1')
    answers = [([(6, 5), (7, 4)],1),([(6, 1), (7, 0)],-3),([(3, 4), (5, 2), (7, 4), (5, 6)],-4),([(3, 4), (5, 2), (7, 4), (5, 6)],-6)]
    correct = 0
    correctval = 0
    for i in range(0,len(checkersboards)):
        board = Board(checkersboards[i], None)

        color = 'b'
        (value, move) = alphabeta_min_node(board, color, float("-Inf"), float("Inf"), 1, 0, 0)
        if move != None: move = move.move
        answer = answers[i][0]
        answer_value = answers[i][1]

        if move is not None:
            if (answer[0] == move[0] and answer[1] == move[1]) :
                correct+=1
            if (answer_value == value or answer_value == -1*value ):
                correctval+=1

    print("You computed correct alpha-beta min moves for {} of {} boards".format(correct, len(checkersboards)))
    print("You computed correct alpha-beta min values for {} of {} boards".format(correctval, len(checkersboards)))


if test_alphabeta_min_node_2:

    print('Testing Alpha Beta Min Node - Player 2')
    answers = [([(6, 5), (7, 4)],2),([(6, 1), (7, 0)],-2),([(3, 4), (5, 2), (7, 4), (5, 6)],-4),([(3, 4), (5, 2), (7, 4), (5, 6)],-6)]
    correct = 0
    correctval = 0
    for i in range(0,len(checkersboards)):
        board = Board(checkersboards[i])

        color = 'b'
        (value, move)  = alphabeta_min_node(board, color, float("-Inf"), float("Inf"), 2, 0, 0)
        if move != None: move = move.move
        else: move = (None,None)
        answer = answers[i][0]
        answer_value = answers[i][1]

        if (answer[0] == move[0] and answer[1] == move[1]):
            correct+=1
        if (answer_value == value):
            correctval+=1

    print("You computed correct alpha-beta min moves for {} of {} boards".format(correct, len(checkersboards)))
    print("You computed correct alpha-beta min values for {} of {} boards".format(correctval, len(checkersboards)))

if test_alphabeta_max_node_1:

    print('Testing Alpha Beta Max Node - Player 1')
    answers1 = [([(1, 7), (0, 6)],3),([(1, 7), (0, 6)],-1),([(4, 3), (2, 5)],1),([(4, 3), (2, 1), (0, 3)],1)]
    answers2 = [([(6, 3), (7, 4)],3),([(1, 7), (0, 6)],-1),([(4, 3), (2, 5)],1),([(4, 3), (2, 1), (0, 3)],1)]
    correct = 0
    correctval = 0
    for i in range(0,len(checkersboards)):
        board = Board(checkersboards[i])

        color = 'r'
        (value, move) = alphabeta_max_node(board, color, float("-Inf"), float("Inf"), 1, 0, 0)
        if move != None: move = move.move

        answer1 = answers1[i][0]
        answer2 = answers2[i][0]
        answer_value = answers1[i][1]

        if move is not None:
            if (answer1[0] == move[0] and answer1[1] == move[1]) or (answer2[0] == move[0] and answer2[1] == move[1]):
                correct+=1
            if (answer_value == value or answer_value == -1*value ):
                correctval+=1

    print("You computed correct alpha-beta max moves for {} of {} boards".format(correct, len(checkersboards)))
    print("You computed correct alpha-beta max values for {} of {} boards".format(correctval, len(checkersboards)))


if test_minimax_max_node_1:

    print('Testing Minimax Max Node - Player 1')
    answers1 = [([(1, 7), (0, 6)],3),([(1, 7), (0, 6)],-1),([(4, 3), (2, 5)],1),([(4, 3), (2, 1), (0, 3)],1)]
    answers2 = [([(6, 3), (7, 4)],3),([(1, 7), (0, 6)],-1),([(4, 3), (2, 5)],1),([(4, 3), (2, 1), (0, 3)],1)]
    correct = 0
    correctval = 0

    for i in range(0,len(checkersboards)):
        board = Board(checkersboards[i])

        color = 'r'
        (value, move)  = minimax_max_node(board, color, 1, 0)
        if move != None: move = move.move

        answer1 = answers1[i][0]
        answer2 = answers2[i][0]
        answer_value = answers1[i][1]

        if move is not None:
            if (answer1[0] == move[0] and answer1[1] == move[1]) or (answer2[0] == move[0] and answer2[1] == move[1]) :
                correct+=1
            if (answer_value == value or answer_value == -1*value ):
                correctval+=1

    print("You computed correct minimax max moves for {} of {} boards".format(correct, len(checkersboards)))
    print("You computed correct minimax max values for {} of {} boards".format(correctval, len(checkersboards)))


if test_alphabeta_max_node_2:

    print('Testing Alpha Beta Max Node - Player 2')
    answers1 = [([(1, 7), (0, 6)],2),([(1, 7), (0, 6)],-2),([(4, 3), (2, 5)],1),([(4, 3), (2, 1), (0, 3)],1)]
    answers2 = [([(6, 3), (7, 4)],2),([(1, 7), (0, 6)],-2),([(4, 3), (2, 5)],1),([(4, 3), (2, 1), (0, 3)],1)]
    correctval = 0
    correct = 0

    for i in range(0,len(checkersboards)):
        board = Board(checkersboards[i])

        color = 'r'
        (value, move)  = alphabeta_max_node(board, color, float("-Inf"), float("Inf"), 2, 0, 0)
        if move != None: move = move.move

        answer1 = answers1[i][0]
        answer2 = answers2[i][0]
        answer_value = answers1[i][1]

        if move is not None:
            if (i > 1): #other boards have multiple solutions
                if (answer1[0] == move[0] and answer1[1] == move[1]) or (answer2[0] == move[0] and answer2[1] == move[1]):
                    correct+=1
            if (answer_value == value or answer_value == -1*value ):
                correctval+=1

    print("You computed correct alpha-beta max moves for {} of 2 boards".format(correct))
    print("You computed correct alpha-beta max values for {} of {} boards".format(correctval, len(checkersboards)))


if test_minimax_max_node_2:

    print('Testing Minimax Max Node - Player 2')
    answers1 = [([(1, 7), (0, 6)],2),([(1, 7), (0, 6)],-2),([(4, 3), (2, 5)],1),([(4, 3), (2, 1), (0, 3)],1)]
    answers2 = [([(6, 3), (7, 4)],2),([(1, 7), (0, 6)],-2),([(4, 3), (2, 5)],1),([(4, 3), (2, 1), (0, 3)],1)]
    correct = 0
    correctval = 0
    for i in range(0,len(checkersboards)):
        board = Board(checkersboards[i])

        color = 'r'
        (value, move) = minimax_max_node(board, color, 2, 0)
        if move != None: move = move.move

        answer1 = answers1[i][0]
        answer2 = answers2[i][0]
        answer_value = answers1[i][1]

        if move is not None:
            if (i > 1): #other boards have multiple solutions
                if (answer1[0] == move[0] and answer1[1] == move[1]) or (answer2[0] == move[0] and answer2[1] == move[1]) :
                    correct+=1
            if (answer_value == value or answer_value == -1*value ):
                correctval+=1

    print("You computed correct minimax max moves for {} of 2 boards".format(correct))
    print("You computed correct minimax max values for {} of {} boards".format(correctval, len(checkersboards)))

if test_minimax_min_node_1:
    ##############################################################
    # Must program some trees where we know cut set
    ##############################################################

    print('Testing Minimax Min Node - Player 1')
    answers = [([(6, 5), (7, 4)],1),([(6, 1), (7, 0)],-3),([(3, 4), (5, 2), (7, 4), (5, 6)],-4),([(3, 4), (5, 2), (7, 4), (5, 6)],-6)]
    correct = 0
    correctval = 0
    for i in range(0,len(checkersboards)):
        board = Board(checkersboards[i])

        color = 'b'
        (value, move) = minimax_min_node(board, color, 1, 0)
        if move != None: move = move.move

        answer = answers[i][0]
        answer_value = answers[i][1]

        if move is not None:
            if (answer[0] == move[0] and answer[1] == move[1]):
                correct+=1
            if (answer_value == value or answer_value == -1*value):
                correctval+=1

    print("You computed correct minimax min moves for {} of {} boards".format(correct, len(checkersboards)))
    print("You computed correct minimax min values for {} of {} boards".format(correctval, len(checkersboards)))

if test_minimax_min_node_2:
    ##############################################################
    # Must program some trees where we know cut set
    ##############################################################

    print('Testing Minimax Min Node - Player 2')
    answers = [([(6, 5), (7, 4)],2),([(6, 1), (7, 0)],-2),([(3, 4), (5, 2), (7, 4), (5, 6)],-4),([(3, 4), (5, 2), (7, 4), (5, 6)],-6)]
    correct = 0
    correctval = 0
    for i in range(0,len(checkersboards)):
        board = Board(checkersboards[i])

        color = 'b'
        (value, move)  = minimax_min_node(board, color, 2, 0)
        if move != None: move = move.move

        answer = answers[i][0]
        answer_value = answers[i][1]

        if move is not None:
            if (answer[0] == move[0] and answer[1] == move[1]):
                correct+=1
            if (answer_value == value or answer_value == -1*value ):
                correctval+=1

    print("You computed correct minimax min moves for {} of {} boards".format(correct, len(checkersboards)))
    print("You computed correct minimax min values for {} of {} boards".format(correctval, len(checkersboards)))

