# Look for #IMPLEMENT tags in this file.
'''
All models need to return a CSP object, and a list of lists of Variable objects
representing the board. The returned list of lists is used to access the
solution.

For example, after these three lines of code

    csp, var_array = futoshiki_csp_model_1(board)
    solver = BT(csp)
    solver.bt_search(prop_FC, var_ord)

var_array[0][0].get_assigned_value() should be the correct value in the top left
cell of the Futoshiki puzzle.

1. futoshiki_csp_model_1 (worth 20/100 marks)
    - A model of a Futoshiki grid built using only
      binary not-equal constraints for both the row and column constraints.

2. futoshiki_csp_model_2 (worth 20/100 marks)
    - A model of a Futoshiki grid built using only n-ary
      all-different constraints for both the row and column constraints.

'''
from cspbase import *
import itertools

def futo_variable_helper(futo_grid):
    """
    creates a new csp with variables assigned to it
    returns the csp and a list of futo grids variables and inequalty
    """
    futo_grid_var = []
    futo_grid_inq = []
    all_var = []
    domain = []
    for i in range(1, len(futo_grid) + 1):
        domain.append(i)
    # print(domain)

    # variables
    for i in range(len(futo_grid)):
        row_var = []
        row_inq = []
        for j in range(len(futo_grid[0])):
            # if the variable is 0 (no value assigned)
            if futo_grid[i][j] == 0:
                var = Variable('V{}{}'.format(i, j//2), domain)
                row_var.append(var)
                all_var.append(var)
            # variable has a number assigned (!= 0)
            elif (type(futo_grid[i][j]) == int) & (futo_grid[i][j] != 0):
                var = Variable('V{}{}'.format(i, j//2), [futo_grid[i][j]])
                var.assign(futo_grid[i][j])
                row_var.append(var)
                all_var.append(var)
            elif futo_grid[i][j] in ['.', '<', '>']:
                row_inq.append(futo_grid[i][j])
        futo_grid_var.append(row_var)
        futo_grid_inq.append(row_inq)


    csp = CSP('{}-Futoshiki_model_1'.format(len(futo_grid)) , all_var)
    # print('futo grid var is', futo_grid_var)
    # print('futo grid inq is', futo_grid_inq)
    return csp, futo_grid_var, futo_grid_inq


def futoshiki_csp_model_1(futo_grid):
    """
    futo_grid = [[1,'<',0,'.',0],
                 [0,'.',0,'.',2],
                 [2,'.',0,'>',0]]
    """
    ##IMPLEMENT
    # futo_grid = [[1,'<',0,'.',0],[0,'.',0,'.',2],[2,'.',0,'>',0]]

    # variables
    csp, futo_grid_var, futo_grid_inq = futo_variable_helper(futo_grid)

    # constraints
    cons = []
    for i in range(len((futo_grid))):
        for j in range(len(futo_grid)):
            adj_j = j + 1
            # check while row variable is in range
            while adj_j < len(futo_grid):
                cur_var = futo_grid_var[i][j]
                adj_row_var = futo_grid_var[i][adj_j]
                con = Constraint('C(V{}{},V{}{})'.format(i, j, i, adj_j),
                                 [cur_var, adj_row_var])
                cur_dom = cur_var.domain()
                adj_row_dom = adj_row_var.domain()
                satisfied_tuples = []
                # print('this is cur var',cur_var)
                # print('this is cur dom', cur_dom)
                # print('this is adj var', adj_row_var)
                # print('this is adj var dom', adj_row_dom)
                # print('this is inq', futo_grid_inq[i][j])
                for x in itertools.product(cur_dom, adj_row_dom):
                    if (futo_grid_inq[i][j] == '.') & (x[0] != x[1]):
                            satisfied_tuples.append(x)
                    elif (futo_grid_inq[i][j] == '<') & (x[0] < x[1]):
                        satisfied_tuples.append(x)
                    elif(futo_grid_inq[i][j] == '>') & (x[0] > x[1]):
                        satisfied_tuples.append(x)
                con.add_satisfying_tuples(satisfied_tuples)
                cons.append(con)

                # columns
                cur_var = futo_grid_var[j][i]
                adj_col_var = futo_grid_var[adj_j][i]
                con = Constraint('C(V{}{},V{}{})'.format(j, i, adj_j, i),
                                 [cur_var, adj_col_var])
                cur_dom = cur_var.domain()
                adj_col_dom = adj_col_var.domain()
                satisfied_tuples = []
                for x in itertools.product(cur_dom, adj_col_dom):
                    if(x[0] != x[1]):
                        satisfied_tuples.append(x)
                con.add_satisfying_tuples(satisfied_tuples)
                cons.append(con)
                adj_j += 1

    #csp.cons = cons
    for c in cons:
        csp.add_constraint(c)
        # print(c, c.sat_tuples)
    return csp, futo_grid_var


def futoshiki_csp_model_2(futo_grid):
    ##IMPLEMENT
    # variables
    csp, futo_grid_var, futo_grid_inq = futo_variable_helper(futo_grid)

    # constraints
    cons = []
    for i in range(len(futo_grid)):
        col_var = []
        col_dom = []
        row_dom = []
        for j in range(len(futo_grid)):
            col_var.append(futo_grid_var[j][i])
            col_dom.append(futo_grid_var[j][i].cur_domain())
            row_dom.append(futo_grid_var[i][j].cur_domain())
            adj_j = j + 1
            # check neighbor only
            if adj_j < len(futo_grid):
                cur_var = futo_grid_var[i][j]
                adj_row_var = futo_grid_var[i][adj_j]
                # con = Constraint('C(V{}{},V{}{})'.format(i, j, i, adj_j),
                #                  [cur_var, adj_row_var])
                con = None
                cur_dom = cur_var.domain()
                adj_row_dom = adj_row_var.domain()
                satisfied_tuples = []
                for x in itertools.product(cur_dom, adj_row_dom):
                    if (futo_grid_inq[i][j] == '<') & (x[0] < x[1]):
                        satisfied_tuples.append(x)
                        con = Constraint('C(V{}{},V{}{})'.format(i, j, i, adj_j),
                                         [cur_var, adj_row_var])
                    elif(futo_grid_inq[i][j] == '>') & (x[0] > x[1]):
                        satisfied_tuples.append(x)
                        con = Constraint('C(V{}{},V{}{})'.format(i, j, i, adj_j),
                                         [cur_var, adj_row_var])
                if con:
                    con.add_satisfying_tuples(satisfied_tuples)
                    cons.append(con)

        # n-ary cons
        # row
        satisfied_tuples = []
        row_con = Constraint('C(ROW{})'.format(i), futo_grid_var[i])
        for x in itertools.product(*row_dom):
            # check if variables is tuple are distinct
            # reference: https://www.geeksforgeeks.org/python-test-if-tuple-is-distinct/
            if len(set(x)) == len(x):
                satisfied_tuples.append(x)
        row_con.add_satisfying_tuples(satisfied_tuples)
        cons.append(row_con)

        # col
        satisfied_tuples = []
        col_con =  Constraint('C(COL{})'.format(i) , col_var)
        for x in itertools.product(*col_dom):
            if len(set(x)) == len(x):
                satisfied_tuples.append(x)
        col_con.add_satisfying_tuples(satisfied_tuples)
        cons.append(col_con)

    #csp.cons = cons
    for c in cons:
        csp.add_constraint(c)
        # print(c, c.sat_tuples)
    return csp, futo_grid_var


# if __name__ == "__main__":
#
#     board_1 = [[1, '<', 0, '.', 0], [0, '.', 0, '.', 2], [2, '.', 0, '>', 0]]
#     # var_array = [[v1 , v2 ,v3] , [v4 , v5 , v6] , [v7 , v8 , v9]]
#
#     board_2 = [[1, '>', 0, '.', 3], [0, '.', 0, '.', 0], [3, '<', 0, '.', 1]]
#
#     score = 1
#     # 1st model test
#     csp, var_array = futoshiki_csp_model_1(board_1)
