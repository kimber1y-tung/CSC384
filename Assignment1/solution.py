#   Look for #IMPLEMENT tags in this file. These tags indicate what has
#   to be implemented to complete the warehouse domain.

#   You may add only standard python imports---i.e., ones that are automatically
#   available on TEACH.CS
#   You may not remove any imports.
#   You may not import or otherwise source any of your own files

# python3 autograder.py
import math
import os  # for time functions
from search import *  # for search engines
from sokoban import SokobanState, Direction, \
    PROBLEMS  # for Sokoban specific classes and problems
import csv


def sokoban_goal_state(state):
    """
    @return: Whether all boxes are stored.
    """
    for box in state.boxes:
        if box not in state.storage:
            return False
    return True


def heur_manhattan_distance(state):
    # IMPLEMENT
    '''admissible sokoban puzzle heuristic: manhattan distance'''
    '''INPUT: a sokoban state'''
    '''OUTPUT: a numeric value that serves as an estimate of the distance of 
                the state to the goal.'''
    # We want an admissible heuristic, which is an optimistic heuristic.
    # It must never overestimate the cost to get from the current state to the goal.
    # The sum of the Manhattan distances between each box that has yet to be
    # stored and the storage point nearest to it is such a heuristic.
    # When calculating distances, assume there are no obstacles on the grid.
    # You should implement this heuristic function exactly, even if it is tempting to improve it.
    # Your function should return a numeric value; this is the estimate of the distance to the goal.
    sum_distance = 0
    for box in state.boxes:
        min_distance = float("inf")
        for storage in state.storage:
            curr_distance = abs(storage[0] - box[0]) + abs(storage[1] - box[1])
            if curr_distance < min_distance:
                # if current distance is smaller, we update previous
                min_distance = curr_distance
        sum_distance += min_distance  # add the min. distance to the total
    return sum_distance


# SOKOBAN HEURISTICS
def trivial_heuristic(state):
    '''trivial admissible sokoban heuristic'''
    '''INPUT: a sokoban state'''
    '''OUTPUT: a numeric value that serves as an estimate of the distance of 
    the state (# of moves required to get) to the goal.'''
    count = 0
    for box in state.boxes:
        if box not in state.storage:
            count += 1
    return count


def possible_storages(state, box):
    """
    return a list of possible storage location for the input box
    if the box is already in a storage, return [location of the box]
    """
    possible_storages = []

    # get all locations
    for storage in state.storage:
        possible_storages.append(storage)

    # if box already in a storage
    if box in possible_storages:
        return [box]

    # get only empty storages
    for other_box in state.boxes:
        if other_box != box:
            if other_box in possible_storages:
                possible_storages.remove(other_box)
    return possible_storages


def deadlock_corners(state, box):
    """
    return true if a box is in a deadlock corner (check 4 corners and obstacle made corners)
    also if a box is on an edge and beside obstacles, it is also considered as an deadlock corner
    """
    obstacles = state.obstacles | state.boxes
    # obstacles = state.obstacles | state.boxes | state.robots

    # directions
    UP = Direction("up", (0, -1))
    up = UP.move(box)
    RIGHT = Direction("right", (1, 0))
    right = RIGHT.move(box)
    DOWN = Direction("down", (0, 1))
    down = DOWN.move(box)
    LEFT = Direction("left", (-1, 0))
    left = LEFT.move(box)

    # left side
    if box[0] == 0:
        # left up and left down corners
        if box[1] == 0 or box[1] == state.height - 1:
            return True
        # on left side edge and has obstacles above or below it
        if up in obstacles or down in obstacles:
            return True

    # right side
    if box[0] == state.width - 1:
        # right up and right down corners
        if box[1] == 0 or box[1] == state.height - 1:
            return True
        # on right side edge and has obstacles above or below it
        if up in obstacles or down in obstacles:
            return True

    # obstacles surrounding box making a deadlock corner
    if up in obstacles:
        if left in obstacles or right in obstacles:
            return True
    if down in obstacles:
        if left in obstacles or right in obstacles:
            return True

    return False


def deadlock_edge(state, box):
    """
    if box is on an edge and there is no empty storage on the same edge
    """
    possible_storage = possible_storages(state, box)
    storage_x = []
    storage_y = []
    # get all of the x and y coordinates of possible storages
    for storage in possible_storage:
        storage_x.append(storage[0])
        storage_y.append(storage[1])

    # box is on left edge
    if box[0] == 0:
        if not any(i == 0 for i in storage_x):
            return True

    # box is on right edge
    if box[0] == state.width - 1:
        if not any(i == state.width - 1 for i in storage_x):
            return True

    # box is on upper edge
    if box[1] == 0:
        if not any(i == 0 for i in storage_y):
            return True

    # box in on lower edge
    if box[1] == state.height - 1:
        if not any(i == state.height - 1 for i in storage_y):
            return True

    return False


def deadlock_state(state):
    if state.boxes == None or state.storage == None or state.robots == None:
        return True

    for box in state.boxes:
        possible_storages = state.storage
        if box not in possible_storages:
            # check if box is in corners or has obstacles beside it making it impossible to push
            if deadlock_corners(state, box):
                return True
            # check if box is on an edge and there are no possible storages on the same edge
            if deadlock_edge(state, box):
                return True
    return False


def obstacle_count1(state, robot, box):
    """
    return the total count of obstacles for robot_to_box
    other robots and other boxes are also obstacles
    """
    total_count = 0
    robots = set()
    for other_robots in state.robots:  # other boxes
        if other_robots != robot:
            robots.add(other_robots)
    robots = frozenset(robots)

    boxes = set()
    for other_box in state.boxes:  # other robots
        if box != other_box:
            boxes.add(box)
    boxes = frozenset(boxes)

    obstacles = state.obstacles | robots | boxes # remove boxes?

    for obstacle in obstacles:  # larger scale?
        if max(box[0], robot[0]) > obstacle[0] > min(box[0], robot[0]):
            if max(box[1], robot[1]) > obstacle[1] > min(box[1], robot[1]):
                total_count += 1

    return total_count


def obstacle_count2(state, box, storage):
    """
    return the total count of obstacles for box_to_storage
    robots and other boxes are also obstacles
    """
    total_count = 0
    boxes = set()
    for other_box in state.boxes:  # other boxes
        if box != other_box:
            boxes.add(box)
    boxes = frozenset(boxes)
    robots = frozenset(state.robots)
    obstacles = state.obstacles | robots | boxes

    for obstacle in obstacles:  # Try larger scale
        if max(box[0], storage[0]) > obstacle[0] > min(box[0], storage[0]):
            if max(box[1], storage[1]) > obstacle[1] > min(box[1], storage[1]):
                total_count += 1

    return total_count


'''
Parameters in box_to_storage() and robot_to_box()

# Set 1 -> [12 ,12, 14] 
  theta_box_storage = 1.5 , lamb_box_storage = 1.5 
  theta_robot_box = 1 , lamb_robot_box = 3 
  
# Set 2 -> [14 ,14, 16] best
  theta_box_storage = 1 , lamb_box_storage = 1.5 
  theta_robot_box = 1 , lamb_robot_box = 3 
'''


def robot_to_box(state):
    total_cost = 0
    theta = 1
    lamb = 3

    for robot in state.robots:
        min_cost = float("inf")
        for box in state.boxes:
            current_cost = math.sqrt((robot[0] - box[0]) ** 2 + (
                        robot[1] - box[1]) ** 2) * theta + obstacle_count1(
                                                    state, robot, box) * lamb

            # want to find the smallest cost
            if current_cost < min_cost:
                # update smallest
                min_cost = current_cost
        total_cost += min_cost

    return total_cost


def box_to_storage(state):
    total_cost = 0
    theta = 1
    lamb = 1.5

    for box in state.boxes:
        possible_storage = possible_storages(state, box)
        min_cost = float("inf")
        for storage in possible_storage:
            current_cost = (abs(box[0] - storage[0]) + abs(
                box[1] - storage[1])) * theta + obstacle_count2(state, box,
                                                                storage) * lamb
            # want to find the smallest cost
            if current_cost < min_cost:
                # update smallest
                min_cost = current_cost
        total_cost += min_cost

    return total_cost


def heur_alternate(state):
    # IMPLEMENT
    '''a better heuristic'''
    '''INPUT: a sokoban state'''
    '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''
    # heur_manhattan_distance has flaws.
    # Write a heuristic function that improves upon heur_manhattan_distance to estimate distance between the current state and the goal.
    # Your function should return a numeric value for the estimate of the distance to the goal.
    if deadlock_state(state):
        return float("inf")
    return 0.5 * robot_to_box(state) + 0.5 * box_to_storage(state)


def heur_zero(state):
    '''Zero Heuristic can be used to make A* search perform uniform cost search'''
    return 0


def fval_function(sN, weight):
    # IMPLEMENT
    """
    Provide a custom formula for f-value computation for Anytime Weighted A star.
    Returns the fval of the state contained in the sNode.
    Use this function stub to encode the standard form of weighted A* (i.e. g + w*h)

    @param sNode sN: A search node (containing a SokobanState)
    @param float weight: Weight given by Anytime Weighted A star
    @rtype: float
    """

    # Many searches will explore nodes (or states) that are ordered by their f-value.
    # For UCS, the fvalue is the same as the gval of the state.
    # For best-first search, the fvalue is the hval of the state.
    # You can use this function to create an alternate f-value for states;
    # this must be a function of the state and the weight.
    # The function must return a numeric f-value.
    # The value will determine your state's position on the Frontier list
    # during a 'custom' search.
    # You must initialize your search engine object as a 'custom' search engine
    # if you supply a custom fval function.
    return sN.gval + weight * sN.hval


def fval_function_XUP(sN, weight):
    # IMPLEMENT
    """
    Another custom formula for f-value computation for Anytime Weighted A star.
    Returns the fval of the state contained in the sNode.
    Use this function stub to encode the XUP form of weighted A*

    @param sNode sN: A search node (containing a SokobanState)
    @param float weight: Weight given by Anytime Weighted A star
    @rtype: float
    """
    return 1 / (weight * 2) * (sN.gval + sN.hval + math.sqrt(
        (sN.gval + sN.hval) ** 2 + 4 * weight * (weight - 1) * (sN.hval) ** 2))


def fval_function_XDP(sN, weight):
    # IMPLEMENT
    """
    A third custom formula for f-value computation for Anytime Weighted A star.
    Returns the fval of the state contained in the sNode.
    Use this function stub to encode the XDP form of weighted A*

    @param sNode sN: A search node (containing a SokobanState)
    @param float weight: Weight given by Anytime Weighted A star
    @rtype: float
    """
    return 1 / (weight * 2) * (sN.gval + (weight * 2 - 1) * sN.hval + math.sqrt(
        (sN.gval - sN.hval) ** 2 + weight * 4 * sN.gval * sN.hval))


def compare_weighted_astars():
    # IMPLEMENT
    '''Compares various different implementations of A* that use different f-value functions'''
    '''INPUT: None'''
    '''OUTPUT: None'''
    """
    This function should generate a CSV file (comparison.csv) that contains statistics from
    4 varieties of A* search on 3 practice problems.  The four varieties of A* are as follows:
    Standard A* (Variant #1), Weighted A*  (Variant #2),  Weighted A* XUP (Variant #3) and Weighted A* XDP  (Variant #4).  
    Format each line in your your output CSV file as follows:

    A,B,C,D,E,F

    where
    A is the number of the problem being solved (0,1 or 2)
    B is the A* variant being used (1,2,3 or 4)
    C is the weight being used (2,3,4 or 5)
    D is the number of paths extracted from the Frontier (or expanded) during the search
    E is the number of paths generated by the successor function during the search
    F is the overall solution cost    

    Note that you will submit your CSV file (comparison.csv) with your code
    """
    with open('comparison.csv', 'w', newline='') as comparison:
        writer = csv.writer(comparison, delimiter=',')
        writer.writerow(["Problem", "A* Variant", "Weight", "Extracted Paths",
                         "Generated Paths", "Overall Solution Cost"])
        for i in range(0, 3):
            problem = PROBLEMS[i]
            for weight in [2, 3, 4, 5]:
                # you can write code in here if you like
                # Standard A*
                se = SearchEngine('astar', 'full')
                se.init_search(problem, goal_fn=sokoban_goal_state,
                               heur_fn=heur_manhattan_distance)
                final, stats = se.search(timebound=5)
                writer.writerow([i, 1, weight, stats.states_expanded,
                                 stats.states_generated, final.gval])
                # Weighted A*
                se = SearchEngine('custom', 'full')
                se.init_search(problem, goal_fn=sokoban_goal_state,
                               heur_fn=heur_manhattan_distance,
                               fval_function=lambda sN: fval_function(sN,
                                                                      weight))
                final, stats = se.search(timebound=5)
                writer.writerow([i, 2, weight, stats.states_expanded,
                                 stats.states_generated, final.gval])
                # Weighted A* XUP
                se.init_search(problem, goal_fn=sokoban_goal_state,
                               heur_fn=heur_manhattan_distance,
                               fval_function=lambda sN: fval_function_XUP(sN,
                                                                          weight))
                final, stats = se.search(timebound=5)
                writer.writerow([i, 3, weight, stats.states_expanded,
                                 stats.states_generated, final.gval])
                # Weighted A* XDP
                se.init_search(problem, goal_fn=sokoban_goal_state,
                               heur_fn=heur_manhattan_distance,
                               fval_function=lambda sN: fval_function_XDP(sN,
                                                                          weight))
                final, stats = se.search(timebound=5)
                writer.writerow([i, 4, weight, stats.states_expanded,
                                 stats.states_generated, final.gval])


def anytime_weighted_astar(initial_state, heur_fn, weight=1., timebound=10):
    # IMPLEMENT
    '''Provides an implementation of anytime weighted a-star, as described in the HW1 handout'''
    '''INPUT: a sokoban state that represents the start state and a timebound (number of seconds)'''
    '''OUTPUT: A goal state (if a goal is found), else False'''
    '''implementation of weighted astar algorithm'''

    start_time = os.times()[0]
    remain_time = timebound
    se = SearchEngine('custom', 'full')
    se.init_search(initial_state, goal_fn=sokoban_goal_state, heur_fn=heur_fn,
                   fval_function=lambda sN: fval_function(sN, weight))

    # first iteration
    best_f = float("inf")
    path, stats = se.search(remain_time, (
    float("inf"), float("inf"), best_f))  # (gval, hval, fval)
    remain_time = timebound - (os.times()[0] - start_time)
    if path:
        best_f = path.gval + heur_fn(path)

    # more searching
    while remain_time > 0 and not se.open.empty():
        more_path, stats = se.search(remain_time,
                                     (float("inf"), float("inf"), best_f))
        if more_path:
            best_f = more_path.gval + heur_fn(more_path)
            path = more_path
        remain_time = timebound - (os.times()[0] - start_time)

    if path:
        return path
    else:
        return False


def anytime_gbfs(initial_state, heur_fn, timebound=10):
    # IMPLEMENT
    '''Provides an implementation of anytime greedy best-first search,
        as described in the HW1 handout'''
    '''INPUT: a sokoban state that represents the start state and a timebound 
            (number of seconds)'''
    '''OUTPUT: A goal state (if a goal is found), else False'''
    '''implementation of anytime greedy best-first search'''

    start_time = os.times()[0]
    remain_time = timebound
    se = SearchEngine('best_first', 'full')
    se.init_search(initial_state, goal_fn=sokoban_goal_state, heur_fn=heur_fn)

    # first iteration
    best_g = float("inf")
    path, stats = se.search(remain_time, (
    best_g, float("inf"), float("inf")))  # (gval, hval, fval)
    remain_time = timebound - (os.times()[0] - start_time)
    if path:
        best_g = path.gval

    # more searching
    while remain_time > 0 and not se.open.empty():
        more_path, stats = se.search(remain_time,
                                     (best_g, float("inf"), float("inf")))
        if more_path:
            best_g = more_path.gval
            path = more_path
        remain_time = timebound - (os.times()[0] - start_time)

    if path:
        return path
    else:
        return False
