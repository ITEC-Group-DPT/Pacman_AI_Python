"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""

from game import Directions

n = Directions.NORTH
s = Directions.SOUTH
e = Directions.EAST
w = Directions.WEST


def depthFirstSearch(problem):
    '''
    return a path to the goal
    '''
    # TODO 17


def breadthFirstSearch(problem):
    '''
    return a path to the goal
    '''
    # TODO 18


def uniformCostSearch(problem):
    '''
    return a path to the goal
    '''
    # TODO 19


def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0


def singleFoodSearchHeuristic(state, problem=None):
    """
    A heuristic function for the problem of single food search
    """
    # TODO 20
    pass


def multiFoodSearchHeuristic(state, problem=None):
    """
    A heuristic function for the problem of multi-food search
    """
    # TODO 21
    pass


def aStarSearch(problem, heuristic=nullHeuristic):
    '''
    return a path to the goal
    '''
    # TODO 22


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
