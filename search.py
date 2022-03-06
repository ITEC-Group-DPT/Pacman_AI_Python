"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""

from game import Directions
import util
n = Directions.NORTH
s = Directions.SOUTH
e = Directions.EAST
w = Directions.WEST


def depthFirstSearch(problem):
    print(problem.foodPosition)
    
    
    # TODO 17


def breadthFirstSearch(problem):
    '''
    return a path to the goal
    '''
    """Search the shallowest nodes in the search tree first."""

    #to be explored (FIFO)
    queue = util.Queue()
    
    #previously expanded states (for cycle checking), holds states
    visitedNodes = []
    
    startState = problem.getStartState()
    startNode = (startState, []) 
    
    queue.enqueue(startNode)
    
    while (queue.is_empty() == False):
        #begin exploring first (earliest-pushed) node on frontier
        currentState, actions = queue.dequeue()
        
        if currentState not in visitedNodes:
            #put popped node state into explored list
            visitedNodes.append(currentState)

            if problem.isGoalState(currentState):
                return actions
            else:
                #list of (successor, action, stepCost)
                successors = problem.getSuccessors(currentState)
                
                for succState, succAction in successors:
                    newAction = actions + [succAction]
                 
                    newNode = (succState, newAction)

                    queue.enqueue(newNode)

    return actions
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
