"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""

from game import Directions
import util
import copy
n = Directions.NORTH
s = Directions.SOUTH
e = Directions.EAST
w = Directions.WEST


def depthFirstSearch(problem):

    pass

    # TODO 17


def breadthFirstSearch(problem):
    """
    return a path to the goal
    """
    """Search the shallowest nodes in the search tree first."""
    # startState = problem.getStartState()

    # #to be explored (FIFO)
    paths = []
    
    while (len(problem.foodPosition) != 0):
        queue = util.Queue()
        visitedNodes = []   
        start = (problem.getStartState(), paths)
        print(start)

        queue.enqueue(start)

        while queue.is_empty() == False:

            currentState, paths = queue.dequeue()

            if currentState not in visitedNodes:
                visitedNodes.append(currentState)
                if problem.isGoalState(currentState):
                    problem.start = currentState
                    problem.foodPosition.remove(currentState)
                    break
                else:
                    
                    successors = problem.getSuccessors(currentState)

                    for successorState, successorAction in successors:
                        successorPath = paths + [successorAction]

                        queue.enqueue((successorState,successorPath))
        if queue.is_empty(): 
            problem.foodPosition.pop(0)
    return paths
    

# same implementation as BFS because all successor cost is the same
def uniformCostSearch(problem):
    """
    return a path to the goal
    """
    # TODO 19

    paths = []

    while len(problem.foodPosition) != 0:
        pQueue = util.PriorityQueue()
        visitedNodes = []

        startNode = (problem.getStartState(), paths)
        pQueue.push(startNode, 0)

        while not pQueue.isEmpty():
            totalCost, (currentState, paths) = pQueue.pop()
            visitedNodes.append(currentState)

            if problem.isGoalState(currentState):
                problem.start = currentState
                break

            successors = problem.getSuccessors(currentState)
            for successorState, successorAction in successors:
                if successorState not in visitedNodes:
                    successorPath = paths + [successorAction]
                    successorCost = 1  # every successor has a cost of 1 in pacman
                    pQueue.push((successorState, successorPath), totalCost + successorCost)
                    print(pQueue)

        if pQueue.isEmpty():
            problem.foodPosition.pop(0)

    return paths



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
    """
    return a path to the goal
    """

    
    pass
    # TODO 22


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
