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
    

# same implementation as BFS because all successors' cost are the same
def uniformCostSearch(problem):
    """
    return a path to the goal
    """
    # TODO 19

    paths = []  # list of actions from initial state

    while len(problem.foodPosition) != 0:
        pQueue = util.PriorityQueue()
        visitedNodes = []
        # a state is a position (x, y)
        # a node contains its state (position) and the path from initial node
        # assign the starting node with its position and paths from initial node
        startNode = (problem.getStartState(), paths)
        pQueue.push(startNode, 0)  # push it into the priority queue

        while not pQueue.isEmpty():  # loop until pQueue is empty
            # unpack the popped node, totalCost is the total cost from initial state to current state
            totalCost, (currentState, paths) = pQueue.pop()
            visitedNodes.append(currentState)  # mark the current state (x,y) as visited

            if problem.isGoalState(currentState):  # check whether the current state (x,y) is goal state (food position)
                problem.start = currentState  # assign the new starting state as current state
                problem.foodPosition.remove(currentState)  # remove the food from the food list (already eaten)
                break  # start UCS again with new starting node as the current food position

            successors = problem.getSuccessors(currentState)  # get current node's successors
            for successorState, successorAction in successors:  # for each successor
                if successorState not in visitedNodes:  # check whether successor is visited
                    successorPath = paths + [successorAction]  # add current path with a new successor's actions
                    successorNode = (successorState, successorPath)  # new successor node

                    newTotalCost = totalCost + 1  # in pacman, every successor has a cost of 1
                    pQueue.push(successorNode, newTotalCost)  # push the new successor in pQueue

        # check whether queue is empty means all node has been traversed
        # this happens when some food can't be reached by pacman
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
