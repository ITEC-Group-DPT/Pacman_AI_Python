"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""
import problems
from game import Directions
import util
import copy

n = Directions.NORTH
s = Directions.SOUTH
e = Directions.EAST
w = Directions.WEST


def depthFirstSearch(problem):
    paths = []
    
    for i in range(len(problem.foodPosition)):
        stack = util.Stack()
        visitedNodes = []   
        startNode = (problem.getStartState(), paths)
        stack.push(startNode)

        while not stack.is_empty():
            currentState, paths = stack.pop()
            visitedNodes.append(currentState)

            if problem.isGoalState(currentState):
                problem.start = currentState
                problem.foodPosition.remove(currentState)
                break

            if problem.isGoalState(currentState):
                return paths

            successors = problem.getSuccessors(currentState)
            for successorState, successorAction in successors:
                if successorState not in visitedNodes:
                    successorPath = paths + [successorAction]
                    stack.push((successorState, successorPath))
                else:
                    # happens when all nodes have been traversed
                    # update the new start node as the current state
                    # if you don't update, the next for loop will use the previous starting node
                    # therefore high chance of leading to illegal actions
                    problem.start = currentState
    return paths


def breadthFirstSearch(problem):
    """
    return a path to the goal
    """
    """Search the shallowest nodes in the search tree first."""
    # startState = problem.getStartState()

    # #to be explored (FIFO)
    paths = []

    for i in range(len(problem.foodPosition)):
        queue = util.Queue()
        visitedNodes = []
        startNode = (problem.getStartState(), paths)
        queue.enqueue(startNode)

        while not queue.is_empty():
            currentState, paths = queue.dequeue()
            visitedNodes.append(currentState)

            if currentState in problem.foodPosition:
                problem.start = currentState
                problem.foodPosition.remove(currentState)
                break

            if problem.isGoalState(currentState):
                return paths

            successors = problem.getSuccessors(currentState)
            for successorState, successorAction in successors:
                if successorState not in visitedNodes:
                    successorPath = paths + [successorAction]
                    queue.enqueue((successorState, successorPath))
                else:
                    # happens when all nodes have been traversed
                    # update the new start node as the current state
                    # if you don't update, the next for loop will use the previous starting node
                    # therefore high chance of leading to illegal actions
                    problem.start = currentState

    return paths


# same implementation as BFS because all successors' cost are the same
def uniformCostSearch(problem):
    """
    return a path to the goal
    """
    # TODO 19

    paths = []  # list of actions from initial state

    for i in range(len(problem.foodPosition)):
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

            if currentState in problem.foodPosition:  # check whether the current state (x,y) is food state
                problem.start = currentState  # assign the new starting state as current state
                problem.foodPosition.remove(currentState)  # remove the food from the food list (already eaten)
                break  # start UCS again with new starting node as the current food position

            if problem.isGoalState(currentState):
                return paths

            successors = problem.getSuccessors(currentState)  # get current node's successors
            for successorState, successorAction in successors:  # for each successor
                if successorState not in visitedNodes:  # check whether successor is visited
                    successorPath = paths + [successorAction]  # add current path with a new successor's actions
                    successorNode = (successorState, successorPath)  # new successor node

                    newTotalCost = totalCost + 1  # in pacman, every successor has a cost of 1
                    pQueue.update(successorNode, newTotalCost)  # push the new successor in pQueue
                else:
                    # happens when all nodes have been traversed
                    # update the new start node as the current state
                    # if you don't update, the next for loop will use the previous starting node
                    # therefore high chance of leading to illegal actions
                    problem.start = currentState

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
    return util.manhattanDistance(state, problem.foodPosition[0])
    # TODO 20
    pass


def multiFoodSearchHeuristic(state, problem=None):
    """
    A heuristic function for the problem of multi-food search
    """
    agentToFood = []
    foodToFood = []

    for food in problem.foodPosition:
        agentToFood.append((food, getMazeDistance(state, food, problem)))
        for food2 in problem.foodPosition:
            if food != food2:
                foodToFood.append((food, getMazeDistance(food, food2, problem)))

    heuristic = []
    for food, agentDistance in agentToFood:
        currentFood = []
        for food2, foodDistance in foodToFood:
            if food == food2:
                currentFood.append(foodDistance)

        minVar = min(currentFood)
        heuristic.append((minVar + agentDistance, food))

    return min(heuristic)

    # TODO 21


def aStarSearch(problem, heuristic=nullHeuristic):
    # singleFoodSearchHeuristic(problem.getStartState(), problem)

    start = multiFoodSearchHeuristic(problem.getStartState(), problem)
    """
    return a path to the goal
    """

    pass
    # TODO 22


def bfs2(problem):
    """Search the shallowest nodes in the search tree first."""

    # to be explored (FIFO)
    frontier = util.Queue()

    # previously expanded states (for cycle checking), holds states
    exploredNodes = []

    startState = problem.getStartState()
    startNode = (startState, [])  # (state, action, cost)

    frontier.enqueue(startNode)

    while not frontier.is_empty():
        # begin exploring first (earliest-pushed) node on frontier
        currentState, actions = frontier.dequeue()

        if currentState not in exploredNodes:
            # put popped node state into explored list
            exploredNodes.append(currentState)

            if problem.isGoalState(currentState):
                return actions
            else:
                # list of (successor, action, stepCost)
                successors = problem.getSuccessors(currentState)

                for succState, succAction in successors:
                    successorPath = actions + [succAction]

                    succNode = (succState, successorPath)
                    frontier.enqueue(succNode)

    return actions


def getMazeDistance(start, end, problem):
    try:
        return problem.nodeDistance[(start, end)] or problem.nodeDistance[(end, start)]
    except:
        posProblem = copy.deepcopy(problem)
        posProblem.goal = end

        distance = problem.nodeDistance[(start, end)] = len(bfs(posProblem))
        return distance
    pass


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
