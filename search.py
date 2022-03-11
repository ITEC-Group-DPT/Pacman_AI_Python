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

    while len(problem.foodPosition) != 0:
        stack = util.Stack()
        visitedNodes = []
        start = (problem.getStartState(), paths)

        stack.push(start)

        while not stack.is_empty():

            currentState, paths = stack.pop()

            visitedNodes.append(currentState)
            if currentState in problem.foodPosition:
                problem.start = currentState
                problem.foodPosition.remove(currentState)
                break

            successors = problem.getSuccessors(currentState)

            for successorState, successorAction in successors:
                successorPath = paths + [successorAction]

                if problem.isGoalState(successorState):
                    print("Total Cost: ", len(successorPath))
                    return successorPath

                if successorState not in visitedNodes:
                    stack.push((successorState, successorPath))

    print("Total Cost: ", len(paths))
    return paths


def breadthFirstSearch(problem):
    # startState = problem.getStartState()

    # #to be explored (FIFO)
    paths = []

    while len(problem.foodPosition) != 0:
        queue = util.Queue()
        visitedNodes = []
        start = (problem.getStartState(), paths)

        queue.enqueue(start)

        while not queue.is_empty():
            currentState, paths = queue.dequeue()
            visitedNodes.append(currentState)

            visitedNodes.append(currentState)
            if currentState in problem.foodPosition:
                problem.start = currentState
                problem.foodPosition.remove(currentState)
                break

            successors = problem.getSuccessors(currentState)

            for successorState, successorAction in successors:
                successorPath = paths + [successorAction]

                if problem.isGoalState(successorState):
                    print("Total Cost: ", len(successorPath))
                    return successorPath

                if successorState not in visitedNodes:
                    queue.enqueue((successorState, successorPath))

    print("Total Cost: ", len(paths))
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
    foodToFood = [0]

    try:
        firstFood = problem.foodPosition[0]
        mahattanMin = util.manhattanDistance(state, firstFood)

        agentToFood.append(getMazeDistance(state, firstFood, problem))
    except:
        mahattanMin = 1000000000

    for food in problem.foodPosition:
        mahattanAgent = util.manhattanDistance(state, food)
        if mahattanAgent < mahattanMin:
            agentToFood.append(getMazeDistance(state, food, problem))
            mahattanMin = mahattanAgent

        for food2 in problem.foodPosition:
            if food != food2:
                foodToFood.append(getMazeDistance(food, food2, problem))

    return min(agentToFood)
    # return 0

    # TODO 21


def aStarSearch(problem, heuristic=singleFoodSearchHeuristic):
    n = len(problem.foodPosition)

    if type(problem) == problems.MultiFoodSearchProblem:
        heuristic = multiFoodSearchHeuristic

    path = []
    for i in range(n):
        frontier = util.PriorityQueue()
        exploredNodes = []

        startNode = (problem.getStartState(), path)
        startNodeHeuristic = heuristic(problem.getStartState(), problem)

        frontier.push(startNode, startNodeHeuristic)

        while not frontier.isEmpty():
            currentCost, (currentState, actions) = frontier.pop()
            exploredNodes.append(currentState)  # put popped node into explored list

            if problem.isGoalState(currentState):
                return actions

            if currentState in problem.foodPosition:
                problem.foodPosition.remove(currentState)
                problem.start = currentState
                path = actions
                break

            else:
                successors = problem.getSuccessors(currentState)
                for successorState, successorAction in successors:  # examine each successor
                    if successorState not in exploredNodes:
                        newAction = actions + [successorAction]
                        newCost = problem.getCostOfActions(newAction) + heuristic(currentState, problem)
                        successorNode = successorState, newAction
                        frontier.update(successorNode, newCost)

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
