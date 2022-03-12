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

    count = 0

    print("Depth First Search")

    for i in range(len(problem.foodPosition)):
        stack = util.Stack()
        visitedNodes = []
        start = (problem.getStartState(), paths)

        stack.push(start)

        while not stack.is_empty():

            currentState, curPath = stack.pop()

            if currentState in visitedNodes:
                continue
            count += 1
            visitedNodes.append(currentState)

            successors = problem.getSuccessors(currentState)

            for successorState, successorAction in successors:
                successorPath = curPath + [successorAction]

                if successorState in problem.foodPosition:
                    problem.start = successorState
                    problem.foodPosition.remove(successorState)

                    if problem.isGoalState(successorState):
                        print("Visited Nodes: ", count)
                        print("Total Cost: ", len(successorPath))
                        return successorPath

                    stack.items = []
                    paths = successorPath
                    break

                if successorState not in visitedNodes:
                    stack.push((successorState, successorPath))

    print("Visited Nodes: ", count)
    print("Total Cost: ", len(paths))
    return paths


def breadthFirstSearch(problem):
    print("Breadth First Search")
    BFSpaths = []

    count = 0

    for i in range(len(problem.foodPosition)):
        queue = util.Queue()
        visitedNodes = []
        start = (problem.getStartState(), BFSpaths)

        queue.enqueue(start)

        while not queue.is_empty():

            currentState, paths = queue.dequeue()

            if currentState in visitedNodes:
                continue

            count += 1

            visitedNodes.append(currentState)

            successors = problem.getSuccessors(currentState)

            for successorState, successorAction in successors:
                successorPath = paths + [successorAction]

                if successorState in problem.foodPosition:
                    problem.start = successorState
                    problem.foodPosition.remove(successorState)

                    if problem.isGoalState(successorState):
                        print("Visited Nodes: ", count)
                        print("Total Cost: ", len(successorPath))
                        return successorPath

                    queue.data = []
                    BFSpaths = successorPath
                    break

                if successorState not in visitedNodes:
                    queue.enqueue((successorState, successorPath))

    print("Visited Nodes: ", count)
    print("Total Cost: ", len(BFSpaths))
    return BFSpaths


# same implementation as BFS because all successors' cost are the same
def uniformCostSearch(problem):
    print("Uniform Cost Search")

    count = 0
    paths = []  # list of actions from initial state

    for i in range(len(problem.foodPosition)):
        pQueue = util.PriorityQueue()
        visitedNodes = []
        # a state is a position (x, y)
        # a node contains its state (position) and the path from initial node
        # assign the starting node with its position and paths from initial node
        startNode = (problem.getStartState(), paths)
        pQueue.push(startNode, 0)  # push it into the priority queue

        while not pQueue.is_empty():  # loop until pQueue is empty
            # unpack the popped node, totalCost is the total cost from initial state to current state
            totalCost, (currentState, curPaths) = pQueue.pop()

            if currentState in visitedNodes:
                continue

            count += 1
            visitedNodes.append(currentState)  # mark the current state (x,y) as visited

            if currentState in problem.foodPosition:  # check whether the current state (x,y) is food state
                problem.start = currentState  # assign the new starting state as current state
                problem.foodPosition.remove(currentState)  # remove the food from the food list (already eaten)
                paths = curPaths
                break  # start UCS again with new starting node as the current food position

            if problem.isGoalState(currentState):
                print("Explored Node: ", count)
                print("Total Cost: ", len(paths))
                return paths

            successors = problem.getSuccessors(currentState)  # get current node's successors
            for successorState, successorAction in successors:  # for each successor
                if successorState not in visitedNodes:  # check whether successor is visited
                    successorPath = curPaths + [successorAction]  # add current path with a new successor's actions
                    successorNode = (successorState, successorPath)  # new successor node

                    newTotalCost = totalCost + 1  # in pacman, every successor has a cost of 1
                    pQueue.update(successorNode, newTotalCost)  # push the new successor in pQueue

    print("Explored Node: ", count)
    print("Total Cost: ", len(paths))
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


def multiFoodSearchHeuristic(coordinate, problem=None):
    """
    A heuristic function for the problem of multi-food search
    """
    agentToFood = []
    foodToFood = []

    if len(problem.foodPosition) == 0:
        return 0

    for food in problem.foodPosition:
        agentToFood.append((util.manhattanDistance(coordinate, food), food))

    mahattanMin, coordinateMin = min(agentToFood)

    for food in problem.foodPosition:
        foodToFood.append(util.manhattanDistance(coordinateMin, food))

    minFoodDistance = min(foodToFood)

    return mahattanMin + minFoodDistance + len(problem.foodPosition) - 2
    # return 0

    # TODO 21


def aStarSearch(problem, heuristic=singleFoodSearchHeuristic):
    n = len(problem.foodPosition)

    if type(problem) == problems.MultiFoodSearchProblem:
        heuristic = multiFoodSearchHeuristic

    print("A Star Search")

    path = []
    count = 0
    for i in range(n):
        frontier = util.PriorityQueue()
        exploredNodes = []

        startNode = (problem.getStartState(), path)
        startNodeHeuristic = heuristic(problem.getStartState(), problem)

        frontier.push(startNode, startNodeHeuristic)

        while not frontier.is_empty():
            currentCost, (currentState, actions) = frontier.pop()

            if currentState in exploredNodes:
                continue

            count += 1
            exploredNodes.append(currentState)  # put popped node into explored list

            if problem.isGoalState(currentState):
                print("Explored Node: ", count)
                print("Total Cost: ", len(path))
                return actions

            if currentState in problem.foodPosition:
                problem.foodPosition.remove(currentState)
                problem.start = currentState
                path = actions
                break

            successors = problem.getSuccessors(currentState)
            for successorState, successorAction in successors:  # examine each successor
                if successorState not in exploredNodes:
                    newAction = actions + [successorAction]
                    newCost = problem.getCostOfActions(newAction) + heuristic(currentState, problem)
                    successorNode = successorState, newAction
                    frontier.update(successorNode, newCost)

    print("Explored Node: ", count)
    print("Total Cost: ", len(path))
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
