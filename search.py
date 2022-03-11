"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""
import problems
from game import Directions
import util
import copy
from collections import defaultdict

n = Directions.NORTH
s = Directions.SOUTH
e = Directions.EAST
w = Directions.WEST


def depthFirstSearch(problem):
    stack = util.Stack()
    visitedNodes = []

    startNode = (problem.getStartState(), [])
    stack.push(startNode)

    while not stack.is_empty():
        curState, paths = stack.pop()

        if curState not in visitedNodes:
            visitedNodes.append(curState)

            successors = problem.getSuccessors(curState)

            for successorState, successorAction in successors:
                successorPath = paths + [successorAction]
                if problem.isGoalState(successorState):
                    return successorPath

                stack.push((successorState, successorPath))

    return paths


def breadthFirstSearch(problem):
    queue = util.Queue()
    visitedNodes = defaultdict()

    startNode = (problem.getStartState(), [])
    queue.enqueue(startNode)

    while not queue.is_empty():
        curState, paths = queue.dequeue()

        curCoordinate, curFoodPosition = curState
        try:
            visitedNodes[curCoordinate]
        except:
            visitedNodes.setdefault(curCoordinate, [])

        if curFoodPosition not in visitedNodes[curCoordinate]:

            if len(visitedNodes[curCoordinate]) > 0:
                minFoodRemain = min(map(len, visitedNodes[curCoordinate]))

                if len(curFoodPosition) > minFoodRemain:
                    continue

            visitedNodes[curState[0]].append(curState[1])

            successors = problem.getSuccessors(curState)

            for successorState, successorAction in successors:
                successorPath = paths + [successorAction]

                if problem.isGoalState(successorState):
                    print("true ?")
                    return successorPath

                queue.enqueue((successorState, successorPath))

    return paths


def uniformCostSearch(problem):
    frontier = util.PriorityQueue()
    visitedNodes = defaultdict()

    startNode = (problem.getStartState(), [])
    frontier.push(startNode, 0)

    while not frontier.is_empty():
        curCost, curNode = frontier.pop()
        (curState, curPath) = curNode

        curCoordinate, curFoodPosition = curState
        try:
            visitedNodes[curCoordinate]
        except:
            visitedNodes.setdefault(curCoordinate, [])

        if curFoodPosition not in visitedNodes[curCoordinate]:

            if len(visitedNodes[curCoordinate]) > 0:
                minFoodRemain = min(map(len, visitedNodes[curCoordinate]))

                if len(curFoodPosition) > minFoodRemain:
                    continue

            visitedNodes[curState[0]].append(curState[1])

            if problem.isGoalState(curState):
                return curPath

            successors = problem.getSuccessors(curState)

            for successorState, successorAction in successors:
                # if successorState not in visitedNodes:
                successorPath = curPath + [successorAction]
                frontier.update((successorState, successorPath), len(successorPath))

    return curPath
    pass


def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0


def singleFoodSearchHeuristic(state, problem=None):
    if len(state[1]) == 0:
        return 0
    return util.manhattanDistance(state[0], state[1][0])


def multiFoodSearchHeuristic(state, problem):
    """
    A heuristic function for the problem of multi-food search
    """
    agentToFood = []
    foodToFood = []

    coordinate, foodPosition = state

    if len(foodPosition) == 0:
        return 0

    for food in foodPosition:
        agentToFood.append((util.manhattanDistance(coordinate, food), food))
        for food2 in foodPosition:
            foodToFood.append((util.manhattanDistance(food, food2), (food, food2)))

    mahattanMin = min(agentToFood)[1]
    mahattanMax = max(foodToFood)[1]

    closestFood = getMazeDistance(coordinate, mahattanMin, problem)
    furthestFood = getMazeDistance(mahattanMax[0], mahattanMax[1], problem)

    return closestFood + furthestFood

    # TODO 21


def aStarSearch(problem, heuristic=singleFoodSearchHeuristic):
    if type(problem) == problems.MultiFoodSearchProblem:
        heuristic = multiFoodSearchHeuristic

    frontier = util.PriorityQueue()
    visitedNodes = defaultdict()

    count = 0
    startNode = (problem.getStartState(), [])
    frontier.push(startNode, 0)

    while not frontier.is_empty():
        curCost, curNode = frontier.pop()
        (curState, curPath) = curNode

        curCoordinate, curFoodPosition = curState
        try:
            visitedNodes[curCoordinate]
        except:
            visitedNodes.setdefault(curCoordinate, [])

        if curFoodPosition not in visitedNodes[curCoordinate]:

            if len(visitedNodes[curCoordinate]) > 0:
                minFoodRemain = min(map(len, visitedNodes[curCoordinate]))

                if len(curFoodPosition) > minFoodRemain:
                    continue

            visitedNodes[curState[0]].append(curState[1])

            if problem.isGoalState(curState):
                print("maze run: ", count * 2)
                return curPath

            successors = problem.getSuccessors(curState)

            for successorState, successorAction in successors:
                visitedNodes.setdefault(successorState[0], [])
                try:
                    minFoodSuccessor = min(map(len, visitedNodes[successorState[0]]))
                except:
                    minFoodSuccessor = 10000

                if successorState[1] not in visitedNodes[successorState[0]] and (
                        len(successorState[1]) < minFoodSuccessor):
                    successorPath = curPath + [successorAction]
                    count += 1
                    successorCost = len(successorPath) + heuristic(successorState, problem)

                    frontier.update((successorState, successorPath), successorCost)

    return curPath


def getMazeDistance(start, end, problem):
    try:
        return problem.nodeDistance[(start, end)] or problem.nodeDistance[(end, start)]
    except:
        posProblem = copy.deepcopy(problem)
        posProblem.start = start
        posProblem.goal = end

        distance = problem.nodeDistance[(start, end)] = len(bfs(posProblem))
        return distance
    pass


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
