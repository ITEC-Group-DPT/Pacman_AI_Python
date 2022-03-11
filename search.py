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

    count = 0
    for food in problem.foodPosition:
        mahattanAgent = util.manhattanDistance(state, food)
        if mahattanAgent < mahattanMin:
            count += 1
            agentToFood.append(getMazeDistance(state, food, problem))
            mahattanMin = mahattanAgent

        # for food2 in problem.foodPosition:
        #     if food != food2:
        #         foodToFood.append(getMazeDistance(food, food2, problem))

    agentToFood.sort()

    return agentToFood[0] + agentToFood[len(agentToFood) - 1]
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
