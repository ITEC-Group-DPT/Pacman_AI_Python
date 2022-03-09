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

        queue.enqueue(start)

        while queue.is_empty() == False:

            currentState, paths = queue.dequeue()

            if currentState not in visitedNodes:
                visitedNodes.append(currentState)
                if currentState in problem.foodPosition:
                    problem.start = currentState
                    problem.foodPosition.remove(currentState)
                    break

                if problem.isGoalState(currentState):
                    return paths
                else:

                    successors = problem.getSuccessors(currentState)

                    for successorState, successorAction in successors:
                        successorPath = paths + [successorAction]

                        queue.enqueue((successorState, successorPath))
        if queue.is_empty():
            problem.foodPosition.pop(0)
    return paths


def uniformCostSearch(problem):
    """
    return a path to the goal
    """
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

    return min(agentToFood) + max(foodToFood)

    # TODO 21


def aStarSearch(problem, heuristic=nullHeuristic):
    # singleFoodSearchHeuristic(problem.getStartState(), problem)

    exploredNodeHeuristic = multiFoodSearchHeuristic(problem.getStartState(), problem)
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
