import util
from game import Actions
from game import Directions


class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).

    You do not need to change anything in this class, ever.
    """

    def getStartState(self):
        """
        Returns the start state for the search problem.
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state.
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples, (successor,
        action, stepCost), where 'successor' is a successor to the current
        state, 'action' is the action required to get there, and 'stepCost' is
        the incremental cost of expanding to that successor.
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.
        The sequence must be composed of legal moves.
        """
        util.raiseNotDefined()


class SingleFoodSearchProblem(SearchProblem):
    def __init__(self, startingGameState):

        # pacman start (col,row bottom up)
        self.start = startingGameState.getPacmanPosition()
        self.wallGrid = startingGameState.getWalls()
        self.foodGrid = startingGameState.getFood()
        print(self.foodGrid)

        self.goal = None
        self.foodPosition = []

        for col in range(self.foodGrid.width - 1):
            for row in range(self.foodGrid.height - 1):
                if self.foodGrid.data[col][row] == True:
                    self.foodPosition.append((col, row))

        pass

    def getStartState(self):
        return self.start

    def isGoalState(self, state):
        return state in self.foodPosition

    def getSuccessors(self, state):
        successors = []
        for action in [Directions.NORTH, Directions.EAST, Directions.WEST, Directions.SOUTH]:
            x, y = state
            dx, dy = Actions.directionToVector(action)
            neighborX, neighborY = int(x + dx), int(y + dy)

            if self.wallGrid[neighborX][neighborY] == False:
                nextState = (neighborX, neighborY)
                successors.append((nextState, action))

        return successors

    def getCostOfActions(self, actions):
        return len(actions)


class MultiFoodSearchProblem(SearchProblem):
    def __init__(self, startingGameState):
        self.start = startingGameState.getPacmanPosition()

        self.wallGrid = startingGameState.getWalls()

        self.foodGrid = startingGameState.getFood()

        self.foodPosition = []
        self.goal = None

        self.nodeDistance = {}

        for col in range(self.foodGrid.width - 1):
            for row in range(self.foodGrid.height - 1):
                if self.foodGrid.data[col][row] == True:
                    self.foodPosition.append((col, row))

    def getStartState(self):
        return self.start

    def isGoalState(self, state):
        return self.goal is not None and state == self.goal


    def getSuccessors(self, state):
        successors = []
        for action in [Directions.NORTH, Directions.EAST, Directions.WEST, Directions.SOUTH]:
            x, y = state
            dx, dy = Actions.directionToVector(action)
            neighborX, neighborY = int(x + dx), int(y + dy)

            if self.wallGrid[neighborX][neighborY] == False:
                nextState = (neighborX, neighborY)
                successors.append((nextState, action))

        return successors

    def getCostOfActions(self, actions):
        return len(actions)
