import util


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
        # TODO 1
        pass

    def getStartState(self):
        # TODO 1
        pass

    def isGoalState(self, state):
        # TODO 3
        pass

    def getSuccessors(self, state):
        # TODO 4
        pass

    def getCostOfActions(self, actions):
        # TODO 5
        pass


class MultiFoodSearchProblem(SearchProblem):
    def __init__(self, startingGameState):
        # TODO 6
        pass

    def getStartState(self):
        # TODO 7
        pass

    def isGoalState(self, state):
        # TODO 8
        pass

    def getSuccessors(self, state):
        # TODO 9
        pass

    def getCostOfActions(self, actions):
        # TODO 10
        pass
