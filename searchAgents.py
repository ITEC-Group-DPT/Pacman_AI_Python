import random

from game import Agent
from game import Directions

import problems
import search

class GoWestAgent(Agent):
    def getAction(self, state):
        if Directions.WEST in state.getLegalPacmanActions():
            return Directions.WEST
        else:
            return Directions.STOP


class RandomAgent(Agent):
    def getAction(self, state):
        actions = state.getLegalPacmanActions()
        random.shuffle(actions)
        return actions[0]


class SearchAgent(Agent):
    # def __init__(self):
    #     self.problem =None
    #     self.algorithm=None
        
    def registerInitialState(self, state):   
        # self.start = state.getPacmanPosition()
        # self.wallGrid = state.getWalls()
        # self.foodPosition = state.getFood()
        self.index = -1
        self.actions = self.algorithm(self.problem(state))
        """
        This is the first time that the agent sees the layout of the game
        board. Here, we choose a path to the goal. In this phase, the agent
        should compute the path to the goal and store it in a local variable.
        All of the work is done in this method!

        state: a GameState object (pacman.py)
        """
        # TODO 11

    def getAction(self, state):
        """
        Returns the next action in the path chosen earlier (in
        registerInitialState).  Return Directions.STOP if there is no further
        action to take.

        state: a GameState object (pacman.py)
        """

        self.index+=1
        return self.actions[self.index]
        # return Directions.STOP
        # TODO 12


class BFSSingleFoodSearchAgent(SearchAgent):
    def __init__(self):
        self.problem = problems.SingleFoodSearchProblem
        self.algorithm = search.breadthFirstSearch

class BFSFoodSearchAgent(SearchAgent):
    def __init__(self):
        self.problem = problems.MultiFoodSearchProblem
        self.algorithm = search.breadthFirstSearch




class DFSFoodSearchAgent(SearchAgent):
    def __init__(self):
        self.search = search.depthFirstSearch(SearchAgent)
    # TODO 14
    pass


class UCSFoodSearchAgent(SearchAgent):
    # TODO 15
    def __init__(self):
        # SearchAgent.__init__(self)
        self.problem = problems.MultiFoodSearchProblem
        self.algorithm = search.uniformCostSearch


    pass


class AStarFoodSearchAgent(SearchAgent):
    # TODO 16
    pass
