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
        self.index = -1
        self.actions = self.algorithm(self.problem(state))
        """
        This is the first time that the agent sees the layout of the game
        board. Here, we choose a path to the goal. In this phase, the agent
        should compute the path to the goal and store it in a local variable.
        All of the work is done in this method!

        state: a GameState object (pacman.py)
        """


    def getAction(self, state):
        """
        Returns the next action in the path chosen earlier (in
        registerInitialState).  Return Directions.STOP if there is no further
        action to take.

        state: a GameState object (pacman.py)
        """

        self.index+=1
        return self.actions[self.index] if self.index < len(self.actions) else Directions.STOP
        # return Directions.STOP
  


class BFSSingleFoodSearchAgent(SearchAgent):
    def __init__(self):
        self.problem = problems.SingleFoodSearchProblem
        self.algorithm = search.breadthFirstSearch

class BFSFoodSearchAgent(SearchAgent):
    def __init__(self):
        self.problem = problems.MultiFoodSearchProblem
        self.algorithm = search.breadthFirstSearch
        


class DFSSingleFoodSearchAgent(SearchAgent):
    def __init__(self):
        self.problem = problems.SingleFoodSearchProblem
        self.algorithm = search.depthFirstSearch

class DFSFoodSearchAgent(SearchAgent):
    def __init__(self):
        self.problem = problems.MultiFoodSearchProblem
        self.algorithm = search.depthFirstSearch


class UCSSingleFoodSearchAgent(SearchAgent):
    def __init__(self):
        self.problem = problems.SingleFoodSearchProblem
        self.algorithm = search.uniformCostSearch

class UCSFoodSearchAgent(SearchAgent):
    def __init__(self):
        self.problem = problems.MultiFoodSearchProblem
        self.algorithm = search.uniformCostSearch


class AStarFoodSearchAgent(SearchAgent):
    def __init__(self):
        self.problem = problems.SingleFoodSearchProblem
        self.algorithm = search.aStarSearch

class AStarFoodSearchAgent(SearchAgent):
    def __init__(self):
        self.problem = problems.MultiFoodSearchProblem
        self.algorithm = search.aStarSearch
