import random

from game import Agent
from game import Directions

import problems

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
    def registerInitialState(self, state):   
        self.start = state.getPacmanPosition()
        self.wallGrid = state.getWalls()
        self.foodPosition = state.getFood()
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
        # return Directions.STOP
        # TODO 12

import search
class BFSFoodSearchAgent(SearchAgent):
    def registerInitialState(self, state):
        self.action = search.breadthFirstSearch(problems.SingleFoodSearchProblem(state))
        print(action)
    def getAction(self, state):
        
        # if self.index == len(self.action):
        #     self.index =-1
        self.index+=1

        return self.action[self.index]
    
    # TODO 13
    pass


class DFSFoodSearchAgent(SearchAgent):
    def __init__(self):
        self.search = search.depthFirstSearch(SearchAgent)
    # TODO 14
    pass


class UCSFoodSearchAgent(SearchAgent):
    # TODO 15
    pass


class AStarFoodSearchAgent(SearchAgent):
    # TODO 16
    pass
