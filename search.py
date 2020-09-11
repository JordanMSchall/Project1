# search.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
# 
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""

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


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s, s, w, s, w, w, s, w]


def depthFirstSearch(problem):

    frontier = util.Stack()
    reached = []
    path = []
    startState = problem.getStartState()
    newNode = Node(startState, path)

    if problem.isGoalState(newNode.state):
        return path

    frontier.push(newNode)

    while not frontier.isEmpty():
        node = frontier.pop()
        reached.append(node.state)

        if problem.isGoalState(node.state):
            return node.path

        stateSuccessors = problem.getSuccessors(node.state)

        if stateSuccessors:
            for successor in stateSuccessors:
                if successor[0] not in reached:
                    newPath = node.path + [successor[1]]
                    newNode = Node(successor[0], newPath)
                    frontier.push(newNode)

    return []


def breadthFirstSearch(problem):
    #Initiializing search
    frontier = util.Queue()
    reached = []
    path = []
    startState = problem.getStartState()
    newNode = Node(startState, path)

    if problem.isGoalState(newNode.state):
        return path

    frontier.push(newNode)

    while not frontier.isEmpty():
        node = frontier.pop()
        reached.append(node.state)

        if problem.isGoalState(node.state):
            return node.path

        stateSuccessors = problem.getSuccessors(node.state)

        if stateSuccessors:
            for successor in stateSuccessors:
                state = successor[0]
                if state not in reached and state not in ( node.state for node in frontier.list ):
                    newPath = node.path + [successor[1]]
                    newNode = Node(successor[0], newPath)
                    frontier.push(newNode)

    return []

def uniformCostSearch(problem):
     #Initiializing search
    frontier = util.PriorityQueue()
    reached = []
    path = []
    startState = problem.getStartState()
    newNode = Node(startState, path)

    if problem.isGoalState(newNode.state):
        return path

    frontier.push(newNode, 0)

    # find the structure for a heap entry
    # for entry in frontier.heap:
    #     print(entry[2])

    while not frontier.isEmpty():
        node = frontier.pop()
        # print(node)
        reached.append(node.state)

        if problem.isGoalState(node.state):
            return node.path

        stateSuccessors = problem.getSuccessors(node.state)

        if stateSuccessors:
            for successor in stateSuccessors:
                state = successor[0]
                if state not in reached:
                    newPath = node.path + [successor[1]]
                    newCost = problem.getCostOfActions(newPath)
                    if state not in ( (entry[2]).state for entry in frontier.heap ):
                        newNode = Node(successor[0], newPath)
                        frontier.push(newNode, newCost)
                    if state in ( (entry[2]).state for entry in frontier.heap ):
                        for heapEntry in frontier.heap:
                            if (heapEntry[2]).state == state and newCost < problem.getCostOfActions(heapEntry[2].path):
                                    (heapEntry[2]).path = newPath
                                    frontier.update((heapEntry[2]), newCost)

    return []


def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

class PriorityFunction:
    def __init__(self, heuristic, problem):
        self.heuristic = heuristic
        self.problem = problem

    def priorityFunction(self, node):
        return self.problem.getCostOfActions(node.path) + self.heuristic(node.state, self.problem)



def aStarSearch(problem, heuristic=nullHeuristic):
    #Initiializing search
    f = PriorityFunction(heuristic, problem)
    frontier = util.PriorityQueueWithFunction(f.priorityFunction)
    reached = []
    path = []
    startState = problem.getStartState()
    newNode = Node(startState, path)

    if problem.isGoalState(newNode.state):
        return path

    frontier.push(newNode)

    while (not frontier.isEmpty()):
        node = frontier.pop()

        if problem.isGoalState(node.state):
            return node.path

        # expand the node
        if node.state not in reached:
            reached.append(node.state)
            stateSuccessors = problem.getSuccessors(node.state)
            if stateSuccessors:
                for successor in stateSuccessors:
                    state = successor[0]
                    if state not in reached:
                        newPath = node.path + [successor[1]]
                        newNode = Node(successor[0], newPath)
                        frontier.push(newNode)

    return []


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch




class Node:
    """This is a class to hold nodes for searching"""
    def __init__(self, state, path):
        self.state = state
        self.path = path

