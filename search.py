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
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print("Start:", problem.getStartState())
    print("Is the start a goal?", problem.isGoalState(problem.getStartState()))
    print("Start's successors:", problem.getSuccessors(problem.getStartState()))
    """
    # Need to somehow make this cleaner.... maybe use recursion instead
    stack = util.Stack()
    visited = []

    currentNode = (problem.getStartState(), None, 0)  # Initialize in the same way as other successor nodes
    parents = {(problem.getStartState(), None, 0): None}  # Assign parent of source node as None
    goalNode = None

    for successor in problem.getSuccessors(currentNode[0]):
        successorCoordinates = successor[0]
        if successorCoordinates not in visited:
            if problem.isGoalState(successorCoordinates):
                if successor not in parents:
                    parents[successor] = currentNode
                goalNode = successor
            else:
                if successor not in parents:
                    parents[successor] = currentNode
                stack.push(successor)
                visited.append(successorCoordinates)

    # If we still haven't come across a goalNode
    if goalNode is None:
        while stack.isEmpty() is not True:
            # Continue performing DFS Search
            currentNode = stack.pop()
            for successor in problem.getSuccessors(currentNode[0]):
                successorCoordinates = successor[0]
                if successorCoordinates not in visited:
                    if problem.isGoalState(successorCoordinates):
                        if successor not in parents:
                            parents[successor] = currentNode
                        goalNode = successor
                        return returnActionsList(goalNode, parents)
                    else:
                        if successor not in parents:
                            parents[successor] = currentNode
                        stack.push(successor)
                        visited.append(successorCoordinates)

    # We should've come across the goal node by now
    return returnActionsList(goalNode, parents)

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    visited = []
    queue = util.Queue()

    currentNode = (problem.getStartState(), None, 0)
    successors = problem.getSuccessors(currentNode[0])
    parents = {(problem.getStartState(), None, 0): None}  # Assign parent of source node as None
    goalNode = None

    for successor in successors:
        successorCoordinates = successor[0]
        if successorCoordinates not in visited:
            if problem.isGoalState(successorCoordinates):
                if successor not in parents:
                    parents[successor] = currentNode
                goalNode = successor
                break
            else:
                if successor not in parents:
                    parents[successor] = currentNode
                queue.push(successor)
                visited.append(successorCoordinates)

    if goalNode is None:
        while queue.isEmpty() is not True:
            currentNode = queue.pop()
            successors = problem.getSuccessors(currentNode[0])
            for successor in successors:
                successorCoordinates = successor[0]
                if successorCoordinates not in visited:
                    if problem.isGoalState(successorCoordinates):
                        if successor not in parents:
                            parents[successor] = currentNode
                        goalNode = successor
                        break
                    else:
                        if successor not in parents:
                            parents[successor] = currentNode
                        queue.push(successor)
                        visited.append(successorCoordinates)

    return returnActionsList(goalNode, parents)

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    fringe = util.PriorityQueue()  # fringe is implemented as a PriorityQueue


def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    util.raiseNotDefined()


# Helper functions that we may or may not use
def noMoreSuccessors(successors, visited):
    "Returns whether there are any more successors not already visited"
    if visited == [] and successors != []:
        return True
    for successor in successors:
        if successor not in visited:
            return False
    return True

def visitRemainingNodes(currentNode, successors, visited, stack, dfsOutput, parents):
    "Mutatively visits a successor if it hasn't already been visited"
    if not noMoreSuccessors(successors, visited):
        for successor in successors:
            if successor not in visited:
                visited.append(successor)
                stack.push(successor)
                dfsOutput.push(successor)
                if successor not in parents:
                    parents[successor] = currentNode
                return

def returnActionsList(goal, parents):
    "Returns Direction objects' path list from start to goal"
    actionsStack = util.Stack()
    currentNode = goal
    actionsStack.push(currentNode)
    while parents[parents[currentNode]] is not None:
        actionsStack.push(parents[currentNode])
        currentNode = parents[currentNode]
    result = []
    while not actionsStack.isEmpty():
        result.append(actionsStack.pop()[1])
    return result

def containsGoalNode(successors, problem):
    "Returns whether the goal node is one of the successors"
    if getGoalNode(successors, problem) is None:
        return False
    return True

def getGoalNode(successors, problem, currentNode, parents):
    "Returns the goal node from the list of successors, and marks its parent as currentNode"
    for successor in successors:
        if problem.isGoalState(successor):
            if successor in parents:
                parents[successor] = currentNode
            return successor
    return None

# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
