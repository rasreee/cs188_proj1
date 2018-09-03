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
    visited = []
    stack = util.Stack()
    dfsOutput = util.Queue()

    stack.push(problem.getStartState())
    dfsOutput.push(problem.getStartState())
    visited.append(problem.getStartState())
    parents = {problem.getStartState(): None}

    currentNode = problem.getStartState()
    successors = problem.getSuccessors(currentNode)
    foundGoalState = False
    while not noMoreSuccessors(successors, visited):
        if currentNode is not problem.getStartState() and problem.isGoalState(successors[0]):
            foundGoalState = True
            print("FOUND GOAL STATE", foundGoalState)
            break
        stack.push(successors[0])
        dfsOutput.push(successors[0])
        visited.append(successors[0])
        if successors[0] not in parents:
            parents[successors[0]] = currentNode
        currentNode = successors[0]
        successors = problem.getSuccessors(currentNode[0])

    if foundGoalState:
        return returnActionsList(problem.getStartState(), successors[0], parents)

    # Check for unvisited nodes by back-tracking the stack
    currentNode = stack.pop()
    while not stack.isEmpty():
        if currentNode == problem.getStartState():
            stack.pop()  # Now stack is empty
            break
        successors = problem.getSuccessors(currentNode[0])
        if not containsGoalNode(successors, problem):
            visitRemainingNodes(currentNode, successors, visited, stack, dfsOutput, parents)
        else:
            goalNode = getGoalNode(successors, problem, currentNode, parents)
            return returnActionsList(problem.getStartState(), goalNode, parents)

        currentNode = stack.pop()

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    visited = []
    queue = util.Queue()
    currentNode = problem.getStartState()

    parents = {problem.getStartState(): None}  # Assign parent of source node as Null
    visited.append(currentNode)
    queue.push(currentNode)

    bfsOutput = util.Queue()
    foundGoalState = False
    while not queue.isEmpty():
        if currentNode is not problem.getStartState() and problem.isGoalState(currentNode[1]):
            foundGoalState = True
            break
        currentNode = queue.pop()
        bfsOutput.push(currentNode)
        if currentNode == problem.getStartState():
            successors = problem.getSuccessors(currentNode)
        else:
            successors = problem.getSuccessors(currentNode[0])
        for successor in successors:
            if successor not in parents:
                parents[successor] = currentNode
            if successor not in visited:
                visited.append(successor)
                queue.push(successor)
    print("FOUND GOAL STATE BEFORE EMPTY QUEUE", foundGoalState)
    bfsOutput.printSelf()

    actionsStack = util.Stack()
    while parents[parents[currentNode]] is not None:
        actionsStack.push(parents[currentNode])
        currentNode = parents[currentNode]
    # actionsStack.printSelf()

    return actionObjectVersion(actionsStack)

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    util.raiseNotDefined()

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
    if noMoreSuccessors(successors, visited):
        return
    else:
        for successor in successors:
            if successor not in visited:
                visited.append(successor)
                stack.push(successor)
                dfsOutput.push(successor)
                if successor not in parents:
                    parents[successor] = currentNode
                return

def returnActionsList(start, goal, parents):
    "Returns Direction objects' path list from start to goal"
    actionsStack = util.Stack()
    currentNode = goal
    while parents[parents[currentNode]] is not None:
        actionsStack.push(parents[currentNode])
        currentNode = parents[currentNode]
    return actionObjectVersion(actionsStack)

def actionObjectVersion(stack):
    """Converts stack of String directions to actual direction objects NORTH, SOUTH, EAST, WEST
    making sure to exclude start"""
    from game import Directions
    result = []
    while not stack.isEmpty():
        current = stack.pop()
        if current[1] == "North":
            result.append(Directions.NORTH)
        if current[1] == "East":
            result.append(Directions.EAST)
        if current[1] == "South":
            result.append(Directions.SOUTH)
        if current[1] == "West":
            result.append(Directions.WEST)
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
