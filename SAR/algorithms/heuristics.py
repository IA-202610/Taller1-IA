from typing import Any, Tuple
import math as m
from algorithms import utils
from algorithms.problems import MultiSurvivorProblem


def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0


def manhattanHeuristic(state, problem):
    dx = abs(problem.goal[0] - state[0])
    dy = abs(problem.goal[1] - state[1])
    return dx + dy


def euclideanHeuristic(state, problem):
    """
    The Euclidean distance heuristic.
    """
    dx = problem.goal[0] - state[0]
    dy = problem.goal[1] - state[1]
    dist = m.sqrt(dx**2 + dy**2)
    return dist

#def calcular_MST(state):
#    if 


def survivorHeuristic(state: Tuple[Tuple, Any], problem: MultiSurvivorProblem):
    """
    Your heuristic for the MultiSurvivorProblem.

    state: (position, survivors_grid)
    problem: MultiSurvivorProblem instance

    This must be admissible and preferably consistent.

    Hints:
    - Use problem.heuristicInfo to cache expensive computations
    - Go with some simple heuristics first, then build up to more complex ones
    - Consider: distance to nearest survivor + MST of remaining survivors
    - Balance heuristic strength vs. computation time (do experiments!)
    """
    position ,survivorgrid = state
    if survivorgrid == []:
        return nullHeuristic(state)
    min_dist = m.inf
    for survivor in survivorgrid:
        distance = manhattanHeuristic(position, survivor)
        if distance < min_dist:
            min_dist = distance
    
#    MST_cost
