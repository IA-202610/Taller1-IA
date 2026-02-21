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
    
    survivors = []
    for x in range(survivorgrid.width):
        for y in range(survivorgrid.height):
            if survivorgrid[x][y]:
                survivors.append((x, y))
                
    if not survivors:
        return 0
    
    min_dist = m.inf
    for survivor in survivors:
        distance = abs(position[0] - survivor[0]) + abs(position[1] - survivor[1])
        if distance < min_dist:
            min_dist = distance
            
    if len(survivors) == 1:
        return min_dist
    
    mst_cost = 0
    visited = {survivors[0]}
    not_visited = set(survivors[1:])

    while not_visited:
        best_edge = float("inf")
        best_node = None

        for v in visited:
            for u in not_visited:
                dist = abs(v[0] - u[0]) + abs(v[1] - u[1])
                if dist < best_edge:
                    best_edge = dist
                    best_node = u

        mst_cost += best_edge
        visited.add(best_node)
        not_visited.remove(best_node)

    return min_dist + mst_cost
    
    
    

