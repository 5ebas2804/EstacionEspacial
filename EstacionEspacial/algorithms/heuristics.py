from typing import Tuple
from algorithms import utils
from algorithms.problems import SystemRepairProblem
import math


def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0



def manhattanHeuristic(state, problem):
    """
    The Manhattan distance heuristic.

    Baseline rule for this workshop: estimate the direct distance to the next
    mandatory target:
    - K if the robot does not have the kit yet.
    - the nearest pending T if the robot has the kit and systems remain.
    - C if all systems have been repaired.
    
    abs(x2-x1)-abs(y2-y1)
    """
    pos_i, hasKit, pendingSystems = state
    x1, y1 = pos_i

    if not hasKit:
        pos_f = problem.kitPosition
    elif len(pendingSystems) > 0:
        pos_f = min(pendingSystems, key=lambda t: abs(t[0]-x1) + abs(t[1]-y1))
    else:
        pos_f = problem.controlPosition

    x2, y2 = pos_f
    return abs(x2 - x1) + abs(y2 - y1)
    



def euclideanHeuristic(state, problem):
    """
    The Euclidean distance heuristic.

    Baseline rule for this workshop: estimate the direct distance to the next
    mandatory target:
    - K if the robot does not have the kit yet.
    - the nearest pending T if the robot has the kit and systems remain.
    - C if all systems have been repaired.
    
    d= math.sqrt((x2-x1)**2-(y2-y1)**2)
    """
    pos_i, hasKit, pendingSystems = state
    x1, y1 = pos_i

    if not hasKit:
        pos_f = problem.kitPosition
    elif len(pendingSystems) > 0:
        pos_f = min(pendingSystems, key=lambda t: abs(t[0]-x1) + abs(t[1]-y1))
    else:
        pos_f = problem.controlPosition

    x2, y2 = pos_f
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)


def systemRepairHeuristic(
    state: Tuple[Tuple, bool, Tuple], problem: SystemRepairProblem
):
    """
    Your heuristic for the SystemRepairProblem.

    state: (position, hasKit, pendingSystems)
    problem: SystemRepairProblem instance

    This must be admissible and preferably consistent.

    Hints:
    - Use problem.heuristicInfo to cache expensive computations
    - Go with some simple heuristics first, then build up to more complex ones
    - Consider the kit, pending systems, and the final return to control center
    - Balance heuristic strength vs. computation time (do experiments!)
    """
    # TODO: Add your code here
    utils.raiseNotDefined()
