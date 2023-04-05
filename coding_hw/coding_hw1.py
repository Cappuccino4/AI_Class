from typing import List, Tuple, Dict, Optional, cast
from environments.environment_abstract import Environment, State
from environments.farm_grid_world import FarmState
from environments.n_puzzle import NPuzzleState
from heapq import heappush, heappop
import time
import numpy as np

goal = [0,1,2,3,4,5,6,7,8]

class Node:
    def __init__(self, state: State, path_cost: float, parent_action: Optional[int], parent, g: int):
        self.state: State = state
        self.parent: Optional[Node] = parent
        self.path_cost: float = path_cost
        self.parent_action: Optional[int] = parent_action
        self.g: int = g

    def __hash__(self):
        return self.state.__hash__()

    def __gt__(self, other):
        return self.path_cost < other.path_cost

    def __eq__(self, other):
        return self.state == other.state


def get_next_state_and_transition_cost(env: Environment, state: State, action: int) -> Tuple[State, float]:
    """

    :param env: Environment
    :param state: State
    :param action: Action
    :return: the next state and the transition cost
    """
    rw, states_a, _ = env.state_action_dynamics(state, action)
    state: State = states_a[0]
    transition_cost: float = -rw

    return state, transition_cost


def visualize_bfs(viz, closed_states: List[State], queue: List[Node], wait: float):
    """

    :param viz: visualizer
    :param closed_states: states in CLOSED
    :param queue: states in priority queue
    :param wait: number of seconds to wait after displaying
    :return: None
    """

    if viz is None:
        return

    grid_dim_x, grid_dim_y = viz.env.grid_shape
    for pos_i in range(grid_dim_x):
        for pos_j in range(grid_dim_y):
            viz.board.itemconfigure(viz.grid_squares[pos_i][pos_j], fill="white")

    for state_u in closed_states:
        pos_i_up, pos_j_up = state_u.agent_idx
        viz.board.itemconfigure(viz.grid_squares[pos_i_up][pos_j_up], fill="red")

    for node in queue:
        state_u: FarmState = cast(FarmState, node.state)
        pos_i_up, pos_j_up = state_u.agent_idx
        viz.board.itemconfigure(viz.grid_squares[pos_i_up][pos_j_up], fill="grey")

    viz.window.update()
    time.sleep(wait)


def search_optimal(state_start: State, env: Environment, viz) -> Optional[List[int]]:
    """ Return an optimal path

    :param state_start: starting state
    :param env: environment
    :param viz: visualization object

    :return: a list of integers representing the actions that should be taken to reach the goal or None if no solution
    """

# A*
    startNode = Node(state_start, 0, None, None, 0)
    open = [startNode]
    closed = {startNode.state: startNode}
    if env.is_terminal(state_start):
        return []
    while len(open) > 0:
        open.sort(key=getLength)
        q = open[0]
        open.remove(q)
        possibleMoves = []
        ## Made my own possible moves
        tiles = q.state.tiles
        ind = np.where(tiles==0)[0][0]
        if ind//3 == 0:
            possibleMoves.append(0)
        elif ind//3 == 1:
            possibleMoves.append(0)
            possibleMoves.append(1)
        else:
            possibleMoves.append(1)
        if ind%3 == 0:
            possibleMoves.append(2)
        elif ind%3 == 1:
            possibleMoves.append(2)
            possibleMoves.append(3)
        else:
            possibleMoves.append(3)
        if q.parent_action == 0:
            possibleMoves.remove(1)
        elif q.parent_action == 1:
            possibleMoves.remove(0)
        elif q.parent_action == 2:
            possibleMoves.remove(3)
        elif q.parent_action == 3:
            possibleMoves.remove(2)
        # Possible move loop
        for i in possibleMoves:
            newState = get_next_state_and_transition_cost(env, q.state, i)[0]
            if env.is_terminal(newState):
                # Done
                finalState = Node(newState, 0, i, q, 0)
                ret = getInstructionList(finalState)
                open.clear()
                closed.clear()
                return reversed(ret)
            ## Find f value for new Node
            f = manhattanDistance(newState) + q.g + 1
            newNode = Node(newState, f, i, q, q.g+1)
            if newState not in closed or newNode.path_cost < closed[newState].path_cost:
                closed[newState] = newNode
                open.insert(0, newNode)
    return []

def search_speed(state_start: State, env: Environment, viz) -> Optional[List[int]]:
    """ Return a path as quickly as possible

    :param state_start: starting state
    :param env: environment
    :param viz: visualization object

    :return: a list of integers representing the actions that should be taken to reach the goal or None if no solution
    """
    # Greedy BFS
    startNode = Node(state_start, 0, None, None, 0)
    open = [startNode]
    closed = {startNode.state: startNode}
    if env.is_terminal(state_start):
        return []
    while len(open) > 0:
        open.sort(key=getLength)
        q = open[0]
        open.remove(q)
        possibleMoves = []
        ## Made my own possible moves
        tiles = q.state.tiles
        ind = np.where(tiles==0)[0][0]
        if ind//3 == 0:
            possibleMoves.append(0)
        elif ind//3 == 1:
            possibleMoves.append(0)
            possibleMoves.append(1)
        else:
            possibleMoves.append(1)
        if ind%3 == 0:
            possibleMoves.append(2)
        elif ind%3 == 1:
            possibleMoves.append(2)
            possibleMoves.append(3)
        else:
            possibleMoves.append(3)
        if q.parent_action == 0:
            possibleMoves.remove(1)
        elif q.parent_action == 1:
            possibleMoves.remove(0)
        elif q.parent_action == 2:
            possibleMoves.remove(3)
        elif q.parent_action == 3:
            possibleMoves.remove(2)
        # Possible Move Loop
        for i in possibleMoves:
            newState = get_next_state_and_transition_cost(env, q.state, i)[0]
            if env.is_terminal(newState):
                # Done
                finalState = Node(newState, 0, i, q, 0)
                ret = getInstructionList(finalState)
                open.clear()
                closed.clear()
                return reversed(ret)
            ## Find f value for new Node
            f = manhattanDistance(newState)
            newNode = Node(newState, f, i, q, q.g+1)
            if newState not in closed or newNode.path_cost < closed[newState].path_cost:
                closed[newState] = newNode
                open.insert(0, newNode)
    return []

def getLength(node: Node) -> Optional[List[int]]:
    return node.path_cost

def getInstructionList(node: Node) -> Optional[List[int]]:
    ret = []
    if node.parent is not None:
        ret = [node.parent_action]
        ret += getInstructionList(node.parent)
    return ret

def differentTiles(state: State) -> Optional[int]:
    ret = 0
    for i in range(len(state.tiles)):
        if state.tiles[i] != i and state.tiles[i] != 0:
            ret += 1
    return ret

def manhattanDistance(state: State) -> Optional[int]:
    ret = 0
    for i in goal:
        if i != 0:
            for j in range(len(state.tiles)):
                if i == state.tiles[j]:
                    col = abs((j%3) - (i%3))
                    row = abs((i//3)-(j//3))
                    ret += (col + row)
    return ret