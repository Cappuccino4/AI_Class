from environments.connect_four import ConnectFourState, ConnectFour
import numpy as np

vert_sets = [[[0,0],[0,1],[0,2],[0,3]], [[0,1],[0,2],[0,3],[0,4]], [[0,2],[0,3],[0,4],[0,5]], [[1,0],[1,1],[1,2],[1,3]], [[1,1],[1,2],[1,3],[1,4]], [[1,2],[1,3],[1,4],[1,5]],
[[2,0],[2,1],[2,2],[2,3]], [[2,1],[2,2],[2,3],[2,4]], [[2,2],[2,3],[2,4],[2,5]], [[3,0],[3,1],[3,2],[3,3]], [[3,1],[3,2],[3,3],[3,4]], [[3,2],[3,3],[3,4],[3,5]],
[[4,0],[4,1],[4,2],[4,3]], [[4,1],[4,2],[4,3],[4,4]], [[4,2],[4,3],[4,4],[4,5]], [[5,0],[5,1],[5,2],[5,3]], [[5,1],[5,2],[5,3],[5,4]], [[5,2],[5,3],[5,4],[5,5]],
[[6,0],[6,1],[6,2],[6,3]], [[6,1],[6,2],[6,3],[6,4]], [[6,2],[6,3],[6,4],[6,5]]]
horz_sets = [[[0,0], [1,0], [2,0], [3,0]], [[1,0], [2,0], [3,0], [4,0]], [[2,0], [3,0], [4,0], [5,0]], [[3,0], [4,0], [5,0], [6,0]],  [[0,1], [1,1], [2,1], [3,1]], [[1,1], [2,1], [3,1], [4,1]], [[2,1], [3,1], [4,1], [5,1]], [[3,1], [4,1], [5,1], [6,1]],
[[0,2], [1,2], [2,2], [3,2]], [[1,2], [2,2], [3,2], [4,2]], [[2,2], [3,2], [4,2], [5,2]], [[3,2], [4,2], [5,2], [6,2]],  [[0,3], [1,3], [2,3], [3,3]], [[1,3], [2,3], [3,3], [4,3]], [[2,3], [3,3], [4,3], [5,3]], [[3,3], [4,3], [5,3], [6,3]],
[[0,4], [1,4], [2,4], [3,4]], [[1,4], [2,4], [3,4], [4,4]], [[2,4], [3,4], [4,4], [5,4]], [[3,4], [4,4], [5,4], [6,4]],  [[0,5], [1,5], [2,5], [3,5]], [[1,5], [2,5], [3,5], [4,5]], [[2,5], [3,5], [4,5], [5,5]], [[3,5], [4,5], [5,5], [6,5]]]
diag_sets = [[[0,0], [1,1], [2,2], [3,3]], [[1,0], [2,1], [3,2], [4,3]], [[2,0], [3,1], [4,2], [5,3]], [[3,0], [4, 1], [5, 2], [6, 3]],
[[0,1], [1,2], [2,3], [3,4]], [[1,1], [2,2], [3,3], [4,4]], [[2,1], [3,2], [4,3], [5,4]], [[3,1], [4,2], [5,3], [6,4]],
[[0,2], [1,3], [2,4], [3,5]], [[1,2], [2,3], [3,4], [4,5]], [[2,2], [3,3], [4,4], [5,5]], [[3,2], [4,3], [5,4], [6,5]],
[[6,0], [5,1], [4,2], [3,3]], [[5,0], [4,1], [3,2], [2,3]], [[4,0], [3,1], [2,2], [1,3]], [[3,0], [2,1], [1,2], [0,3]], 
[[6,1], [5,2], [4,3], [3,4]], [[5,1], [4,2], [3,3], [2,4]], [[4,1], [3,2], [2,3], [1,4]], [[3,1], [2,2], [1,3], [0,4]], 
[[6,2], [5,3], [4,4], [3,5]], [[5,2], [4,3], [3,4], [2,5]], [[4,2], [3,3], [2,4], [1,5]], [[3,2], [2,3], [1,4], [0,5]]]

def make_move(state: ConnectFourState, env: ConnectFour) -> int:
    """

    :param state: the current state
    :param env: the environment
    :return: the action to take
    """
    lim = 6
    value, move = maxValue(state, env, lim, -1000000000, 1000000000)
    return move
    pass

def maxValue(state, env, lim, a, b):
    if env.is_terminal(state):
        return env.utility(state), None
    if lim == 0:
        return getValues(state, env), None
    v = -1000000000
    for i in env.get_actions(state):
        v2, a2 = minValue(env.next_state(state, i), env, (lim - 1), a, b)
        if v2 > v:
            v = v2
            m = i
            a = max(a, v)
        if v >= b:
            return v, m
    return v, m

def minValue(state, env, lim, a, b):
    if env.is_terminal(state):
        return env.utility(state), None
    if lim == 0:
        return getValues(state, env), None
    v = 1000000000
    for i in env.get_actions(state):
        v2, a2 = maxValue(env.next_state(state, i), env, (lim - 1), a, b)
        if v2 < v:
            v = v2
            m = i
            b = min(b, v)
        if v <= a:
            return v, m
    return v, m

def getValues(state, env):
    finalMaxVal = 0
    finalMinVal = 0
    for i in vert_sets:
        ## I is each set
        onlyMax = True
        onlyMin = True
        val = 0
        for j in i:
            if state.grid[j[0]][j[1]] == 1:
                ## Max
                onlyMin = False
                val = val + 1
            elif state.grid[j[0]][j[1]] == -1:
                ## Mim
                onlyMax = False
                val = val + 1
        if onlyMax and val != 0:
            finalMaxVal = finalMaxVal + val
        elif onlyMin and val != 0:
            finalMinVal = finalMinVal + val
    for i in horz_sets:
        ## I is each set
        onlyMax = True
        onlyMin = True
        val = 0
        for j in i:
            if state.grid[j[0]][j[1]] == 1:
                ## Max
                onlyMin = False
                val = val + 1
            elif state.grid[j[0]][j[1]] == -1:
                ## Mim
                onlyMax = False
                val = val + 1
        if onlyMax and val != 0:
            finalMaxVal = finalMaxVal + val
        elif onlyMin and val != 0:
            finalMinVal = finalMinVal + val
    for i in diag_sets:
        ## I is each set
        onlyMax = True
        onlyMin = True
        val = 0
        for j in i:
            if state.grid[j[0]][j[1]] == 1:
                ## Max
                onlyMin = False
                val = val + 1
            elif state.grid[j[0]][j[1]] == -1:
                ## Mim
                onlyMax = False
                val = val + 1
        if onlyMax and val != 0:
            finalMaxVal = finalMaxVal + val
        elif onlyMin and val != 0:
            finalMinVal = finalMinVal + val
    return finalMaxVal - finalMinVal
