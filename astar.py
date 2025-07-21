import numpy as np
import heapq

def heuristic(current, goal):
    x1, y1, theta1, _ = current
    x2, y2, theta2, _ = goal

    return (x2-x1)**2+(y2-y1)**2+(min(theta2-theta1, 360-(theta2-theta1)))**2

def astar(start, goal, 
          rule, RuleParams=(), 
          heuristic=heuristic, HeuristicParams=(), 
          MaxError=0.1):
    '''
    start - starting values
    goal - ending values
    rule - function that gives neighbouring nodes
    heuristic - function that gives estimated cost to goal
    '''
    
    OpenHeap = [] # Effectively a sorted list of [estimated distance to get from node to finish, distance to get to node from start, current (x, y, theta, t), last (x, y, theta, t)]
    ClosedDict = {} # Dictionary of already searched points in the form of {(x, y, theta, t) : (distance to get to node from start, last (x, y, theta, t))}
    heapq.heappush(OpenHeap, (heuristic(start, goal, *HeuristicParams), 0, start, None))
    
    while True:
        # Exit if failed (no route found)
        if len(OpenHeap) == 0:
            return None
        
        _, g, current, last = heapq.heappop(OpenHeap)

        # Return path if at goal
        if np.mean(np.abs(current-goal)) < MaxError:
            path = [current]

            while last != None:
                path.append(last)
                _, last = ClosedDict[last]
            
            return path[::-1]
        
        # Iterate through neighbors and add to heap if first time arriving at node or if cost to get to node found is lower
        neighbors = rule(current, *RuleParams)
        for neighbor in neighbors:
            gNeighbor = g + 1
            gPreexisting = ClosedDict.get(neighbor, (None, float('inf')))[1]

            if gNeighbor >= gPreexisting:
                continue

            ClosedDict[neighbor] = (g, current)
            heapq.heappush(OpenHeap, (heuristic(current, goal, *HeuristicParams)+gNeighbor, gNeighbor, neighbor, current))