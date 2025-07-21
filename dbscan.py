import numpy as np

def GetNeighboursMask(data, ind, eps):
        point = data[ind]
        distances = np.linalg.norm(data - point, axis=1)
        return list(np.nonzero(distances <= eps)[0])

def dbscan(data, eps, MinNeighbors):
    """
    data - Input array of shape [n, 2]
    eps - Maximum distance between points in the same neighborhood
    MinNeighbors - Minimum number of points to form a dense region
    """
    
    n = data.shape[0]
    visited = [False for _ in range(n)]
    clusters = []
    
    for SeedInd in range(n):
        if visited[SeedInd]: 
            continue

        visited[SeedInd] = True
        neighbors = GetNeighboursMask(data, SeedInd, eps)
        
        if len(neighbors) < MinNeighbors:
            continue

        clusters.append([data[SeedInd]])
        i = 0
        while True:
            if i == len(neighbors):
                break

            NeighborInd = neighbors[i]
            if visited[NeighborInd]:
                i += 1
                continue
            
            visited[NeighborInd] = True
            NewNeighbours = GetNeighboursMask(data, NeighborInd, eps)
            
            if len(NewNeighbours) < MinNeighbors:
                clusters[-1].append(data[NeighborInd])
                i += 1
                continue
            
            neighbors += NewNeighbours
            clusters[-1].append(data[NeighborInd])
            
            i += 1
        
        clusters[-1] = np.array(clusters[-1])
    
    return clusters