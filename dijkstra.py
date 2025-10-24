import heapq as hq

class Vertex:
    def __init__(self, i, d, p):
        self.i = i
        self.d = d
        self.p = p
    
    def __repr__(self): # Print formatting for the vertex
        return f"Vertex(i={self.i}, p = {self.p}, d = {self.d})"
    
    def __lt__(self, other): # Ordering rule. This breaks ties in case of duplicate distances in the min-priority queue. 
        return self.i < other.i

g = [[0,1,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0],
     [1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
     [0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
     [0,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
     [0,0,0,1,0,2,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
     [0,0,0,0,2,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
     [0,0,0,0,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
     [0,0,0,0,0,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0],
     [0,0,0,0,0,0,0,1,0,1,0,0,0,0,0,0,0,0,1,0,0,0],
     [0,0,0,0,0,0,0,0,1,0,1,0,0,0,0,0,0,2,0,0,0,0],
     [1,0,0,0,0,0,0,0,0,1,0,2,0,0,0,0,1,0,0,0,0,0],
     [0,0,0,0,0,0,0,0,0,0,2,0,2,0,0,0,0,0,0,0,0,0],
     [0,0,0,0,0,0,0,0,0,0,0,2,0,2,0,0,0,0,0,0,1,0],
     [0,0,0,0,0,0,0,0,0,0,0,0,2,0,1,1,0,0,0,1,0,0],
     [0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0],
     [0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,2,0,0,0,0,0],
     [0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,2,0,1,0,0,0,0],
     [0,0,0,0,0,0,0,0,0,2,0,0,0,0,0,0,1,0,2,0,0,0],
     [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,2,0],
     [0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,2,1],
     [0,1,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,2,0,2],
     [0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,2,0]]

s = 0

def INITIALIZE_SINGLE_SOURCE(G, Q, V, s):
    for i in range(0, len(G)):
        if i != s: # Create non-source node
            v = Vertex(i, float('inf'), None)
        else: # Create source node
            v = Vertex(i, 0, i)

        hq.heappush(Q, (v.d, v))
        V.append(v)

def RELAX(u:Vertex, v:Vertex, G, V, Q):
    if v.d > u.d + G[u.i][v.i]:
        # Update v
        v.d = u.d + G[u.i][v.i]
        v.p = u.i

        # Update the global Vertex list
        V[v.i].d = v.d
        V[v.i].p = v.p
        # print("Relaxed " + str(v))

        hq.heappush(Q, (v.d, v)) # Push another instance of v into the queue, with the updated values. 

def DIJKSTRA(G, s):
    Q = [] # Min-Priority Queue
    V = [] # All vertices in G, by order of index in the adjacency list (G)
    INITIALIZE_SINGLE_SOURCE(G, Q, V, s)

    
    while Q:
        dist, u = hq.heappop(Q)
        if dist > u.d: # If the node is outdated, then skip it
            continue
        # print("\nExploring " + str(u))
        for i in range(len(G[u.i])):
            if G[u.i][i] != 0:
                v = V[i]
                # print("...is connected to vertex " + str(v))
                RELAX(u, v, G, V, Q) 

        # print(V)

    return V  
    
print(DIJKSTRA(g, s))