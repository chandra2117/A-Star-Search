<h1>ExpNo 4 : Implement A* search algorithm for a Graph</h1> 
<h3>Name: CHANDRAPRIYADHARSHINI C</h3>
<h3>Register Number: 212223240019</h3>
<H3>Aim:</H3>
<p>To ImplementA * Search algorithm for a Graph using Python 3.</p>
<H3>Algorithm:</H3>
1.  Initialize the open list

2.  Initialize the closed list
    put the starting node on the open 
    list (you can leave its f at zero)
    
3.  while the open list is not empty
    a) find the node with the least f on 
       the open list, call it "q"
    b) pop q off the open list
    c) generate q's 8 successors and set their 
       parents to q
    d) for each successor
        i) if successor is the goal, stop search
        ii) else, compute both g and h for successor
          successor.g = q.g + distance between 
                              successor and q
          successor.h = distance from goal to 
          successor (This can be done using many 
          ways, we will discuss three heuristics- 
          Manhattan, Diagonal and Euclidean 
          Heuristics)
          successor.f = successor.g + successor.h
        iii) if a node with the same position as 
            successor is in the OPEN list which has a 
           lower f than successor, skip this successor
        iV) if a node with the same position as 
            successor  is in the CLOSED list which has
            a lower f than successor, skip this successor
            otherwise, add  the node to the open list
     end (for loop)
    e) push q on the closed list
    end (while loop)
<hr>
<h2>Program</h2>

```
from collections import defaultdict
H_dist = {}
def aStarAlgo(start_node, stop_node):
    open_set = set(start_node)
    closed_set = set()
    g = {}               # store distance from starting node
    parents = {}         # parents contains an adjacency map of all nodes
    # distance of starting node from itself is zero
    g[start_node] = 0
    # start_node is root node i.e. it has no parent
    parents[start_node] = start_node
    while len(open_set) > 0:
        n = None
        # node with lowest f() is found
        for v in open_set:
            if n is None or g[v] + heuristic(v) < g[n] + heuristic(n):
                n = v
        # if n is None, no path can be found
        if n is None:
            print('Path does not exist!')
            return None
        if n == stop_node or Graph_nodes[n] is None:
            pass
        else:
            for (m, weight) in get_neighbors(n):
                # nodes 'm' not in open or closed sets are added to open_set
                if m not in open_set and m not in closed_set:
                    open_set.add(m)
                    parents[m] = n
                    g[m] = g[n] + weight
                else:
                    # update if a shorter path is found
                    if g[m] > g[n] + weight:
                        g[m] = g[n] + weight
                        parents[m] = n
                        if m in closed_set:
                            closed_set.remove(m)
                            open_set.add(m)
        
        # if the current node is the stop_node, reconstruct path
        if n == stop_node:
            path = []
            while parents[n] != n:
                path.append(n)
                n = parents[n]
            path.append(start_node)
            path.reverse()
            print('Path found: {}'.format(path))
            return path
        # move n from open_set to closed_set
        open_set.remove(n)
        closed_set.add(n)
    print('Path does not exist!')
    return None
    def get_neighbors(v):
    """
    Retrieves a value from the Graph_nodes dictionary based on the provided key.

    Parameters:
    v (hashable): The key used to look up the value in the Graph_nodes dictionary.

    Returns:
    list of tuples: Each tuple contains (neighbor, cost)
    """
        if v in Graph_nodes:
            return Graph_nodes[v]
        else:
            return None

    def heuristic(n):
        return H_dist[n]

    if __name__ == "__main__":
        graph = defaultdict(list)
        n, e = map(int, input("Enter number of nodes and edges: ").split())
        print("Enter edges (u v cost):")
        for i in range(e):
            u, v, cost = map(str, input().split())
            t = (v, float(cost))
            graph[u].append(t)
            t1 = (u, float(cost))
            graph[v].append(t1)  
        print("Enter heuristic values (node h):")
        for i in range(n):
            node, h = map(str, input().split())
            H_dist[node] = float(h)
    
        print("Heuristic values:", H_dist)
        Graph_nodes = graph
        print("Graph adjacency list:", dict(graph))
        start = input("Enter start node: ")
        goal = input("Enter goal node: ")
        aStarAlgo(start, goal)
```
<h2>Sample Graph</h2>
<hr>

![image](https://github.com/natsaravanan/19AI405FUNDAMENTALSOFARTIFICIALINTELLIGENCE/assets/87870499/b1377c3f-011a-4c0f-a843-516842ae056a)

<hr>
<h2>Input</h2>
<hr>
10 14 <br>
A B 6 <br>
A F 3 <br>
B D 2 <br>
B C 3 <br>
C D 1 <br>
C E 5 <br>
D E 8 <br>
E I 5 <br>
E J 5 <br>
F G 1 <br>
G I 3 <br>
I J 3 <br>
F H 7 <br>
I H 2 <br>
A 10 <br>
B 8 <br>
C 5 <br>
D 7 <br>
E 3 <br>
F 6 <br>
G 5 <br>
H 3 <br>
I 1 <br>
J 0 <br>
<hr>
<h2>Output</h2>
<hr>
<img width="1365" height="739" alt="image" src="https://github.com/user-attachments/assets/c9e11dd9-524b-4967-ae57-663f058d0aac" />

<hr>
Path found: ['A', 'F', 'G', 'I', 'J']
<hr>
