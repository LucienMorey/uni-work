Quiz 4
======

Part A
------

1) TASK: A class representing an undirected, weighted graph has been partially defined in [graph.h](./a/graph.h). Use your knowledge of the `std::map` STL to complete the methods `addVertex()`, `hasVertex()`, and `getVertices()`.

2) TASK: Compile the program, execute, and check that the program outputs all the graph's vertices as expected.

3) TASK: Complete the method `bfs()` in [graph.cpp](./a/graph.cpp) so that it performs a breadth first search on the entire graph. Check that the output of the BFS is as expected.

4) QUESTION: What is the worst case computational complexity of BFS?  

Expected time complexoity and worst case is O(V+E).  Other time complexitys make this process faster such as
no node (vertex) nor edge being visited more than once, henceforth O(|V| + |E|). or even by using Dijkstra’s shortest path algorithm, we can get a shortest path in O(E + VLogV) time.

5) QUESTION: Which algorithm(s) can find the shortest path between two vertices in a graph that has weights on edges?
The most important algorithms for solving this problem are:

- Dijkstra's algorithm solves the single-source shortest path problem with non-negative edge weight.
- Bellman–Ford algorithm solves the single-source problem if edge weights may be negative.
- A* search algorithm solves for single pair shortest path using heuristics to try to speed up the search.
- Floyd–Warshall algorithm solves all pairs shortest paths.
- Johnson's algorithm solves all pairs shortest paths, and may be faster than Floyd–Warshall on sparse graphs.
- Viterbi algorithm solves the shortest stochastic path problem with an additional probabilistic weight on each node.

Part B
------

1) TASK: In [main.cpp](./b/main.cpp) add the appropriate thread instruction(s) to ensure that `main()` waits for the threads to finalise.

2) TASK: Control access to the `std::queue` in `TDataBuffer` by adding an appropriate thread mechanism to the struct.

3) TASK: Use the mechanism in threads `fibonacci` and `printToTerminal`.

4) TASK: Implement the mechanism to make the `consumer` thread wait for new data?

5) TASK: Rather than making `TDataBuffer` variable `sequence` global, change it's scope so that both threads had access to this shared resource
