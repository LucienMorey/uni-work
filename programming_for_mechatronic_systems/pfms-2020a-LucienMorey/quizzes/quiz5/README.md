Quiz 4
======

Part A
------

1) TASK: A class representing an undirected, weighted graph has been partially defined in [graph.h](./a/graph.h). Use your knowledge of the `std::map` STL to complete the methods `addVertex()`, `hasVertex()`, and `getVertices()`.
*see graph.cpp

2) TASK: Compile the program, execute, and check that the program outputs all the graph's vertices as expected.

3) TASK: Complete the method `bfs()` in [graph.cpp](./a/graph.cpp) so that it performs a breadth first search on the entire graph. Check that the output of the BFS is as expected.
*see graph.cpp

4) QUESTION: What is the worst case computational complexity of BFS?
* In a worst case scenario the BFS will have a complexity of o(V^2) where V is the number of verticies

5) QUESTION: Which algorithm(s) can find the shortest path between two vertices in a graph that has weights on edges?
* Dijkstra’s shortest path algorithm

Part B
------

1) TASK: In [main.cpp](./b/main.cpp) add the appropriate thread instruction(s) to ensure that `main()` waits for the threads to finalise.
* added thread.join() calls.

2) TASK: Control access to the `std::queue` in `TDataBuffer` by adding an appropriate thread mechanism to the struct.
* added mutex to structure

3) TASK: Use the mechanism in threads `fibonacci` and `printToTerminal`.
* added lockguard calls in main to preserve data integrity

4) TASK: Implement the mechanism to make the `consumer` thread wait for new data?
* added condVar to structure and implemented wait until data is there

5) TASK: Rather than making `TDataBuffer` variable `sequence` global, change it's scope so that both threads had access to this shared resource
* made sequence local and passed by reference to the threads so they use shared memory
