#include <iostream>
#include "graph.h"


void printVerticies(std::vector<std::string> verticies){
  std::cout << "[ ";
  for(auto v: verticies){
    std::cout << v << " ";
  }
  std::cout << "]" << std::endl;
}

Graph createGraph(void) {
  //Create an arbitary undirected, weighted graph
  Graph g;
  g.addVertex("alpha");
  g.addVertex("beta");
  g.addVertex("charlie");
  g.addVertex("delta");
  g.addVertex("echo");
  g.addEdge("alpha", "charlie", 2);
  g.addEdge("alpha", "echo", 3);
  g.addEdge("charlie", "beta", 5);

  return g;
}

int main (void) {
  Graph g = createGraph();

  std::cout << "Graph vertices: "; printVerticies(g.getVertices()); //Correct output: [ alpha beta charlie delta echo]
  std::cout << "Vertex 'alpha' exists: " << g.hasVertex("alpha") << std::endl;  //Correct output: true
  std::cout << "Vertex 'foxtrot' exists: " << g.hasVertex("foxtrot") << std::endl; //Correct output: false

  std::cout << "BFS at 'alpha': "; printVerticies(g.bfs("alpha"));  //Correct output: [ alpha charlie echo beta ]

  return 0;
}
