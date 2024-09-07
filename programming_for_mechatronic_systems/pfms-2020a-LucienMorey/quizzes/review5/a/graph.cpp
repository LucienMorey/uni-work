#include <set>
#include <queue>
#include "graph.h"
#include <iostream>

void Graph::addVertex(vertex v) {

  //Insert a vertex with no edges
}

bool Graph::hasVertex(vertex v) {
  //Check if the vertex v, exists in the graph
  return true;
}

void Graph::addEdge(vertex u, vertex v, weight w) {
  //Assumes that u & v have already been added to the graph
  //We need to record the same edge twice as this is an undirected graph
 // weightedGraph_.at(u).insert({v, w}); //Inserting an edge between u and v, with weight w
 // weightedGraph_.at(v).insert({u, w}); //Inserting an edge between v and u, with weight w
}


std::vector<Graph::vertex> Graph::getVertices(void) {
  //Iterate through the weightedGraph_ and push back each vertex to the vertices vector

    // Declaring iterator to a vector
   std::vector<Graph::vertex>::iterator ptr;

    // Displaying vector elements using begin() and end()
    std::cout << "The vector elements are : ";
    for (ptr = weightedGraph_.begin(); ptr < weightedGraph_.end(); ptr++)
        std::cout << ptr << " ";
  return std::vector<Graph::vertex>();
}

std::vector<Graph::vertex> Graph::bfs(vertex start) {
  //Perform a breadth first search on the entire graph
  return std::vector<Graph::vertex>();
}
