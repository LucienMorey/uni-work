#include <iostream>
#include <vector>
#include <stack>
#include <set>
#include <queue>
#include <map>
#include <algorithm>

using namespace std;

typedef vector<vector<int> > AdjacencyList;

vector<int> reconstruct_path(map<int,int> parents, int goal) {
    vector<int> path;
    path.push_back(goal);
    while (parents.find(path.back()) != parents.end()) {
        path.push_back(parents.at(path.back()) );
    }
    std::reverse(path.begin(), path.end());
    return path;
}

void print_path(vector<int> path) {
    for (vector<int>::iterator n = path.begin();
         n != path.end(); n++) {
        cout << *n << " ";
    }
    cout << endl;
}


vector<int> breadth_first_search(AdjacencyList& graph, int start, int goal) {
    cout << "Breadth first search exploration order: ";
    // Additional search variables
    int current;
    map<int,int> parents;
    // create empty set S
    set<int> visited;
    // create empty queue Q
    queue<int> q;
    // add root to S
    visited.insert(start);
    // Q.enqueue(root)
    q.push(start);
    // while Q is not empty:
    while (!q.empty()) {
        // current = Q.dequeue()
        current = q.front();
        q.pop();
        // Printing exploration order
        cout << current << " ";
        // if current is the goal:
        if (current == goal) {
            // return current
            cout << endl;
            return reconstruct_path(parents, current);
        }
        // for each node n that is adjacent to current:
        for (vector<int>::iterator n = graph.at(current).begin();
             n != graph.at(current).end(); n++) {
            // if n is not in S:
            if (visited.find(*n) == visited.end()) {
                // add n to S
                visited.insert(*n);
                // n.parent = current
                parents[*n] = current;
                // Q.enqueue(n)
                q.push(*n);
            }
        }
    }
}

vector<int> depth_first_search(AdjacencyList& graph, int root, int goal) {
    cout << "Depth first search exploration order: ";
    // Additional search variables
    int current;
    map<int,int> parents;
    // create empty set S

    set<int> s;
    // create empty queue Q
    stack<int> q;
    // add root to S
    s.insert(root);
    // Q.enqueue(root)
    q.push(root);
    // while Q is not empty:
    while (!q.empty()) {
        // current = Q.dequeue()
        current = q.top();
        q.pop();
        // Printing exploration order
        cout << current << " ";
        // if current is the goal:
        if (current == goal) {
            // return current
            cout << endl;
            return reconstruct_path(parents, current);
        }
        // for each node n that is adjacent to current:
        for (vector<int>::iterator n = graph.at(current).begin();
             n != graph.at(current).end(); n++) {
            // if n is not in S:
            if (s.find(*n) == s.end()) {
                // add n to S
                s.insert(*n);
                // n.parent = current
                parents[*n] = current;
                // Q.enqueue(n)
                q.push(*n);
            }
        }
    }
}


int main () {
    // Build the graph
    AdjacencyList house_graph = {
        {0, 1},
        {0, 4},
        {0, 3, 4},
        {2, 4, 5},
        {1, 2, 3},
        {3, 6},
        {5}
    };

    vector<int> path;

    path = breadth_first_search(house_graph, 5, 1);
    cout << "Path found by breadth first search: ";
    print_path(path);

    path = depth_first_search(house_graph, 5, 1);
    cout << "Path found by depth first search: ";
    print_path(path);

    return 0;
}
