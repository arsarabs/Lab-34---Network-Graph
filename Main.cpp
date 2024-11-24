#include <iostream>
#include <vector>
#include <queue>    // Included for BFS implementation
using namespace std;

const int SIZE = 7;

struct Edge {
    int src, dest, weight;
};

typedef pair<int, int> Pair;  // Creates alias 'Pair' for the pair<int,int> data type

class Graph {
public:
    // A vector of vectors of Pairs to represent an adjacency list
    vector<vector<Pair>> adjList;

    // Graph Constructor
    Graph(vector<Edge> const& edges) {
        // Resize the vector to hold SIZE elements of type vector<Pair>
        adjList.resize(SIZE);

        // Add edges to the undirected graph
        for (auto& edge : edges) {
            int src = edge.src;
            int dest = edge.dest;
            int weight = edge.weight;

            // Insert at the end
            adjList[src].push_back(make_pair(dest, weight));
            // For an undirected graph, add an edge from dest to src also
            adjList[dest].push_back(make_pair(src, weight));
        }
    }

    // Print the graph's adjacency list
    void printGraph() {
        cout << "Graph's adjacency list:" << endl;
        for (int i = 0; i < adjList.size(); i++) {
            cout << i << " --> ";
            for (Pair v : adjList[i])
                cout << "(" << v.first << ", " << v.second << ") ";
            cout << endl;
        }
    }

    // Depth-First Search (DFS) Traversal
    void depthFirstSearch(int start) {
        // Initialize a visited array
        vector<bool> visited(adjList.size(), false);

        cout << "DFS Traversal starting from node " << start << ": ";
        // Call the recursive helper function
        _dfsUtil(start, visited);
        cout << endl;
    }

    // Breadth-First Search (BFS) Traversal
    void breadthFirstSearch(int start) {
        // Initialize a visited array
        vector<bool> visited(adjList.size(), false);

        // Create a queue for BFS
        queue<int> q;

        // Mark the current node as visited and enqueue it
        visited[start] = true;
        q.push(start);

        cout << "BFS Traversal starting from node " << start << ": ";

        while (!q.empty()) {
            // Dequeue a vertex from the queue and print it
            int current = q.front();
            q.pop();
            cout << current << " ";

            // Get all adjacent vertices of the dequeued vertex
            // If an adjacent has not been visited, mark it visited and enqueue it
            for (auto& neighbor : adjList[current]) {
                int adjVertex = neighbor.first;
                if (!visited[adjVertex]) {
                    visited[adjVertex] = true;
                    q.push(adjVertex);
                }
            }
        }
        cout << endl;
    }

private:
    // Helper function for DFS
    void _dfsUtil(int v, vector<bool>& visited) {
        // Mark the current node as visited and print it
        visited[v] = true;
        cout << v << " ";

        // Recur for all the vertices adjacent to this vertex
        for (auto& neighbor : adjList[v]) {
            int adjVertex = neighbor.first;
            if (!visited[adjVertex]) {
                _dfsUtil(adjVertex, visited);
            }
        }
    }
};

int main() {
    // Creates a vector of graph edges/weights
    vector<Edge> edges = {
        // (x, y, w) -> edge from x to y having weight w
        {0,1,12},{0,2,8},{0,3,21},{2,3,6},{2,6,2},{5,6,6},{4,5,9},{2,4,4},{2,5,5}
    };

    // Creates graph
    Graph graph(edges);

    // Prints adjacency list representation of graph
    graph.printGraph();

    // Perform Depth-First Search (DFS) starting from node 0
    graph.depthFirstSearch(0);

    // Perform Breadth-First Search (BFS) starting from node 0
    graph.breadthFirstSearch(0);

    return 0;
}
