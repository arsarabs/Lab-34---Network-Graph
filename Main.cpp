#include <iostream>
#include <vector>
#include <queue>
#include <unordered_map>
using namespace std;

// Updated SIZE after deleting 2 nodes and adding 6 new nodes
const int SIZE = 13;  // Nodes 0 through 12

struct Edge {
    int src, dest, weight;
};

typedef pair<int, int> Pair;  // Alias 'Pair' for the pair<int, int> data type

class Graph {
public:
    // A vector of vectors of Pairs to represent an adjacency list
    vector<vector<Pair>> adjList;
    // Vector to store usernames corresponding to node IDs
    vector<string> userNames;

    // Graph Constructor
    Graph(vector<Edge> const& edges, vector<string> const& names) {
        // Initialize userNames
        if (names.size() != SIZE) {
            cerr << "Error: Number of usernames provided does not match SIZE." << endl;
            exit(EXIT_FAILURE);
        }
        userNames = names;

        // Resize the adjacency list to hold SIZE elements of type vector<Pair>
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

    // Print the graph's adjacency list with usernames
    void printGraph() {
        cout << "Social Network's Adjacency List:" << endl;
        for (int i = 0; i < adjList.size(); i++) {
            // Skip deleted nodes (1 and 3)
            if (i == 1 || i == 3) continue;

            cout << userNames[i] << " --> ";
            for (Pair v : adjList[i])
                cout << "(" << userNames[v.first] << ", " << v.second << ") ";
            cout << endl;
        }
    }

    // Depth-First Search (DFS) Traversal
    void depthFirstSearch(int start) {
        // Initialize a visited array
        vector<bool> visited(adjList.size(), false);

        cout << "DFS Traversal starting from " << userNames[start] << ": ";
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

        cout << "BFS Traversal starting from " << userNames[start] << ": ";

        while (!q.empty()) {
            // Dequeue a vertex from the queue and print it
            int current = q.front();
            q.pop();
            cout << userNames[current] << " ";

            // Get all adjacent vertices of the dequeued vertex
            // If an adjacent has not been visited and is not deleted, mark it visited and enqueue it
            for (auto& neighbor : adjList[current]) {
                int adjVertex = neighbor.first;
                if (!visited[adjVertex] && adjVertex != 1 && adjVertex != 3) {  // Skip deleted nodes
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
        // Skip deleted nodes
        if (v == 1 || v == 3) return;

        // Mark the current node as visited and print it
        visited[v] = true;
        cout << userNames[v] << " ";

        // Recur for all the vertices adjacent to this vertex
        for (auto& neighbor : adjList[v]) {
            int adjVertex = neighbor.first;
            if (!visited[adjVertex] && adjVertex != 1 && adjVertex != 3) {  // Skip deleted nodes
                _dfsUtil(adjVertex, visited);
            }
        }
    }
};

int main() {
    // Define usernames for each node (0 through 12)
    // Nodes 1 and 3 are deleted and hence have placeholder names
    vector<string> usernames = {
        "Alice",      // 0
        "Bob",        // 1 (deleted)
        "Charlie",    // 2
        "David",      // 3 (deleted)
        "Eve",        // 4
        "Frank",      // 5
        "Grace",      // 6
        "Heidi",      // 7
        "Ivan",       // 8
        "Judy",       // 9
        "Karl",       // 10
        "Leo",        // 11
        "Mallory"     // 12
    };

    // Creates a vector of graph edges/weights
    vector<Edge> edges = {
        // Existing edges after deleting nodes 1 and 3 with updated weights
        {0, 2, 10}, {2, 6, 5}, {5, 6, 7}, {4, 5, 12}, {2, 4, 6}, {2, 5, 9},

        // New edges involving added nodes with new weights
        {0, 7, 15}, {7, 8, 4}, {8, 9, 8}, {9, 10, 3}, {10, 11, 9}, {11, 12, 2},
        {12, 0, 11}, {7, 12, 6}, {8, 10, 7}, {5, 9, 10}
    };

    // Creates graph
    Graph graph(edges, usernames);

    // Prints adjacency list representation of graph
    graph.printGraph();

    // Perform Depth-First Search (DFS) starting from "Alice" (node 0)
    graph.depthFirstSearch(0);

    // Perform Breadth-First Search (BFS) starting from "Alice" (node 0)
    graph.breadthFirstSearch(0);

    return 0;
}
