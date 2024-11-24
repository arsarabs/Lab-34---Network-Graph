#include <iostream>
#include <vector>
#include <queue>
#include <climits>    // For INT_MAX
#include <algorithm>  // For reverse and sort
using namespace std;

// Total number of nodes (users) in the social network
const int SIZE = 13;  // Nodes 0 through 12

// Structure to represent an edge (friendship) with source, destination, and friendship strength
struct Edge {
    int src, dest, weight;
};

// Alias 'Pair' for a pair of integers (used in priority queue for Dijkstra's)
typedef pair<int, int> Pair;

// Structure to represent an edge for MST (src, dest, weight)
struct MST_Edge {
    int src, dest, weight;
};

// Comparator for sorting edges in descending order of weight (for MaxST)
bool compareEdges(const MST_Edge& a, const MST_Edge& b) {
    return a.weight > b.weight;
}

// Union-Find (Disjoint Set) Structure for Kruskal's Algorithm
struct UnionFind {
    vector<int> parent;
    UnionFind(int size) : parent(size, -1) {}

    // Find the set of an element with path compression
    int findSet(int i) {
        if (parent[i] == -1)
            return i;
        return parent[i] = findSet(parent[i]);
    }

    // Union two sets
    bool unionSet(int x, int y) {
        int s1 = findSet(x);
        int s2 = findSet(y);
        if (s1 != s2) {
            parent[s1] = s2;
            return true;
        }
        return false;
    }
};

// Graph class to represent the social network
class Graph {
public:
    // Adjacency list where each node has a list of pairs (neighbor, friendship strength)
    vector<vector<Pair>> adjList;

    // Vector to store usernames corresponding to node IDs
    vector<string> userNames;

    // Constructor to initialize the graph with edges and usernames
    Graph(vector<Edge> const& edges, vector<string> const& names) {
        // Validate that the number of usernames matches the number of nodes
        if (names.size() != SIZE) {
            cerr << "Error: Number of usernames provided does not match SIZE." << endl;
            exit(EXIT_FAILURE);
        }
        userNames = names;

        // Resize the adjacency list to hold SIZE elements
        adjList.resize(SIZE);

        // Add edges to the undirected graph
        for (auto& edge : edges) {
            int src = edge.src;
            int dest = edge.dest;
            int weight = edge.weight;

            // Insert the edge in both directions since the graph is undirected
            adjList[src].push_back(make_pair(dest, weight));
            adjList[dest].push_back(make_pair(src, weight));
        }
    }

    // Function to print the social network's adjacency list with usernames and friendship strengths
    void printGraph() {
        cout << "\nSocial Network Topology:\n";
        cout << "===============================\n";
        for (int i = 0; i < adjList.size(); i++) {
            // Skip deleted nodes (1 and 3)
            if (i == 1 || i == 3) continue;

            cout << "User " << i << " (" << userNames[i] << ") connects to:\n";
            for (Pair v : adjList[i]) {
                // Skip connections to deleted nodes
                if (v.first == 1 || v.first == 3) continue;
                cout << "  -> User " << v.first << " (" << userNames[v.first] << ") - Friendship Strength: " << v.second << "\n";
            }
            cout << endl;
        }
        cout << "===============================\n";
    }

    // Depth-First Search (DFS) Traversal
    void depthFirstSearch(int start) {
        // Check if the start node is deleted
        if (start == 1 || start == 3) {
            cout << "User " << start << " (" << userNames[start] << ") is deleted and cannot be used for traversal.\n";
            return;
        }

        // Initialize a visited array
        vector<bool> visited(adjList.size(), false);

        cout << "\nNetwork Trace (DFS) from User " << start << " (" << userNames[start] << "):\n";
        cout << "Purpose: Tracing possible contamination paths through the network\n";
        cout << "=======================================\n";

        // Call the recursive helper function
        _dfsUtil(start, visited);

        cout << "=======================================\n";
    }

    // Breadth-First Search (BFS) Traversal
    void breadthFirstSearch(int start) {
        // Check if the start node is deleted
        if (start == 1 || start == 3) {
            cout << "User " << start << " (" << userNames[start] << ") is deleted and cannot be used for traversal.\n";
            return;
        }

        // Initialize a visited array
        vector<bool> visited(adjList.size(), false);

        // Create a queue for BFS
        queue<int> q;

        // Mark the current node as visited and enqueue it
        visited[start] = true;
        q.push(start);

        cout << "\nLayer-by-Layer Network Inspection (BFS) from User " << start << " (" << userNames[start] << "):\n";
        cout << "Purpose: Analyzing service areas by distance from source\n";
        cout << "================================================\n";

        while (!q.empty()) {
            // Dequeue a vertex from the queue and print it
            int current = q.front();
            cout << "Checking User " << current << " (" << userNames[current] << ")\n";
            q.pop();

            // Get all adjacent vertices of the dequeued vertex
            // If an adjacent has not been visited and is not deleted, mark it visited and enqueue it
            for (auto& neighbor : adjList[current]) {
                int adjVertex = neighbor.first;
                int strength = neighbor.second;
                if (!visited[adjVertex] && adjVertex != 1 && adjVertex != 3) {  // Skip deleted nodes
                    visited[adjVertex] = true;
                    q.push(adjVertex);
                    cout << "  -> Next service area: User " << adjVertex << " (" << userNames[adjVertex] << ") - Friendship Strength: " << strength << "\n";
                }
            }
        }
        cout << "================================================\n";
    }

    // Shortest Path using Dijkstra's Algorithm
    void shortestPath(int start, int end) {
        // Check if start or end nodes are deleted
        if (start == 1 || start == 3 || end == 1 || end == 3) {
            cout << "One or both of the selected users are deleted and cannot be used for pathfinding.\n";
            return;
        }

        // Initialize distances to all nodes as infinity
        vector<int> distances(adjList.size(), INT_MAX);
        // To store the path
        vector<int> previous(adjList.size(), -1);
        // Min-heap priority queue: (distance, vertex)
        priority_queue<Pair, vector<Pair>, std::greater<Pair>> pq;

        // Distance to start is 0
        distances[start] = 0;
        pq.push(make_pair(0, start));

        while (!pq.empty()) {
            Pair currentPair = pq.top();
            pq.pop();
            int currentDistance = currentPair.first;
            int currentVertex = currentPair.second;

            // If we reach the end node, we can stop
            if (currentVertex == end) break;

            // Iterate over neighbors
            for (auto& neighbor : adjList[currentVertex]) {
                int adjVertex = neighbor.first;
                int weight = neighbor.second;

                // Skip deleted nodes
                if (adjVertex == 1 || adjVertex == 3) continue;

                // If a shorter path is found
                if (currentDistance + weight < distances[adjVertex]) {
                    distances[adjVertex] = currentDistance + weight;
                    previous[adjVertex] = currentVertex;
                    pq.push(make_pair(distances[adjVertex], adjVertex));
                }
            }
        }

        // Check if a path exists
        if (distances[end] == INT_MAX) {
            cout << "No path exists from User " << start << " (" << userNames[start] << ") to User " << end << " (" << userNames[end] << ").\n";
            return;
        }

        // Reconstruct the path from end to start
        vector<int> path;
        int crawl = end;
        path.push_back(crawl);
        while (previous[crawl] != -1) {
            path.push_back(previous[crawl]);
            crawl = previous[crawl];
        }

        // Reverse the path to get from start to end
        reverse(path.begin(), path.end());

        // Print the path
        cout << "Shortest path from User " << start << " (" << userNames[start] << ") to User " << end << " (" << userNames[end] << ") with total Friendship Strength " << distances[end] << ":\n";
        cout << "  ";
        for (size_t i = 0; i < path.size(); i++) {
            cout << "User " << path[i] << " (" << userNames[path[i]] << ")";
            if (i != path.size() - 1) cout << " -> ";
        }
        cout << "\n";
    }

    // Function to compute the Maximum Spanning Tree (MST) using Kruskal's Algorithm
    void maximumSpanningTree() {
        // Step 1: Gather all edges, avoiding duplicates and deleted nodes
        vector<MST_Edge> allEdges;
        for (int src = 0; src < adjList.size(); src++) {
            if (src == 1 || src == 3) continue; // Skip deleted nodes
            for (auto& pair : adjList[src]) {
                int dest = pair.first;
                int weight = pair.second;
                if (src < dest && dest != 1 && dest != 3) { // Avoid duplicates and skip deleted nodes
                    allEdges.push_back(MST_Edge{ src, dest, weight });
                }
            }
        }

        // Step 2: Sort all edges in descending order of weight
        sort(allEdges.begin(), allEdges.end(), compareEdges);

        // Step 3: Initialize Union-Find structure
        UnionFind uf(SIZE);

        // Step 4: Iterate through sorted edges and construct MST
        vector<MST_Edge> mst;
        int mstWeight = 0;

        for (auto& edge : allEdges) {
            if (uf.unionSet(edge.src, edge.dest)) {
                mst.push_back(edge);
                mstWeight += edge.weight;
            }
            // If MST has (SIZE - number of deleted nodes - 1) edges, we can stop early
            if (mst.size() == (SIZE - 2 - 1)) { // SIZE - deleted nodes -1
                break;
            }
        }

        // Step 5: Print the MST
        cout << "\nMaximum Spanning Tree (MST) using Kruskal's Algorithm:\n";
        cout << "===============================================\n";
        cout << "Edges in the MST:\n";
        for (auto& edge : mst) {
            cout << "  -> User " << edge.src << " (" << userNames[edge.src] << ") -- User " << edge.dest << " (" << userNames[edge.dest] << ") == Friendship Strength: " << edge.weight << "\n";
        }
        cout << "Total Friendship Strength of MST: " << mstWeight << "\n";
        cout << "===============================================\n";
    }

private:
    // Helper function for DFS
    void _dfsUtil(int v, vector<bool>& visited) {
        // Mark the current node as visited and print it
        visited[v] = true;
        cout << "Inspecting User " << v << " (" << userNames[v] << ")\n";

        // Recur for all the vertices adjacent to this vertex
        for (auto& neighbor : adjList[v]) {
            int adjVertex = neighbor.first;
            int strength = neighbor.second;

            // Skip deleted nodes and already visited nodes
            if (!visited[adjVertex] && adjVertex != 1 && adjVertex != 3) {
                cout << "  -> Potential spread to User " << adjVertex << " (" << userNames[adjVertex] << ") - Friendship Strength: " << strength << "\n";
                _dfsUtil(adjVertex, visited);
            }
        }
    }
};

// Driver program to demonstrate the MST functionality
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

    // Create a vector of graph edges (friendships) with friendship strengths
    vector<Edge> edges = {
        // Existing edges after deleting nodes 1 and 3 with updated weights
        {0, 2, 10}, {2, 6, 5}, {5, 6, 7}, {4, 5, 12}, {2, 4, 6}, {2, 5, 9},

        // New edges involving added nodes with new weights
        {0, 7, 15}, {7, 8, 4}, {8, 9, 8}, {9, 10, 3}, {10, 11, 9}, {11, 12, 2},
        {12, 0, 11}, {7, 12, 6}, {8, 10, 7}, {5, 9, 10}
    };

    // Instantiate the Graph with edges and usernames
    Graph graph(edges, usernames);

    // Display the social network's adjacency list
    graph.printGraph();
    cout << endl;

    // Perform Depth-First Search (DFS) starting from user 0 (Alice)
    graph.depthFirstSearch(0);
    cout << endl;

    // Perform Breadth-First Search (BFS) starting from user 0 (Alice)
    graph.breadthFirstSearch(0);
    cout << endl;

    // Demonstrate Shortest Path Functionality

    // Example 1: Find shortest path from user 0 (Alice) to user 11 (Leo)
    int startNode = 0;
    int endNode = 11;
    graph.shortestPath(startNode, endNode);
    cout << endl;

    // Example 2: Find shortest path from user 4 (Eve) to user 10 (Karl)
    startNode = 4;
    endNode = 10;
    graph.shortestPath(startNode, endNode);
    cout << endl;

    // Example 3: Attempt to find a path involving a deleted user (user 1: Bob)
    startNode = 0;
    endNode = 1;  // Node 1 is deleted
    graph.shortestPath(startNode, endNode);
    cout << endl;

    // Demonstrate Maximum Spanning Tree (MST) Functionality
    graph.maximumSpanningTree();
    cout << endl;

    return 0;
}
