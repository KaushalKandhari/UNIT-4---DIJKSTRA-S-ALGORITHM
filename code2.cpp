#include <iostream>
#include <vector>
#include <queue>
#include <limits>

using namespace std;

// Structure to represent a node and its associated weight
struct Edge {
    int destination;
    int weight;
};

// Dijkstra's Algorithm
vector<int> dijkstra(const vector<vector<Edge>>& graph, int startNode, int numNodes) {
    vector<int> dist(numNodes, numeric_limits<int>::max()); // Initialize distances to infinity
    dist[startNode] = 0; // Set distance to the starting node as 0
    vector<bool> visited(numNodes, false); // Keep track of visited nodes
    priority_queue<pair<int, int>, vector<pair<int, int>>, greater<pair<int, int>>> pq; // Min heap to store nodes by their distance
    pq.push({0, startNode}); // Push the starting node with distance 0

    while (!pq.empty()) {
        int u = pq.top().second;
        pq.pop();

        if (visited[u]) continue; // Skip if the node has already been visited
        visited[u] = true;

        for (const Edge& edge : graph[u]) {
            int v = edge.destination;
            int weight = edge.weight;

            // Relaxation step
            if (dist[v] > dist[u] + weight) {
                dist[v] = dist[u] + weight;
                pq.push({dist[v], v});
            }
        }
    }

    return dist;
}

int main() {
    // Your main function code remains the same as in the previous approach
    // ...
}