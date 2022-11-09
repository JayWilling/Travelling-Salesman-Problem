#include "undirectedGraph.h"
#include <string>
#include <utility>
#include <cmath>
#include <limits>
#include <queue>

typedef std::pair<float, vertex> minHeap;

vertex::vertex(std::string location, int x, int y)
{
    this->location = location;
    this->xPos = x;
    this->yPos = y;
}

// Not sure if the template line is needed before each function but will see if it works
// template<typename vertex>
bool undirected_graph<vertex>::contains(const vertex &u) const
{
    return vertices.count(u) > 0;
}

void undirected_graph<vertex>::add_vertex(const vertex &u)
{
    if (!contains(u))
    {
        vertices.insert(u);
        // edges[u] = std::unordered_set<vertex>();
        edges[u] = std::unordered_map<vertex, float>();
    }
}

// Determine the actual distance between two points given their location
float undirected_graph<vertex>::calculate_weight(const vertex &u, const vertex &v)
{
    return std::sqrt(std::pow(u.xPos - v.xPos, 2) - std::pow(u.yPos - v.yPos, 2));
}

// Adding two edges to the graph, one in either direction
void undirected_graph<vertex>::add_edge(const vertex &u, const vertex &v)
{
    if (contains(u) && contains(v) && u != v)
    {
        // Calculate weight/distance between locations first
        float distance = calculate_weight(u, v);

        // edges[u].insert(v);
        // edges[v].insert(u);
        float newWeight = calculate_weight(u, v);
        edges[u][v] = newWeight;
    }
}

// Removing vertex removes edges
void undirected_graph<vertex>::remove_vertex(const vertex &u)
{
    for (vertex v : vertices) {
        remove_edge(u, v);
    }
    vertices.erase(u);

}

// Removing edge must remove it from both directions
void undirected_graph<vertex>::remove_edge(const vertex &u, const vertex &v)
{
    if (contains(u) && contains(v))
    {
        edges[u].erase(v);
        edges[v].erase(u);
    }
}

// std::pair<vertex, vertex> undirected_graph<vertex>::get_min_weight(std::unordered_set<vertex> &visited) {

//     for (vertex u : vertices) {
//         if (visited.count(u) < 1) {
//             for (std::pair<vertex, float> x : edges[u]) {

//             }
//         }
//     }

// }

void undirected_graph<vertex>::prims_mst() {
    // Define a list of visisted vertices
    std::unordered_map<vertex, bool> visited;
    std::unordered_map<vertex, std::pair<float, vertex>> minEdges;
    std::pair<float, vertex> currentMinimumEdge;
    std::priority_queue<minHeap, std::vector<minHeap>, std::greater<minHeap>> pq;

    // Set minEdge values to infinity
    for (vertex u : vertices) {
        minEdges[u] = std::pair<float, vertex>(0, NULL);
    }

    // Visit the "first" vertex and add edges to minEdge
    auto vertexIt = vertices.begin();
    visited[*vertexIt] = true;
    // TODO: Add vertex to tree

    // Update minimum cost to get to each unvisited vertex
    for (auto edge : edges[*vertexIt]) {
        minEdges[edge.first] = std::pair<float, vertex>(edge.second, *vertexIt);
        pq.push(minEdges[edge.first]);
        // if (minEdges[edge.first].first < currentMinimumEdge.first) {
        //     currentMinimumEdge = minEdges[edge.first];
        // }
    }

    // Iterate over the currentMinimumEdge until all vertices visited
    while (!pq.empty()) {
        // TODO: Add edge and vertex to tree
        std::pair<float, vertex> currentEdge = pq.top();
        pq.pop();
        if (!visited[currentEdge.second]) {
            visited[currentEdge.second] = true;
            for (auto edge : edges[currentMinimumEdge.second]) {
                minEdges[edge.first] = std::pair<float, vertex>(edge.second, *vertexIt);
                pq.push(minEdges[edge.first]);
            }
        }
    }

    // REDO
    std::unordered_map<vertex, bool> visited;
    std::unordered_map<vertex, vertex> minEdge;     // Store the lowest weight "parent" for the given vertex key
    std::unordered_map<vertex, float> weights;      // Store the lowest distance to get to the given vertex key - minEdge and weights updated at the same time

    // Set weights to infinity
    for (vertex u : vertices) {
        weights[u] = std::numeric_limits<float>::infinity();
        visited[u] = false;
    }

    // Set the starting point for the MST
    vertex startVertex = *vertices.begin();
    visited[startVertex] = true;
    weights[startVertex] = 0.0;

}

void undirected_graph<vertex>::kruskals_mst() {
    // Need something to store pairs
    std::vector<std::vector<int>> edgeList;

    // Sort edges by weight
    sort(edgeList.begin(), edgeList.end());
    
    // Pick Smallest Edge
    //for (int i = 0; i < )

    // If edge connects two nodes already in the MST, ignore it

    // Keep going til all nodes are in the tree
}

// Method for Find (from GFG)
int find_set(int i) {
    // Note to self: define parent
    // if i is the parent of itself
    if (i == parent[i]) {
        return i;
    }
    // if not, then i is not the representative of the set
    // use recursion on its parent
    return parent[i] = find(parent[i]);
}
// Method for Union (from GFG)
void union_set(int u, int v) {
    parent[u] = parent[v];
}

void undirected_graph<vertex>::find_odd_degrees() {

}