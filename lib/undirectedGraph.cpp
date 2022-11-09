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

int undirected_graph<vertex>::get_degree(const vertex &u) {
    if (contains(u)) {
        return edges[u].size();
    }
    return 0;
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

vertex undirected_graph<vertex>::get_min_vertex(std::unordered_map<vertex, float> weights, std::unordered_map<vertex, bool> visited) {

    float min = std::numeric_limits<float>::infinity();
    vertex minVertex;

    for (vertex u : vertices) {
        if (!visited[u] && weights[u] < min) {
            min = weights[u];
            minVertex = u;
        }
    }

    return minVertex;

}

void undirected_graph<vertex>::prims_mst() {

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
    minEdge[startVertex] = startVertex; // As minEdge[startVertex] != startVertex, we will use this to identify the root during construction later.

    // Loop for each vertex
    for (int i = 0; i < vertices.size(); i++) {

        // Select the next closest vertex (lowest weight as set in the weights[])
        vertex v = get_min_vertex(weights, visited);
        visited[v] = true;

        // Neighbour iterator approach
        for (auto it = edges[v].begin(); it != edges[v].end(); it++) {
            vertex u = it->first;
            float uWeight = it->second;
            if (!visited[u] && uWeight < weights[u]) {
                weights[u] = uWeight;
                minEdge[u] = v;
            }
        }

        // Normal approach
        for (vertex u : vertices) {
            // TODO: Need function to verify an edge exists
            if (!visited[u] && edges[v][u] < weights[u]) {
                // A cheaper edge has been found
                weights[u] = edges[v][u];
                minEdge[u] = v;
            }
        }
    }

    // Build the MST now that minimum edges are "selected"
    // I assume this can replace the existing edges, for now it is stored separately
    for (vertex u : vertices) {
        vertex v = minEdge[u];
        if (u != v) {
            mst[u][v] = weights[u];
            mst[v][u] = weights[u];
        }
    }

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
    for (vertex u : vertices) {
        int degree = get_degree(u);
        if (degree % 2 == 1) {
            odd_vertices.push_back(u);
        }
    }
}

void undirected_graph<vertex>::perfect_matching() {
    // Very similar to the greedy approach used in prims
    // once we have found the odd degrees we:
    //      Iterate over the odd degrees
    //          Start with any, it does not matter which as they will all be paired
    //      Then find the other odd degree which is closest
    //      These are then pushed to the mst to form a connected multigraph
    //          Not sure how to store as a multigraph with current setup but shouldn't be a problem.
}