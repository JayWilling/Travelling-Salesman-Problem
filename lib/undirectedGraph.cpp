#include "undirectedGraph.h"
#include <string>
#include <utility>
#include <cmath>

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
        edges[u] = std::unordered_set<std::pair<vertex, float>>();
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

        edges[u].insert(std::pair<vertex, float>(v, distance));
        edges[v].insert(std::pair<vertex, float>(u, distance));
    }
}

void undirected_graph<vertex>::remove_vertex(const vertex &u)
{
    vertices.erase(u);
    edges.erase(u);
}

void undirected_graph<vertex>::remove_edge(const vertex &u, const vertex &v)
{
    if (contains(u))
    {
        edges[u].erase(v);
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
            // With mst as a map<map<vertex, float>>
            // mst[u][v] = weights[u];
            // mst[v][u] = weights[u];
            mst[u].insert(v);
            mst[v].insert(u);
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
    return parent[i] = find_set(parent[i]);
}
// Method for Union (from GFG)
void union_set(int u, int v) {
    parent[u] = parent[v];
}

void undirected_graph<vertex>::find_odd_degrees() {
    for (vertex u : vertices) {
        int degree = get_degree(u);
        if (degree % 2 == 1) {
            odd_vertices.insert(u);
        }
    }
}

void undirected_graph<vertex>::perfect_matching() {
    // Very similar to the greedy approach used in prims
    // once we have found the vertices with odd degree we:
    //      Iterate over the odd degrees
    //          Start with any, it does not matter which as they will all be paired
    //      Then find the other vertex with odd degree which is closest
    //      These are then pushed to the mst to form a connected multigraph
    //          Not sure how to store as a multigraph with current setup but shouldn't be a problem.
    //          Because we need a connected multigraph, edges needs to be changed to a map<set<vertex>>
    find_odd_degrees();

    // Select the first vertex
    for (auto it = odd_vertices.begin(); it != odd_vertices.end(); it++) {
        vertex u = *it;
        float minWeight = std::numeric_limits<float>::infinity();
        // Poor practice but we'll make a copy of the iterator and advance that
        std::unordered_set<vertex>::iterator match = it;
        ++match;
        // We then loop over all the other odd vertices whilst tracking the new minimum
        vertex minVertex;
        for (;match != odd_vertices.end(); match++) {
            // Check if the edge between the two vertices is the cheapest
            if (edges[u][*match] < minWeight) {
                minVertex = *match;
                minWeight = edges[u][*match];
            }
        }
        // Add the new edge/matching to the multigraph
        mst[u].insert(minVertex);
        mst[minVertex].insert(u);

        // Remove both vertices from the odd_vertices list so they are not considered again
        odd_vertices.erase(u);
        odd_vertices.erase(minVertex);
        // Because of this a manual loop (not a for loop on the iterator) may be needed to avoid any null references

    }
}

void undirected_graph<vertex>::euler_tour() {
    // All we need to do is walk through the new multigraph, adding each vertex to a list (this will be our tour)
    // mst[] is the adjacency list we will work from
    //      Make a temp copy of the mst[] first
    // Pick a starting vertex (doesn't matter which)
    // Loop until no neighbours available for the current vertex. We can also have a counter to make sure all vertices were reached.
    //      Add a neighbour to the path and remove the corresponding edge from the mstCopy
    //      Set that neighbour to the new current vertex

    // Make copy of mst
    std::unordered_map<vertex, std::unordered_set<vertex>> tmpMstCopy = mst;

    vertex current = *vertices.begin();
    path.push_back(current);
    vertex neighbour;
    // While the current vertex has neighbours
    while (tmpMstCopy[current].size() > 0) {
        // Pick a neihbour and add it to the path
        neighbour = *tmpMstCopy[current].begin();
        tmpMstCopy[neighbour].erase(current);
        tmpMstCopy[current].erase(neighbour);             // This may be a problem
        current = neighbour;

        path.push_back(current);
    }

}

void undirected_graph<vertex>::make_hamiltonian() {
    // Track the visited vertices
    // Iterate over the path
    //      If the next vertex has already been visited, remove it from the path and continue

    // Initialise vertices as unvisited
    std::unordered_map<vertex, bool> visited;
    for (vertex u : vertices) {
        visited[u] = false;
    }

    std::vector<vertex> newPath;                    // Creating a new path, but this can be done with iterators in-place in the main path member

    // Visit the first vertex in the path
    vertex current = path[0];
    visited[current] = true;
    newPath.push_back(current);
    // Walk through the path marking each vertex as visited,
    // If we encounter a visited vertex we look at the next vertex and continue on if not visited
    for (int i = 1; i < path.size(); i++) {
        current = path[i];
        if(!visited[current]) {
            newPath.push_back(current);
            visited[current] = true;
        }
    }

    // Save the new path
    path = newPath;

}