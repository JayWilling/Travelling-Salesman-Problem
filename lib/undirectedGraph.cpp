#include "undirectedGraph.h"

vertex::vertex(std::string location, int x, int y)
{
    this->location = location;
    this->xPos = x;
    this->yPos = y;
}

vertex::vertex() {}

undirected_graph<vertex>::undirected_graph() {}

template<typename vertex>
bool undirected_graph<vertex>::contains(const vertex &u) const
{
    return vertices.count(u) > 0;
}

// Returns the degree of vertices in the minimum spanning tree
template<typename vertex>
int undirected_graph<vertex>::get_degree(const vertex &u) {
    if (contains(u)) {
        return mst[u].size();
    }
    return 0;
}

void undirected_graph<vertex>::add_vertex(vertex &u)
{
    if (!contains(u))
    {
        vertices.insert(u);
        edges[u] = std::unordered_map<vertex, float>();

        // As a complete graph is built, adding a new vertex adds edges to all pre-existing vertices
        for (vertex v : vertices) {
            add_edge(u, v);
        }
    }
}

// Determine the actual distance between two points given their location
float undirected_graph<vertex>::calculate_weight(const vertex &u, const vertex &v)
{

    int dx = std::pow((float)(u.xPos - v.xPos), 2);
    int dy = std::pow((float)(u.yPos - v.yPos), 2);

    return (float)std::sqrt(dx + dy);
}

// Adding two edges to the graph, one in either direction
template<typename vertex>
void undirected_graph<vertex>::add_edge(const vertex &u, const vertex &v)
{
    if (contains(u) && contains(v))
    {
        // Calculate weight/distance between locations first
        float newWeight = calculate_weight(u, v);
        edges[u][v] = newWeight;
        edges[v][u] = newWeight;
    }
}

// Removing vertex removes edges as well
template<typename vertex>
void undirected_graph<vertex>::remove_vertex(const vertex &u)
{
    for (vertex v : vertices) {
        remove_edge(u, v);
    }
    vertices.erase(u);

}

// Removing edge must remove it from both directions
template<typename vertex>
void undirected_graph<vertex>::remove_edge(const vertex &u, const vertex &v)
{
    if (contains(u) && contains(v))
    {
        edges[u].erase(v);
        edges[v].erase(u);
    }
}

template<typename vertex>
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

template<typename vertex>
void undirected_graph<vertex>::prims_mst() {

    std::unordered_map<vertex, bool> visited;
    std::unordered_map<vertex, vertex> minEdge;     // Store the lowest weight "parent" for the given vertex key
    std::unordered_map<vertex, float> weights;      // Store the lowest distance to get to the given vertex key - minEdge and weights updated at the same time

    // Set weights to infinity and all vertices as unvisited
    for (vertex u : vertices) {
        weights[u] = std::numeric_limits<float>::infinity();
        visited[u] = false;
    }

    // Set the starting point for the MST
    vertex startVertex = *vertices.begin();
    weights[startVertex] = 0.0;
    minEdge[startVertex] = startVertex; // As minEdge[startVertex] cannot equal startVertex, we will use this to identify the root during construction later.

    // Loop for each vertex
    for (int i = 0; i < vertices.size() - 1; i++) {

        // Select the next closest vertex (lowest weight as set in the weights[])
        // Initially this will return startVertex
        vertex v = get_min_vertex(weights, visited);
        visited[v] = true;

        for (vertex u : vertices) {
            if (!visited[u] && edges[v][u] < weights[u]) {
                // A cheaper edge has been found
                weights[u] = edges[v][u];
                minEdge[u] = v;
            }
        }
    }

    // Build the MST now that minimum edges are "selected"
    for (vertex u : vertices) {
        vertex v = minEdge[u];
        if (u.location != v.location) {
            mst[u].push_back(v);
            mst[v].push_back(u);
        }
    }

}

// Returns vertices with odd degree in the MST
template<typename vertex>
void undirected_graph<vertex>::find_odd_degrees() {
    for (vertex u : vertices) {
        int degree = get_degree(u);
        if (degree % 2 == 1) {
            odd_vertices.insert(u);
        }
    }
}

// Returns a perfect matching between the odd vertices of the minimum spanning tree
template<typename vertex>
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
    while (!odd_vertices.empty()) {
        // Iterators are handled manually to avoid any null references after deleting elements from odd_vertices
        // Select the first odd vertex
        std::unordered_set<vertex>::iterator it = odd_vertices.begin();
        vertex u = *it;
        std::unordered_set<vertex>::iterator match = it;
        ++match;


        // Loop over all the other odd vertices whilst tracking the minimum weight
        float minWeight = std::numeric_limits<float>::infinity();
        vertex minVertex;
        for (;match != odd_vertices.end(); match++) {
            // Check if the edge between the two vertices is the cheapest
            if (edges[u][*match] < minWeight) {
                minVertex = *match;
                minWeight = edges[u][*match];
            }
        }
        // Add the new edge/matching to the multigraph
        mst[u].push_back(minVertex);
        mst[minVertex].push_back(u);

        // Remove both vertices from the odd_vertices list so they are not considered again
        odd_vertices.erase(u);
        odd_vertices.erase(minVertex);

    }
}

template<typename vertex>
void undirected_graph<vertex>::euler_tour() {

    // Make copy of mst
    std::unordered_map<vertex, std::vector<vertex>> tmpMstCopy = mst;
    std::stack<vertex> stack;

    vertex current = *vertices.begin();
    path.push_back(current);
    vertex neighbour;
    // While the current vertex has neighbours or we have vertices remaining in the stack
    while (!stack.empty() || tmpMstCopy[current].size() > 0) {
        // Pick a neighbour and add it to the stack
        if (tmpMstCopy[current].size() > 0) {
            neighbour = tmpMstCopy[current].back();
            tmpMstCopy[current].pop_back();

            // Also remove the edge from the other vertex as well
            tmpMstCopy[neighbour].erase(std::remove(tmpMstCopy[neighbour].begin(), tmpMstCopy[neighbour].end(), current));
            stack.push(current);
            current = neighbour;
        } else {
        // Otherwise we pull a vertex off the stack, add it to the path, and set to current
            current = stack.top();
            stack.pop();
            path.push_back(current);
        }
    }

}

    // Loop over the euler tour and skip repeated vertices
template<typename vertex>
void undirected_graph<vertex>::make_hamiltonian() {

    // Initialise vertices as unvisited
    std::unordered_map<vertex, bool> visited;
    for (vertex u : vertices) {
        visited[u] = false;
    }
    
    std::vector<vertex> newPath;

    // Visit the first vertex in the path
    vertex current;
    // Walk through the path marking each vertex as visited,
    // If we encounter a visited vertex, look at the next vertex and continue on if not visited
    for (int i = 0; i < path.size() - 1; i++) {
        current = path[i];
        if(!visited[current]) {
            newPath.push_back(current);
            visited[current] = true;
        }
    }

    // Save the new path
    path = newPath;

}

// Runner function for the christofides algorithm
void undirected_graph<vertex>::christofides() {
    prims_mst();
    perfect_matching();
    euler_tour();
    make_hamiltonian();
}

void undirected_graph<vertex>::print_vertices() {
    std::cout << "Vertices:" << std::endl;
    for (vertex u : vertices) {
        std::cout << u.location << ", " << u.xPos << ", " << u.yPos << std::endl;
    }
}

void undirected_graph<vertex>::print_edges() {
    std::cout << "Edges:" << std::endl;
    for (vertex u : vertices) {
        std::cout << "From " << u.location << " to: ";
        for (auto it = edges[u].begin(); it != edges[u].end(); ++it) {
            std::cout << it->first.location << " " << it->second << ", ";
        }
        std::cout << std::endl;
    }
}

void undirected_graph<vertex>::print_path() {
    std::cout << "Optimal Path: " << std::endl;
    float cost = 0.0;
    std::cout << path[0].location << " -> ";
    for (int i = 1; i < path.size()-1; i++) {
        std::cout << path[i].location << " -> ";
        cost += edges[path[i-1]][path[i]];
    }
    std::cout << path[path.size()-1].location << " -> " << path[0].location;
    cost += edges[path[path.size()-1]][path[0]];
    std::cout << std::endl;
    std::cout << "Path Cost: " << std::endl << cost;
}