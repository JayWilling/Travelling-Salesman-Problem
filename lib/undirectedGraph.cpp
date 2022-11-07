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
}