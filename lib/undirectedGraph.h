#ifndef UNDIRECTED_GRAPH
#define UNDIRECTED_GRAPH

#include <vector>
#include <string>
#include <unordered_set>
#include <unordered_map>
#include <utility> // std::pair

// Define the node and constructor

/*
Vertices only contains internal information about itself
No information about the rest of the graph is stored here
*/
struct vertex
{
    std::string location;
    int xPos;
    int yPos;
    vertex(std::string, int, int);

    // Structs do not have predefined comparison operators
    bool operator!=(const vertex &other)
    {
        return location == other.location &&
               xPos == other.xPos &&
               yPos == other.yPos;
    }

    bool operator==(const vertex &other)
    {
        return location == other.location &&
               xPos == other.xPos &&
               yPos == other.yPos;
    }
};

/*
Key requirements for TSP:
    - Fast search and retrieval of edges
    - Insertion/deletion can be slow, will not take place often if at all beyond the initial graph setup
    - Graph is weighted. But weights are determined from positions of the vertices themselves.
        - Weights are paired with the relevant vertex in the adjacency list for faster access (Does not need to be recalculated each time)

This graph will look like a bi-directional directed graph
    (Should be able to update it after we have the core defined)
Space complexity will be as follows:
    Vertices = V
    Edges = E
    O(V + 2*E)
*/
template <typename vertex>
class undirected_graph
{

private:
    std::unordered_set<vertex> vertices;
    std::unordered_map<vertex, std::unordered_set<vertex>> edges;
    std::vector<vertex> odd_vertices;

public:
    // Constructor(s) & destructors
    undirected_graph();
    undirected_graph(const undirected_graph<vertex> otherGraph);
    ~undirected_graph();

    // Iterators for the vertex and edge lists
    typedef typename std::unordered_set<vertex>::iterator vertex_iterator;
    typedef typename std::unordered_set<vertex>::iterator neighbour_iterator;
    typedef typename std::unordered_set<vertex>::const_iterator const_vertex_iterator;
    typedef typename std::unordered_set<vertex>::const_iterator const_neighbour_iterator;

    // Core graph functions - Define more as required
    void add_vertex(const vertex &u);
    void add_edge(const vertex &u, const vertex &v);

    void remove_vertex(const vertex &u);
    void remove_edge(const vertex &u, const vertex &v);

    float calculate_weight(const vertex &u, const vertex &v);
    bool contains(const vertex &u) const;

    // For Christofides
    //      -- List for odd vertices in private ^^ (Currently a vector, might change to unordered_set)
    //      -- Not sure if we'll need these functions to return anything yet but we'll start here
    void christofides();            // Presumably the parent function
    void prims_mst();               // Pick either, both have benefits
    void kruskals_mst();
    void perfect_matching();
    void euler_tour();
    void make_hamiltonian();

    // Iterator function definitions
    vertex_iterator vertexBegin();
    vertex_iterator vertexEnd();
    const_vertex_iterator cVertexBegin();
    const_vertex_iterator cVertexEnd();

    neighbour_iterator neighbourBegin();
    neighbour_iterator neighbourEnd();
    const_neighbour_iterator cNeighbourBegin();
    const_neighbour_iterator cNeighbourEnd();
};

#endif