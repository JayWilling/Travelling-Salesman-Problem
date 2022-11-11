#ifndef UNDIRECTED_GRAPH
#define UNDIRECTED_GRAPH

#include <vector>
#include <string>
#include <unordered_set>
#include <unordered_map>
#include <utility>          // std::pair
#include <cmath>
#include <limits>
#include <queue>
#include <stack>
#include <iostream>


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
    vertex();

    // Structs do not have predefined comparison operators
    bool operator!=(const vertex &other) const
    {
        return (location != other.location &&
               xPos != other.xPos &&
               yPos != other.yPos);
    }

    bool operator==(const vertex &other) const
    {
        return (location == other.location &&
               xPos == other.xPos &&
               yPos == other.yPos);
    }

    // explicit vertex();
};

// Hash function for unordered map
namespace std {
    template <>
    struct hash<vertex>
    {
        std::size_t operator()(const vertex& k) const
        {
        using std::size_t;
        using std::hash;
        using std::string;

        // Compute individual hash values for first,
        // second and third and combine them using XOR
        // and bit shifting:

        return ((hash<string>()(k.location)
                ^ (hash<int>()(k.xPos) << 1)) >> 1)
                ^ (hash<int>()(k.yPos) << 1);
        }
    };
}

/*
Key requirements for TSP:
    - Fast search and retrieval of edges
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
    std::unordered_map<vertex, std::unordered_map<vertex, float>> edges;    // Stores the weight tied to each edge
    std::unordered_map<vertex, std::vector<vertex>> mst;                    // Adjacency list for multigraph construction
    std::unordered_set<vertex> odd_vertices;                                // List of vertices with odd degree in minimum spanning tree
    std::vector<vertex> path;                                               // Path does not need to store weights
    
    vertex get_min_vertex(std::unordered_map<vertex, float> weights, std::unordered_map<vertex, bool> visited);

public:
    // Constructor(s) & destructors
    undirected_graph();

    // Iterators for the vertex and edge lists
    typedef typename std::unordered_set<vertex>::iterator vertex_iterator;
    typedef typename std::unordered_set<vertex>::iterator neighbour_iterator;
    typedef typename std::unordered_set<vertex>::const_iterator const_vertex_iterator;
    typedef typename std::unordered_set<vertex>::const_iterator const_neighbour_iterator;

    // Core graph functions - Define more as required
    void add_vertex(vertex &u);
    void add_edge(const vertex &u, const vertex &v);

    void remove_vertex(const vertex &u);
    void remove_edge(const vertex &u, const vertex &v);

    float calculate_weight(const vertex &u, const vertex &v);
    bool contains(const vertex &u) const;
    int get_degree(const vertex &u);

    // For Christofides
    //      -- List for odd vertices in private ^^ (Currently a vector, might change to unordered_set)
    //      -- Not sure if we'll need these functions to return anything yet but we'll start here
    void prims_mst();
    void find_odd_degrees();
    void perfect_matching();
    void euler_tour();
    void make_hamiltonian();
    void christofides();

    // Iterator function definitions
    vertex_iterator vertexBegin();
    vertex_iterator vertexEnd();
    const_vertex_iterator cVertexBegin();
    const_vertex_iterator cVertexEnd();

    neighbour_iterator neighbourBegin();
    neighbour_iterator neighbourEnd();
    const_neighbour_iterator cNeighbourBegin();
    const_neighbour_iterator cNeighbourEnd();

    // Print and return functions
    void print_vertices();
    void print_edges();
    void print_path();
    void get_path_cost();
    size_t get_edge_count();
    std::unordered_set<vertex> get_vertices();
    std::unordered_set<vertex> get_odd_vertices();
    std::unordered_map<vertex, std::vector<vertex>> get_mst();
    std::vector<vertex> get_path();

};

#endif