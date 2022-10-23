#ifndef UNDIRECTED_GRAPH
#define UNDIRECTED_GRAPH

#include <vector>
#include <string>
#include <unordered_set>
#include <unordered_map>

// Define the node and constructor

struct Node {
    std::string location;
    int x;
    int y;
};

class Graph {

    private:
        std::unordered_set<Node> nodes;
        std::unordered_map<Node, std::unordered_set<Node>> edges;

    protected:


};

#endif