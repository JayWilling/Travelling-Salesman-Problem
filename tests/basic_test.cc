#include "gtest/gtest.h"
#include "../lib/undirectedGraph.h"
#include <vector>
#include <algorithm>
#include <iterator>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <string>

class TSPTest : public testing::Test {
  protected:
    void SetUp() override {
      std::srand(std::time(0));
    }

    // void TearDown() override {

    // }

    // Setup some RB trees for use across all tests
    

};

bool cycleRecursion(std::unordered_map<vertex, std::vector<vertex>> mst, std::unordered_map<vertex, bool> visited, vertex u, vertex parent) {
    visited[u] = true;

    for (std::vector<vertex>::iterator it = mst[u].begin(); it != mst[u].end(); ++it) {
        if (!visited[*it]) {
            return cycleRecursion(mst, visited, *it, u);
        } else if (*it != parent) {return true;}
    }
    return false;
}

TEST_F(TSPTest, TestPrims) {
    undirected_graph<vertex> graph = undirected_graph<vertex>();

    std::vector<vertex> verts;
    std::vector<std::string> place_names = {"Sydney",
     "Melbourne", "Perth", "Darwin", "Adelaide", "Tasmania", "Canberra", "Cairns", "Alice Springs"};

    for (std::string name : place_names) {
        graph.add_vertex(*(new vertex(name, std::rand()%20+1, std::rand()%20+1)));
    }

    // for (vertex u : verts) {
    //     graph.add_vertex(u);
    // }
    
    // Test prims algorithm
    graph.prims_mst();
    std::unordered_map<vertex, bool> visited;
    std::unordered_set<vertex> vertices = graph.get_vertices();
    std::unordered_map<vertex, std::vector<vertex>> mst = graph.get_mst();
    for (vertex u : vertices) {
        visited[u] = false;
    }

    // // Recurse over the MST via DFS
    bool foundCycle = false;
    for (vertex u : vertices) {
        if (!visited[u]) {
            foundCycle = cycleRecursion(mst, visited, u, u);
        }
    }

    ASSERT_EQ(foundCycle, false);
}

TEST_F(TSPTest, TestPerfectMatching) {

    // Construct graph
    undirected_graph<vertex> graph = undirected_graph<vertex>();

    std::vector<vertex> verts;
    std::vector<std::string> place_names = {"Sydney",
     "Melbourne", "Perth", "Darwin", "Adelaide", "Tasmania", "Canberra", "Cairns", "Alice Springs"};

    for (std::string name : place_names) {
        graph.add_vertex(*(new vertex(name, std::rand()%20+1, std::rand()%20+1)));
    }

    // Run prims to obtain MST
    graph.prims_mst();
    size_t edgeCountBefore = graph.get_edge_count();
    graph.find_odd_degrees();
    size_t num_odd_vertices = graph.get_odd_vertices().size();
    graph.perfect_matching();

    size_t edgeCountAfter = graph.get_edge_count();

    // As the perfect matching is added directly to the graph
    // The edgecount can be subtracted from before the perfect matching is made

    ASSERT_EQ(edgeCountAfter - edgeCountBefore, num_odd_vertices);

}

TEST_F(TSPTest, TestEulerian) {

    // Construct graph
    undirected_graph<vertex> graph = undirected_graph<vertex>();

    std::vector<vertex> verts;
    std::vector<std::string> place_names = {"Sydney",
     "Melbourne", "Perth", "Darwin", "Adelaide", "Tasmania", "Canberra", "Cairns", "Alice Springs"};

    for (std::string name : place_names) {
        graph.add_vertex(*(new vertex(name, std::rand()%20+1, std::rand()%20+1)));
    }

    // Run prims to obtain MST
    graph.prims_mst();
    graph.perfect_matching();
    size_t multigraphEdgeCount = graph.get_edge_count();
    
    // Get path after Euler tour obtained
    graph.euler_tour();
    std::vector<vertex> path = graph.get_path();

    // Edgecount divided by 2 to account for storage in both directions
    ASSERT_EQ(multigraphEdgeCount/2, path.size() - 1);

}

TEST_F(TSPTest, TestHamiltonian) {
        // Construct graph
    undirected_graph<vertex> graph = undirected_graph<vertex>();

    std::vector<vertex> verts;
    std::vector<std::string> place_names = {"Sydney",
     "Melbourne", "Perth", "Darwin", "Adelaide", "Tasmania", "Canberra", "Cairns", "Alice Springs"};

    for (std::string name : place_names) {
        graph.add_vertex(*(new vertex(name, std::rand()%20+1, std::rand()%20+1)));
    }

    graph.christofides();

    std::vector<vertex> path = graph.get_path();
    std::unordered_set<vertex> vertices = graph.get_vertices();
    std::unordered_map<vertex, bool> visited;
    for (vertex u : vertices) {
        visited[u] = false;
    }
    bool notHamiltonian = false;
    for (int i = 0; i < path.size() - 1; i++) {
        if (visited[path[i]]) {
            notHamiltonian = true;
        } else {
            visited[path[i]] = true;
        }
    }

    // No repeated vertices present
    ASSERT_EQ(notHamiltonian, false);
    // All vertices included
    ASSERT_EQ(path.size(), vertices.size());

}