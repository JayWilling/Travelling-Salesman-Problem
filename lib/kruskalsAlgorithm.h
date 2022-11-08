// Kruskal's Algorithm
#include <bits/stdc++.h>
using namespace std;
 
class kruskals_algorithm {
    int* parent;
    int* rank;
 
private:
    kruskals_algorithm(int n) {
        parent = new int[n];
        rank = new int[n];
 
        for (int i = 0; i < n; i++) {
            parent[i] = -1;
            rank[i] = 1;
        }
    }

    int find_set(int i) {
        return 0;
    }

    void union_set(int u, int v) {

    }
};

class Graph {
    vector<vector<int>> edgelist;
    int V;

    public:
    Graph(int V) { this->V = V; }
};