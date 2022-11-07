#include <vector>

// Dynamic programming approach to TSP

// Takes input of an n x n weighted adjacency matrix
class held_karp_tsp
{

private:
    int N;
    int start_node;
    int finish_state;

    std::vector<std::vector<double>> distance;
    std::vector<int> tour;
    bool solver = false;

public:
};