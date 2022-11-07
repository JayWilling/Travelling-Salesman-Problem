#include <vector>
#include <limits>

// Dynamic programming approach to TSP

// Takes input of an n x n weighted adjacency matrix
/*
    N - Size of the input 2D matrix
    start_node - Randomly selected starting node. Does not matter which one
    finish_state - Used to check if all nodes have been visited
*/
class held_karp_tsp
{

private:
    int N;
    int start_node;
    int finish_state;

    std::vector<std::vector<double>> distance;
    std::vector<int> tour;
    bool solver = false;
    double minimumTourCost;

    // Recursive function
    double tsp_recursion(int i, int state, std::vector<std::vector<double>> memo, std::vector<std::vector<int>> previous)
    {

        // Tour finished
        if (state == finish_state)
            return distance[i][start_node];

        // Return cached if already computed
        if (memo[i][state] != NULL)
            return memo[i][state];

        double minimumCost = std::numeric_limits<double>::infinity();
        int index = -1;
        for (int next = 0; next < N; next++)
        {

            // If next node has already been visited
            if ((state && (1 << next)) != 0)
                continue;

            // Calculate cost for the next state
            int nextState = state | (i << next);
            double newCost = distance[i][next] + tsp_recursion(next, nextState, memo, previous);
            if (newCost < minimumCost)
            {
                minimumCost = newCost;
                index = next;
            }
        }

        previous[i][state] = index;
        return memo[i][state] = minimumCost;
    }

public:
    // Constructor
    held_karp_tsp(int startNode, std::vector<std::vector<double>> distMatrix)
    {
        this->distance = distMatrix;
        N = distance.size();
        start_node = startNode;

        finish_state = (1 << N) - 1;
    }

    // Entry point to the Held-Karp algorithm
    /*
    We need to track the:
        1. Current state (Where we are and what has been visited)
        2.
    */
    void get_optimal_tour()
    {
        int state = 1 << start_node;
        std::vector<std::vector<double>> memo(N, std::vector<double>(1 << N));
        std::vector<std::vector<int>> previous(N, std::vector<int>(1 << N));
        minimumTourCost = tsp_recursion(start_node, state, memo, previous);

        // After the minimum cost path is found, backtrack and rebuild the optimal tour
        int index = start_node;
        while (true)
        {
            tour.push_back(index);
            int nextIndex = previous[index][state];
            if (nextIndex == NULL)
                break;
            int nextState = state | (1 << nextIndex);
            state = nextState;
            index = nextIndex;
        }

        tour.push_back(start_node);
        solver = true;
    }
};