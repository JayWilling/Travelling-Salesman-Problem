#include <vector>
#include <limits>
#include <iostream>

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
    double minimumTourCost;

    // Recursive function
    double tsp_recursion(int i, int state, std::vector<std::vector<double>>& memo, std::vector<std::vector<int>>& previous)
    {
        // The current state indicates all vertices have been visited
        // Recursion ends and the current cost is returned
        if (state == finish_state) {
            return distance[i][start_node];
        }
        // Return cached value if already computed
        if (memo[i][state] != NULL) {
            return memo[i][state];
        }

        // 
        double minimumCost = std::numeric_limits<double>::infinity();
        int index = -1;
        for (int next = 0; next < N; next++)
        {
            // If the next node has already been visited
            if ((state & (1 << next)) != 0) {
                continue;
            }
            // Calculate cost for the next state
            int nextState = state | (1 << next);
            double newCost = distance[i][next] + tsp_recursion(next, nextState, memo, previous);
            if (newCost < minimumCost)
            {
                std::cout << "New min cost" << std::endl;
                minimumCost = newCost;
                index = next;
            }
        }

        // Set the value of the previous state to match the minimum cost
        previous[i][state] = index;
        memo[i][state] = minimumCost;

        // Return the minimum for the current state
        return memo[i][state];
    }

public:

    // Constructor
    held_karp_tsp(int startNode, std::vector<std::vector<double>> distMatrix)
    {
        this->distance = distMatrix;
        this->N = distance.size();
        this->start_node = startNode;

        this->finish_state = (1 << N) - 1;
    }

    // Entry point to the Held-Karp algorithm
    /*
    We need to track the:
        1. Current state (Where we are and what has been visited)
        2. Calculated optimal sub tours (Stored in memo)
        3. Optimal tour cost
        4. Previous used to track the states or locations when backtracking to obtain the optimal tour
    */
    void calculate_optimal_tour()
    {   
        // Implementation relies on bitwise operation to efficiently maintain states
        int state = 1 << start_node;
        std::vector<std::vector<double>> memo(N, std::vector<double>(1 << N, NULL));
        std::vector<std::vector<int>> previous(N, std::vector<int>(1 << N, NULL));
        minimumTourCost = tsp_recursion(start_node, state, memo, previous);

        // After the minimum cost path is found, backtrack and rebuild the optimal tour
        int index = start_node;
        while (true)
        {
            tour.push_back(index);
            int nextIndex = previous[index][state];
            if (nextIndex == NULL)
                break;
            // Increment to the next state and continue
            int nextState = state | (1 << nextIndex);
            state = nextState;
            index = nextIndex;
        }

        tour.push_back(start_node);
    }

    std::vector<int> get_optimal_tour() {
        return tour;
    }
};