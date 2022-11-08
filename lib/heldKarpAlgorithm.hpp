
// Dynamic programming approach to TSP

// Takes input of an n x n weighted adjacency matrix
class held_karp_tsp
{

private:
    int N;
    int start_node;
    int finish_state;

    double[][] distance;
    double

<<<<<<< Updated upstream
        public:
}
=======
    // Recursive function
    double tsp_recursion(int i, int state, std::vector<std::vector<double>>& memo, std::vector<std::vector<int>>& previous)
    {
        // std::cout << "Recursion started" << std::endl;

        // Tour finished
        if (state == finish_state) {
            std::cout << "Tour done" << std::endl;
            return distance[i][start_node];
        }
        // Return cached if already computed
        if (memo[i][state] != NULL) {
            // std::cout << "Already computed" << std::endl;
            return memo[i][state];
        }
        double minimumCost = std::numeric_limits<double>::infinity();
        int index = -1;
        for (int next = 0; next < N; next++)
        {
            if (i != 4) {
                std::cout << i << ", " << next << std::endl;
            }
            // If next node has already been visited
            if ((state & (1 << next)) != 0) {
                // std::cout << "Already visited" << std::endl;
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

        previous[i][state] = index;
        std::cout << "Previous set" << std::endl;
        memo[i][state] = minimumCost;
        std::cout << minimumCost << std::endl;
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
        int state = 1 << start_node;
        std::vector<std::vector<double>> memo(N, std::vector<double>(1 << N, NULL));
        std::vector<std::vector<int>> previous(N, std::vector<int>(1 << N, NULL));
        minimumTourCost = tsp_recursion(start_node, state, memo, previous);
        std::cout << "Recursion done" << std::endl;

        // After the minimum cost path is found, backtrack and rebuild the optimal tour
        int index = start_node;
        while (true)
        {
            std::cout << "Running loop" << std::endl;
            tour.push_back(index);
            int nextIndex = previous[index][state];
            if (nextIndex == NULL)
                std::cout << "Breaking" << std::endl;
                break;
            int nextState = state | (1 << nextIndex);
            state = nextState;
            index = nextIndex;
        }

        tour.push_back(start_node);
        solver = true;
    }

    std::vector<int> get_optimal_tour() {
        return tour;
    }
};
>>>>>>> Stashed changes
