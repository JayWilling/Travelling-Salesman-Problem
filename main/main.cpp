#include <iostream>
#include <vector>
#include "../lib/heldKarpAlgorithm.hpp"

int main()
{

    // Build adjacency matrix
    int n = 6;
    std::vector<std::vector<double>> distMatrix(n, std::vector<double>(n));
    distMatrix[1][4] = distMatrix[4][1] = 2;
    distMatrix[4][2] = distMatrix[2][4] = 4;
    distMatrix[2][3] = distMatrix[3][2] = 6;
    distMatrix[3][0] = distMatrix[0][3] = 8;
    distMatrix[0][5] = distMatrix[5][0] = 10;
    distMatrix[5][1] = distMatrix[1][5] = 12;

    /* Potential issues may stem from:
            - Incorrect integer type used (needs to be 32 bit)
            - I have not used pointers, focused on replicating the example we have
            - Assumed I have used vectors correctly but haven't seen them in a while..
    */

    return EXIT_SUCCESS;
}