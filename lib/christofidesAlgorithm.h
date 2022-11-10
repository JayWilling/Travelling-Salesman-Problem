// Christofides Algorithm (Approximation) to TSP

// Pseudocode/Steps from Presentation: 
// Create a MST of the graph (Krusal's or Prim's)
// Select all vertices with edges of odd degree to make a set
// Form a new subgraph using the selected vertices for a new MST
// Form a min weight perfect matching
// Create Eulerian Multigraph
// Calculate Euler Tour
// Create Hamiltonian Cycle by removing repeated vertices within the tour and reroute to next vertex and recaluate the edge

#include <string>
#include <vector>

class christofides_algorithm {

    private:
        struct location {
            std::string name;
            int xPos;
            int yPos;
        };

        std::vector<location> locations;
        std::vector<double> 

    public:

};