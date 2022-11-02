#ifndef DIRECTED_GRAPH_H
#define DIRECTED_GRAPH_H

#include <queue>
#include <stack>
#include <unordered_set>
#include <unordered_map>

template <typename vertex> class directed_graph;

template <typename vertex>
directed_graph<vertex> reverse(directed_graph<vertex> g) {

  directed_graph<vertex> rg;
  
  for (auto v : g) {
    rg.add_vertex(v);
  }
  
  for (auto v : g) {
    for (auto u : g.get_neighbours(v)) {
      rg.add_edge(u, v);
    }
  }
  
  return rg;
  
}

template <typename vertex, class Compare = std::less<vertex>>
std::vector<vertex> sort(const std::vector<vertex>& in, Compare cmp = Compare()) {
  std::vector<vertex> out(in);

	for (auto i = 1; i < out.size(); ++i) {
		auto x = out[i];
		auto j = i -1;
		while (j >= 0 && cmp(x, out[j])) {
			out[j+1] = out[j];
			--j;
		}
		out[j+1] = x;
	}

  return out;
}

template <typename vertex>
class directed_graph {

private:

  std::unordered_set<vertex> vertices;
  std::unordered_map<vertex, std::unordered_set<vertex>> edges;

public:
	
  typedef typename std::unordered_set<vertex>::iterator vertex_iterator;
  typedef typename std::unordered_set<vertex>::iterator neighbour_iterator;
  typedef typename std::unordered_set<vertex>::const_iterator const_vertex_iterator;
  typedef typename std::unordered_set<vertex>::const_iterator const_neighbour_iterator;
  
  directed_graph() {}
  directed_graph(const directed_graph<vertex>& other) {
    vertices = other.vertices;
    edges = other.edges;
  }
  ~directed_graph() {}

  bool contains(const vertex& u) const { return vertices.count(u) > 0; }
  
  bool adjacent(const vertex& u, const vertex& v) const {
    if (contains(u) && contains(v)) return edges.at(u).count(v) > 0;
    return false;
  }

  void add_vertex(const vertex& u) {
  	if (!contains(u)) {
    	vertices.insert(u);
    	edges[u] = std::unordered_set<vertex>();
	}
  }
  
  void add_edge(const vertex& u, const vertex& v) {
    if (contains(u) && contains(v) && u != v) edges[u].insert(v);
  }

  void remove_vertex(const vertex& u) {
    vertices.erase(u);
    edges.erase(u);
  }
  
  void remove_edge(const vertex& u, const vertex& v) {
    if (contains(u)) edges[u].erase(v);
  }

  std::size_t in_degree(const vertex& u) const {
    auto count = 0;
    for (auto v : vertices) {
      if (u != v) count += edges.at(v).count(u);
    }
    return count;
  }
  
  std::size_t out_degree(const vertex& u) const { return contains(u)? edges.at(u).size() : 0; }
  
  std::size_t degree(const vertex& u) const { return in_degree(u) + out_degree(u); }
  
  std::size_t num_vertices() const { return vertices.size(); }
  
  std::size_t num_edges() const {
    auto count = 0;
    for (auto entry : edges) count += entry.second.size();
    return count;
  }

  std::vector<vertex> get_vertices() const {
	return std::vector<vertex>(cbegin(), cend());
  }
  
  std::vector<vertex> get_neighbours(const vertex& u) const {
    return std::vector<vertex>(cnbegin(u), cnend(u));
  }

  vertex_iterator begin() {
    return vertices.begin();
  }

  vertex_iterator end() {
    return vertices.end();
  }
    
  const_vertex_iterator cbegin() const {
  	return vertices.cbegin();
  }
  
  const_vertex_iterator cend() const {
  	return vertices.cend();
  }

  neighbour_iterator nbegin(const vertex& u) {
    return edges.at(u).begin();
  }
  
  neighbour_iterator nend(const vertex& u) {
    return edges.at(u).end();
  }
  
  const_neighbour_iterator cnbegin(const vertex& u) const {
    return edges.at(u).cbegin();
  }
  
  const_neighbour_iterator cnend(const vertex& u) const {
    return edges.at(u).cend();
  }

  std::vector<vertex> depth_first(const vertex& u) const {

    std::stack<vertex> to_process;
    std::unordered_set<vertex> visited;
    std::vector<vertex> output;

    to_process.push(u);

    while (!to_process.empty()) {
      vertex current = to_process.top();
      to_process.pop();

      if (visited.count(current) == 0) {
		visited.insert(current);
		output.push_back(current);

		auto n = sort(get_neighbours(current), std::greater<vertex>());

		for (auto v : n) {
	  		if (visited.count(v) == 0) to_process.push(v);
		}

      }
    }

    return output;
    
  }

  std::vector<vertex> breadth_first(const vertex& u) const {
    std::queue<vertex> to_process;
    std::unordered_set<vertex> visited;
    std::vector<vertex> output;

    to_process.push(u);

    while (!to_process.empty()) {
      vertex current = to_process.front();
      to_process.pop();

      if (visited.count(current) == 0) {
		visited.insert(current);
		output.push_back(current);

		auto n = sort(get_neighbours(current));

		for (auto v : n) {
	  		if (visited.count(v) == 0) to_process.push(v);
		}
      }
    }

    return output;
  }

  directed_graph<vertex> out_tree(const vertex& u) const {
    directed_graph<vertex> tree;

    std::queue<vertex> to_process;
    tree.add_vertex(u);
	to_process.push(u);

    while (!to_process.empty()) {
      auto current = to_process.front();
      to_process.pop();
      for (auto w : get_neighbours(current)){
	  	if (!tree.contains(w)) {
			tree.add_vertex(w);
			tree.add_edge(current, w);
			to_process.push(w);
		}
	  }
    }

    return tree;    
  }

  directed_graph<vertex> in_tree(const vertex& u) const {
    return reverse(reverse(*this).out_tree(u));
  }

  bool reachable(const vertex& u, const vertex& v) const {
	return out_tree(u).contains(v);
  }

};

#endif
