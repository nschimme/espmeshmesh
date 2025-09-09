#include "graph.h"
#include <algorithm>

namespace espmeshmesh {

// Helper function to find a node by its ID.
// The vector of nodes must be sorted by ID.
static std::vector<Node>::iterator find_node_by_id(std::vector<Node>& nodes, uint32_t id) {
    auto it = std::lower_bound(nodes.begin(), nodes.end(), Node{id, {}});
    if (it != nodes.end() && it->id == id) {
        return it;
    }
    return nodes.end();
}

void Graph::add_node(uint32_t id) {
    auto it = std::lower_bound(nodes.begin(), nodes.end(), Node{id, {}});
    if (it == nodes.end() || it->id != id) {
        nodes.insert(it, {id, {}});
    }
}

void Graph::add_edge(uint32_t from, uint32_t to, int16_t rssi) {
    add_node(from);
    add_node(to);

    auto it = find_node_by_id(nodes, from);
    if (it != nodes.end()) {
        it->edges.push_back({to, rssi});
    }
}

const std::vector<Node>& Graph::get_nodes() const {
    return nodes;
}

void Graph::clear() {
    nodes.clear();
}

}
