#pragma once

#include <cstdint>
#include <vector>
#include <algorithm>

namespace espmeshmesh {

struct Edge {
    uint32_t to;
    int16_t rssi;
};

struct Node {
    uint32_t id;
    std::vector<Edge> edges;

    bool operator<(const Node& other) const {
        return id < other.id;
    }
};

class Graph {
public:
    void add_node(uint32_t id);
    void add_edge(uint32_t from, uint32_t to, int16_t rssi);
    const std::vector<Node>& get_nodes() const;
    void clear();

private:
    std::vector<Node> nodes;
};

}
