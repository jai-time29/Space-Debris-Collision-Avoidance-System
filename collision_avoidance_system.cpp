#include <iostream>
#include <string>
#include <map>
#include <vector>
#include <array>
using namespace std;

struct Node {
    string id;
    array<double, 3> pos;
    bool is_hazard = false;
    string hazard_reason;

    Node(const string& id1, double x, double y, double z)
        : id(id1), pos{x, y, z} {}
};

struct Edge {
    string to;
    double weight;
    Edge(const string& t, double w) : to(t), weight(w) {}
};

class OrbitalGraph {
public:
    void add_node(const Node& node) {
        nodes[node.id] = node;
        adjacency[node.id] = vector<Edge>{};
    }

    void add_edge(const string& a, const string& b, double w) {
        adjacency[a].emplace_back(b, w);
        adjacency[b].emplace_back(a, w);
    }

    void mark_hazard(const string& id, const string& reason) {
        if (nodes.count(id) == 0) return;
        nodes[id].is_hazard = true;
        nodes[id].hazard_reason = reason;
    }

    void print_nodes() const {
        for (const auto& [id, n] : nodes) {
            cout << "Node " << id << " pos=("
                      << n.pos[0] << ", " << n.pos[1] << ", " << n.pos[2] << ") "
                      << "hazard=" << (n.is_hazard ? "YES" : "NO");
            if (n.is_hazard) cout << " reason=" << n.hazard_reason;
            cout << "\n";
        }
    }

private:
    map<string, Node> nodes;
    map<string, vector<Edge>> adjacency;
};

int main() {

    return 0;
}
