#include <iostream>
#include <vector>
#include <cmath>
#include <string>
#include <map>
#include <queue>
#include <limits>
#include <algorithm>
#include <tuple>
#include <sstream>

using namespace std;

// Define a shorthand for coordinates for clarity
using Position = tuple<double, double, double>;

// --- 1. Graph and Node Setup (YOUR CONTRIBUTION STARTS HERE) ---

// COMMIT 2: Node Struct Implementation
struct Node {
    string id;
    Position pos; // (X, Y, Z) coordinates for heuristic calculation
    bool is_hazard;
    string hazard_reason;

    // Constructor
    Node(const string& node_id, double x, double y, double z)
        : id(node_id), pos(make_tuple(x, y, z)), is_hazard(false), hazard_reason("") {}
};

// COMMIT 2: Edge Structure
struct Edge {
    string target_id;
    double cost; // Represents fuel consumption
};

// COMMIT 3: OrbitalGraph Class and Construction
class OrbitalGraph {
private:
    // Node Map: {node_id: Node_object}
    map<string, Node> nodes;
    // Adjacency List: {node_id: [Edge1, Edge2, ...]}
    map<string, vector<Edge>> adj;

public:
    void add_node(const Node& node) {
        // Adds a new waypoint to the graph
        nodes.insert({node.id, node});
    }

    bool has_node(const string& id) const {
        return nodes.count(id);
    }

    void add_edge(const string& id1, const string& id2, double cost) {
        // Adds a bidirectional edge (path) between two nodes with fuel cost.
        if (has_node(id1) && has_node(id2)) {
            adj[id1].push_back({id2, cost});
            adj[id2].push_back({id1, cost}); // Symmetrical cost
        } else {
            cerr << "Warning: Could not create edge. One or both nodes do not exist." << endl;
        }
    }

    void mark_hazard(const string& node_id, const string& reason = "Predicted Debris Intersection") {
        // Marks a specific waypoint as a hazard to be avoided by A*
        if (has_node(node_id)) {
            nodes.at(node_id).is_hazard = true;
            nodes.at(node_id).hazard_reason = reason;
            cout << "--- COLLISION PREDICTED: Node " << node_id << " marked as HAZARD. Reason: " << reason << " ---" << endl;
        } else {
            cerr << "Error: Cannot mark hazard. Node " << node_id << " does not exist." << endl;
        }
    }

    const Node& get_node(const string& id) const {
        return nodes.at(id);
    }

    const map<string, Node>& get_all_nodes() const {
        return nodes;
    }

    const vector<Edge>& get_neighbors(const string& id) const {
        // Returns the list of neighboring Edges for a given node ID.
        if (adj.count(id)) {
            return adj.at(id);
        }
        static const vector<Edge> empty_edges;
        return empty_edges;
    }
};

// --- 2. A* Search Algorithm (TO BE ADDED LATER) ---

double heuristic(const Node& node, const Node& goal_node) {
    /**
     * Placeholder: Calculates 3D Euclidean distance.
     * Implementation relies on Commit 5.
     */
    double x1, y1, z1, x2, y2, z2;
    tie(x1, y1, z1) = node.pos;
    tie(x2, y2, z2) = goal_node.pos;
    return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2) + pow(z2 - z1, 2));
}

// A* function body is left as a placeholder for later commits
using AStarEntry = pair<double, string>;

pair<vector<string>, double> find_safe_path_astar(
        const OrbitalGraph& graph,
        const string& start,
        const string& goal)
{
    return {{"A* not implemented yet (Commit 9)."}, 0.0};
}

// --- 3. User Input Functions (TO BE ADDED LATER) ---




void setup_and_run_demo() {
    cout << "\n===== SPACE DEBRIS COLLISION AVOIDANCE — DEMO (Commit 8) =====\n";

    OrbitalGraph graph;

    graph.add_node(Node("A", 0, 0, 0));
    graph.add_node(Node("B", 1, 0, 0));
    graph.add_node(Node("C", 2, 0, 0));
    graph.add_node(Node("D", 3, 0, 0));
    graph.add_node(Node("E", 0, 1, 0));
    graph.add_node(Node("F", 1, 1, 0));
    graph.add_node(Node("G", 2, 1, 0));
    graph.add_node(Node("H", 3, 1, 0));

    cout << "✔️  Added nodes A–H\n";

    graph.add_edge("A", "B", 1.0);
    graph.add_edge("B", "C", 1.0);
    graph.add_edge("C", "D", 1.0);
    graph.add_edge("E", "F", 1.0);
    graph.add_edge("F", "G", 1.0);
    graph.add_edge("G", "H", 1.0);
    graph.add_edge("A", "E", 1.4);
    graph.add_edge("B", "F", 1.4);
    graph.add_edge("C", "G", 1.4);
    graph.add_edge("D", "H", 1.4);

    cout << "✔️  Added edges\n";

    graph.mark_hazard("C", "High-Density Debris Region");
    graph.mark_hazard("G", "Severe Orbital Fragment Field");

    string start = "A";
    string goal = "H";

    cout << "\nStart Node : " << start;
    cout << "\nGoal Node  : " << goal << endl;

    auto result = find_safe_path_astar(graph, start, goal);

    cout << "\nA* Output: " << result.first[0] << endl;
}


int main() {
    // run_interactive_demo();
    setup_and_run_demo();
    return 0;
}


