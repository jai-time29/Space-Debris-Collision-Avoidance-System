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

// --- 1. Graph and Node Setup ---

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

// Replace the previous placeholder with this implementation
pair<vector<string>, double> find_safe_path_astar(const OrbitalGraph& graph, const string& start_id, const string& goal_id) {
    // Sanity checks
    if (!graph.has_node(start_id) || !graph.has_node(goal_id)) {
        cerr << "Error: start or goal node does not exist in the graph.\n";
        return {{}, 0.0};
    }

    const Node start_node = graph.get_node(start_id);
    const Node goal_node  = graph.get_node(goal_id);

    // --- Core A* data structures (Commit 6) ---
    // g_score: cost from start to node (default = +inf)
    unordered_map<string, double> g_score;
    // f_score: g_score + heuristic estimate (default = +inf)
    unordered_map<string, double> f_score;
    // came_from: for path reconstruction (parent pointer)
    unordered_map<string, string> came_from;

    // Initialize scores to +infinity for all nodes in graph
    for (const auto& p : graph.get_all_nodes()) {
        const string& nid = p.first;
        g_score[nid] = numeric_limits<double>::infinity();
        f_score[nid] = numeric_limits<double>::infinity();
        // came_from left empty / absent until we set a parent
    }

    // Min-heap (priority_queue) for the open set
    // AStarEntry = pair<f_score, node_id>
    using AStarEntry = pair<double, string>;
    priority_queue<AStarEntry, vector<AStarEntry>, greater<AStarEntry>> open_set;

    // Initialize start node
    g_score[start_id] = 0.0;
    f_score[start_id] = heuristic(start_node, goal_node);
    open_set.push({f_score[start_id], start_id});

    // Debug / confirmation output (safe, non-invasive)
    cout << "[Commit 6] A* core setup initialized.\n";
    cout << "  Start: " << start_id << " | g=" << g_score[start_id] << " | f=" << f_score[start_id] << "\n";
    cout << "  Goal : " << goal_id << "\n";
    cout << "  Open set size: " << open_set.size() << " (start pushed)\n";

    return {{"A* core setup initialized (waiting for full traversal loop in Commit 7)."}, 0.0};
}

// --- 3. User Input Functions (TO BE ADDED LATER) ---

void display_current_nodes(const OrbitalGraph& graph) {
    cout << "\nExisting Nodes (Waypoints): ";
    bool first = true;
    for (const auto& pair : graph.get_all_nodes()) {
        if (!first) cout << ", ";
        cout << pair.first;
        first = false;
    }
    cout << endl;
}

void read_nodes(OrbitalGraph& graph) {
    // Logic for reading nodes...
    cout << "\n[Placeholder for Node reading logic]" << endl;
}

void read_edges(OrbitalGraph& graph) {
    // Logic for reading edges...
    cout << "\n[Placeholder for Edge reading logic]" << endl;
}

void read_scenario_parameters(OrbitalGraph& graph, string& start_id, string& goal_id) {
    // Logic for reading scenario...
    cout << "\n[Placeholder for Scenario reading logic]" << endl;
    // Mock data for compilation test:
    start_id = "A"; goal_id = "B";
    graph.mark_hazard("C");
}


void run_interactive_demo() {
    cout << "--- Graph-Based Space Debris Collision Avoidance System ---" << endl;
    OrbitalGraph orbit_graph;
    string start_id, goal_id;

    // To prevent immediate crash, we add mock nodes for compilation:
    orbit_graph.add_node(Node("A", 0, 0, 0));
    orbit_graph.add_node(Node("B", 1, 1, 1));
    orbit_graph.add_node(Node("C", 2, 2, 2));
    orbit_graph.add_edge("A", "B", 1.0);


    read_nodes(orbit_graph);
    read_edges(orbit_graph);
    read_scenario_parameters(orbit_graph, start_id, goal_id);

    pair<vector<string>, double> result = find_safe_path_astar(orbit_graph, start_id, goal_id);
    cout << "\nSimulation initialized. A* result: " << result.first[0] << endl;
}

int main() {
    run_interactive_demo();
    return 0;
}
