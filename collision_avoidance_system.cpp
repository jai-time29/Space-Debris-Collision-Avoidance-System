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

struct Node {
    string id;
    Position pos; // (X, Y, Z) coordinates for heuristic calculation
    bool is_hazard;
    string hazard_reason;

    Node(const string& node_id, double x, double y, double z)
        : id(node_id), pos(make_tuple(x, y, z)), is_hazard(false), hazard_reason("") {}
};

struct Edge {
    string target_id;
    double cost; // Represents fuel consumption
};

class OrbitalGraph {
private:
    map<string, Node> nodes;
    map<string, vector<Edge>> adj;

public:
    void add_node(const Node& node) {
        nodes.insert({node.id, node});
    }

    bool has_node(const string& id) const {
        return nodes.count(id);
    }

    void add_edge(const string& id1, const string& id2, double cost) {
        if (has_node(id1) && has_node(id2)) {
            adj[id1].push_back({id2, cost});
            adj[id2].push_back({id1, cost});
        } else {
            cerr << "Warning: Could not create edge. One or both nodes do not exist." << endl;
        }
    }

    void mark_hazard(const string& node_id, const string& reason = "Predicted Debris Intersection") {
        if (has_node(node_id)) {
            nodes.at(node_id).is_hazard = true;
            nodes.at(node_id).hazard_reason = reason;
            cout << "--- COLLISION PREDICTED: Node " << node_id
                 << " marked as HAZARD. Reason: " << reason << " ---" << endl;
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
        if (adj.count(id)) {
            return adj.at(id);
        }
        static const vector<Edge> empty_edges;
        return empty_edges;
    }
};

// --- 2. A* Search Algorithm (Placeholder) ---

double heuristic(const Node& node, const Node& goal_node) {
   /**
     * Heuristic function h(n): Estimates the fuel cost (straight-line distance)
     * from the current node to the goal node. Euclidean distance in 3D.
     */
    double x1, y1, z1, x2, y2, z2;
    tie(x1, y1, z1) = node.pos;
    tie(x2, y2, z2) = goal_node.pos;

    // Standard 3D Euclidean distance formula
    return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2) + pow(z2 - z1, 2));
}

// Custom structure for the priority queue
// The priority queue will store pairs of (f_score, node_id)
// We use a custom comparator or negative cost trick to make it a min-heap
using AStarEntry = pair<double, string>;

pair<vector<string>, double> find_safe_path_astar(const OrbitalGraph& graph, const string& start_id, const string& goal_id) {
    /**
     * Implements the A* search algorithm to find the minimum fuel cost path
     * while avoiding all nodes marked as 'hazard'.
     */
    if (!graph.get_all_nodes().count(start_id) || !graph.get_all_nodes().count(goal_id)) {
        return {{"Error: Start or Goal node not found."}, 0.0};
    }

    const Node& start_node = graph.get_node(start_id);
    const Node& goal_node = graph.get_node(goal_id);

    // g_score: actual cost from start to current node
    map<string, double> g_score;
    // f_score: estimated total cost (g_score + heuristic)
    map<string, double> f_score;
    // came_from: Tracks the optimal previous node for path reconstruction
    map<string, string> came_from;

    // Initialize scores to infinity
    for (const auto& pair : graph.get_all_nodes()) {
        g_score[pair.first] = numeric_limits<double>::infinity();
        f_score[pair.first] = numeric_limits<double>::infinity();
    }

    g_score[start_id] = 0;
    f_score[start_id] = heuristic(start_node, goal_node);

    // Priority Queue (Min-Heap): Stores (f_score, node_id)
    // std::priority_queue is a Max-Heap by default. We push the NEGATIVE f_score
    // to simulate a Min-Heap (largest negative is smallest positive).
    priority_queue<AStarEntry, vector<AStarEntry>, greater<AStarEntry>> open_set;
    open_set.push({f_score[start_id], start_id});

    while (!open_set.empty()) {
        double current_f_score = open_set.top().first;
        string current_id = open_set.top().second;
        open_set.pop();

        // 1. Goal Check
        if (current_id == goal_id) {
            // Path found! Reconstruct and return it.
            vector<string> path;
            double total_cost = g_score[current_id];
            string current = current_id;
            while (current != start_id) {
                path.push_back(current);
                current = came_from[current];
            }
            path.push_back(start_id);
            reverse(path.begin(), path.end());
            return {path, total_cost};
        }

        // 2. Explore Neighbors
        const vector<Edge>& neighbors = graph.get_neighbors(current_id);
        for (const auto& edge : neighbors) {
            string neighbor_id = edge.target_id;
            double edge_cost = edge.cost;

            const Node& neighbor_node = graph.get_node(neighbor_id);

            // COLLISION AVOIDANCE LOGIC: Skip hazard nodes
            if (neighbor_node.is_hazard) {
                continue;
            }

            // Calculate tentative g_score
            double tentative_g_score = g_score[current_id] + edge_cost;

            if (tentative_g_score < g_score[neighbor_id]) {
                // This path is better. Record it.
                came_from[neighbor_id] = current_id;
                g_score[neighbor_id] = tentative_g_score;
                f_score[neighbor_id] = tentative_g_score + heuristic(neighbor_node, goal_node);

                // Add or update the neighbor in the priority queue
                open_set.push({f_score[neighbor_id], neighbor_id});
            }
        }
    }

    // If the loop finishes without finding the goal
    return {{"No path found (Hazard may block all routes)."}, 0.0};
}


// --- 3. User Interaction (Placeholder for now) ---

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
    cout << "\n[Placeholder for Node reading logic]" << endl;
}

void read_edges(OrbitalGraph& graph) {
    cout << "\n[Placeholder for Edge reading logic]" << endl;
}

void read_scenario_parameters(OrbitalGraph& graph, string& start_id, string& goal_id) {
    cout << "\n[Placeholder for Scenario reading logic]" << endl;
    start_id = "A";
    goal_id = "B";
    graph.mark_hazard("C");
}

void run_interactive_demo() {
    cout << "--- Graph-Based Space Debris Collision Avoidance System ---" << endl;

    OrbitalGraph orbit_graph;
    string start_id, goal_id;

    orbit_graph.add_node(Node("A", 0, 0, 0));
    orbit_graph.add_node(Node("B", 1, 1, 1));
    orbit_graph.add_node(Node("C", 2, 2, 2));
    orbit_graph.add_edge("A", "B", 1.0);

    read_nodes(orbit_graph);
    read_edges(orbit_graph);
    read_scenario_parameters(orbit_graph, start_id, goal_id);

    auto result = find_safe_path_astar(orbit_graph, start_id, goal_id);
    cout << "\nSimulation initialized. A* result: " << result.first[0] << endl;
}

int main() {
    run_interactive_demo();
    return 0;
}

