
#include <vector>
#include <algorithm>
#include <cmath>
#include <limits>
#include <iostream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <set>
#include <list>
#include <functional>
#include <fstream>
#include <climits>
#include <cstdlib>
#include <utility>
#include <queue>
#include <random>

#include "../../include/Graph.hpp"
#include "../../include/pugixml.hpp"
#include "../../include/helper.hpp"

using Node = std::string;
using CityGraph = Graph<Node>;

std::unordered_map<std::string, std::pair<double,double>> nodes;      // amenity nodes (lat, lon)
std::vector<std::string> amenityNodes;
std::unordered_map<std::string, std::pair<double,double>> roadNodes;  // road node coords (lat, lon)
CityGraph roadMap;

const double EARTH_RADIUS_KM = 6371.0;
const double PENALTY_FACTOR = 1e10; // Large penalty for disconnection

void parseRoadGraphML(const std::string& filename);
void parseAmenityGraphML(const std::string& filename);
std::string findNearestNode(double queryLat, double queryLon);

static inline double deg2rad(double deg) { return deg * (3.14159265358979323846 / 180.0); }

int haversineDistance(double lat1, double lon1, double lat2, double lon2) {
    double lat1_rad = deg2rad(lat1);
    double lat2_rad = deg2rad(lat2);
    double dlat = deg2rad(lat2 - lat1);
    double dlon = deg2rad(lon2 - lon1);
    double a = std::sin(dlat / 2.0) * std::sin(dlat / 2.0) +
               std::cos(lat1_rad) * std::cos(lat2_rad) * std::sin(dlon / 2.0) * std::sin(dlon / 2.0);
    double c = 2.0 * std::atan2(std::sqrt(a), std::sqrt(1.0 - a));
    return static_cast<int>(EARTH_RADIUS_KM * c * 1000.0); // meters
}

void parseRoadGraphML(const std::string& filename) {
    pugi::xml_document doc;
    pugi::xml_parse_result result = doc.load_file(filename.c_str());
    if (!result) {
        std::cerr << "XML parsing error: " << result.description() << '\n';
        std::cerr << "file path: " << filename << std::endl;
        return;
    }
    pugi::xml_node graph = doc.child("graphml").child("graph");

    for (pugi::xml_node node = graph.child("node"); node; node = node.next_sibling("node")) {
        std::string id = node.attribute("id").value();
        double x = NAN, y = NAN;
        for (pugi::xml_node data = node.child("data"); data; data = data.next_sibling("data")) {
            std::string key = data.attribute("key").value();
            if (key == "d0") x = atof(data.child_value());
            if (key == "d1") y = atof(data.child_value());
        }
        roadNodes[id] = std::make_pair(y, x);
    }

    for (pugi::xml_node edge = graph.child("edge"); edge; edge = edge.next_sibling("edge")) {
        double length = -1;
        for (pugi::xml_node data = edge.child("data"); data; data = data.next_sibling("data")) {
            std::string key = data.attribute("key").value();
            if (key == "d14") {
                length = atof(data.child_value());
                break;
            }
        }
        if (length == -1) continue;
        roadMap.addEdge(edge.attribute("source").value(), edge.attribute("target").value(), static_cast<int>(length), true);
    }
}

void parseAmenityGraphML(const std::string& filename) {
    pugi::xml_document doc;
    pugi::xml_parse_result result = doc.load_file(filename.c_str());
    if (!result) {
        std::cerr << "XML parsing error: " << result.description() << '\n';
        std::cerr << "file path: " << filename << std::endl;
        return;
    }

    pugi::xml_node graph = doc.child("graphml").child("graph");

    for (pugi::xml_node node = graph.child("node"); node; node = node.next_sibling("node")) {
        std::string id = node.attribute("id").value();
        double x = NAN, y = NAN;
        for (pugi::xml_node data = node.child("data"); data; data = data.next_sibling("data")) {
            std::string key = data.attribute("key").value();
            if (key == "d0") x = atof(data.child_value());
            if (key == "d1") y = atof(data.child_value());
        }
        if (std::isnan(x) || std::isnan(y)) continue;
        std::pair<double,double> pt = utm32244ToLatLon(x ,y);
        nodes[id] = pt;
        amenityNodes.push_back(id);
    }
}

std::string findNearestNode(double queryLat, double queryLon) {
    if (roadNodes.empty()) {
        std::cerr << "Error: Road node coordinate map is empty. Cannot find nearest node.\n";
        return "";
    }

    std::string nearestNodeId = "";
    int minDistance = std::numeric_limits<int>::max();

    for (const auto& pair : roadNodes) {
        const std::string& nodeId = pair.first;
        double nodeLat = pair.second.first;
        double nodeLon = pair.second.second;

        int currentDistance = haversineDistance(queryLat, queryLon, nodeLat, nodeLon);

        if (currentDistance < minDistance) {
            minDistance = currentDistance;
            nearestNodeId = nodeId;
        }
    }
    return nearestNodeId;
}

using AdjMap = std::unordered_map<Node, std::vector<std::pair<Node,int>>>;

AdjMap buildAdjFromGraph(const CityGraph &g) {
    AdjMap adj;
    for (const auto &n : g.getAllNodes()) adj[n];

    for (const auto &t : g.getAllEdges()) {
        Node u = std::get<0>(t);
        Node v = std::get<1>(t);
        int w = static_cast<int>(std::get<2>(t));
        adj[u].push_back({v, w});
    }
    return adj;
}

// Remove directed edge u->v from adjacency map (in-place)
void removeEdgeInAdj(AdjMap &adj, const Node &u, const Node &v, bool directed=true) {
    if (adj.count(u)) {
        auto &vec = adj[u];
        vec.erase(std::remove_if(vec.begin(), vec.end(), [&](const std::pair<Node,int> &p){ return p.first == v; }), vec.end());
    }
    if (!directed && adj.count(v)) {
        auto &vec = adj[v];
        vec.erase(std::remove_if(vec.begin(), vec.end(), [&](const std::pair<Node,int> &p){ return p.first == u; }), vec.end());
    }
}

// Dijkstra (local) that returns count of nodes reachable within distanceLimit (inclusive)
int dijkstra_reach_count(const AdjMap &adj, const Node &source, int distanceLimit) {
    if (!adj.count(source)) return 0;
    const int INF = std::numeric_limits<int>::max();

    std::unordered_map<Node,int> dist;
    dist.reserve(adj.size());
    for (const auto &p : adj) dist[p.first] = INF;
    dist[source] = 0;

    using Pair = std::pair<int, Node>; // (distance, node)
    std::priority_queue<Pair, std::vector<Pair>, std::greater<Pair>> pq;
    pq.push({0, source});

    int count = 0;
    while (!pq.empty()) {
        auto cur = pq.top(); pq.pop();
        int d = cur.first;
        Node u = cur.second;

        if (d > distanceLimit) continue;
        if (d != dist[u]) continue;

        count++;
        auto it = adj.find(u);
        if (it == adj.end()) continue;
        for (const auto &nbr : it->second) {
            const Node &v = nbr.first;
            int w = nbr.second;
            long long nd = static_cast<long long>(d) + static_cast<long long>(w);
            if (nd < dist[v] && nd <= distanceLimit) {
                dist[v] = static_cast<int>(nd);
                pq.push({dist[v], v});
            }
        }
    }
    
    return count;
}

// Compute reachability counts for list of center nodes and return mapping
std::unordered_map<Node,int> computeReachabilityForCenters(const AdjMap &adj, const std::vector<Node> &centers, int distanceLimit) {
    std::unordered_map<Node,int> out;
    for (const auto &c : centers) {
        out[c] = dijkstra_reach_count(adj, c, distanceLimit);
    }
    return out;
}

// Sum of reachability counts
long long sumReachability(const std::unordered_map<Node,int> &m) {
    long long s = 0;
    for (const auto &p : m) s += p.second;
    return s;
}

int main() {
    const std::string ROAD_FILE = "../../data/roads.graphml";
    const std::string AMENITY_FILE = "../../data/amenity.graphml";

    std::cerr << "Parsing road network from " << ROAD_FILE << std::endl;
    parseRoadGraphML(ROAD_FILE);

    std::cerr << "Parsing amenity data from " << AMENITY_FILE << std::endl;
    parseAmenityGraphML(AMENITY_FILE);

    CityGraph baseGraph = roadMap;
    auto allNodes = baseGraph.getAllNodes();
    std::cerr << "Road graph nodes loaded: " << allNodes.size() << std::endl;

    if (allNodes.empty()) {
        std::cerr << "FATAL ERROR: No nodes loaded into the CityGraph. Check XML file path/format." << std::endl;
        return 1;
    }

    // Map each amenity to nearest road node, and use that road node as center
    std::vector<Node> centers;
    for (const auto &pair : nodes) {
        double amenity_lat = pair.second.first;
        double amenity_lon = pair.second.second;
        std::string nearest = findNearestNode(amenity_lat, amenity_lon);
        if (!nearest.empty()) centers.push_back(nearest);
    }

    if (centers.empty()) {
        std::cerr << "FATAL ERROR: No amenity centers mapped. Cannot run analysis." << std::endl;
        return 1;
    }

    // Build adjacency map once
    AdjMap adj = buildAdjFromGraph(baseGraph);

    // Baseline reachability
    const int DIST_LIMIT = 1000; // meters
    auto baseline_counts = computeReachabilityForCenters(adj, centers, DIST_LIMIT);
    long long baseline_sum = sumReachability(baseline_counts);

    std::cout << "Baseline total reachability (sum over centers): " << baseline_sum << "\n";

    // Get all edges to test
    std::vector<std::tuple<Node,Node,int>> edges = baseGraph.getAllEdges();
    if (edges.empty()) {
        std::cerr << "No edges found to test.\n";
        return 1;
    }

    std::cout << "\n--- Edge vulnerability (reachability drop) ---\n";
    const size_t MAX_TEST_EDGES = 20;
    size_t tested = 0;

    for (const auto &e : edges) {
        if (tested >= MAX_TEST_EDGES) break;
        Node u = std::get<0>(e);
        Node v = std::get<1>(e);
        int w = static_cast<int>(std::get<2>(e));

        AdjMap adj_copy = adj;
        removeEdgeInAdj(adj_copy, u, v, true);

        auto failed_counts = computeReachabilityForCenters(adj_copy, centers, DIST_LIMIT);
        long long failed_sum = sumReachability(failed_counts);

        long long drop = baseline_sum - failed_sum;
        std::cout << u << " -> " << v << " | reachability drop = " << drop << "\n";

        tested++;
    }

    std::cout << "\nAnalysis done. Tested " << tested << " edges (of " << edges.size() << " total).\n";

    std::cout << "\nOptimal New Road Suggestion\n";

    long long best_gain = 0;
    std::pair<Node, Node> best_pair;
    const int NEW_ROAD_LENGTH = 500; // meters (assumed short connector)
    const int SAMPLE_LIMIT = 200;    // limit number of candidate pairs checked

    // Sample random pairs of nodes and test reachability improvement
    std::vector<Node> nodeList = baseGraph.getAllNodes();
    std::mt19937 rng(42);
    std::uniform_int_distribution<size_t> dist(0, nodeList.size() - 1);

    for (int i = 0; i < SAMPLE_LIMIT; ++i) {
        Node a = nodeList[dist(rng)];
        Node b = nodeList[dist(rng)];
        if (a == b) continue;

        // Skip if already directly connected
        bool connected = false;
        for (auto &p : adj[a]) {
            if (p.first == b) { connected = true; break; }
        }
        if (connected) continue;

        // Simulate adding edge
        AdjMap adj_test = adj;
        adj_test[a].push_back({b, NEW_ROAD_LENGTH});
        adj_test[b].push_back({a, NEW_ROAD_LENGTH}); // bidirectional

        auto new_counts = computeReachabilityForCenters(adj_test, centers, DIST_LIMIT);
        long long new_sum = sumReachability(new_counts);
        long long gain = new_sum - baseline_sum;

        if (gain > best_gain) {
            best_gain = gain;
            best_pair = {a, b};
        }
    }

    if (best_gain > 0) {
        std::cout << "Suggested new road: " << best_pair.first
                  << " - " << best_pair.second
                  << "\nReachability gain = +" << best_gain << "\n";
    } else {
        std::cout << "No beneficial new road found in tested pairs.\n";
    }

    std::cout << "\nEnd of Analysis\n";
    return 0;
}