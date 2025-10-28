
#include "../include/BinaryTree.hpp"
#include "../include/Graph.hpp"
#include "../include/PriorityQueue.hpp"
#include "../include/Queue.hpp"
#include "../include/Stack.hpp"
#include "../include/pugixml.hpp"
#include <iostream>
#include <fstream>
#include <string>
#include <cmath>
#include <unordered_map>

std::unordered_map<std::string, std::pair<double,double>> nodes;	// node_id -> (longitude, latitude)
Graph<std::string> roadMap;

const double EARTH_RADIUS_KM = 6371.0;
const double PI = 3.14159265358979323846;

// Helper function to convert degrees to radians
double toRadians(double degrees) {
    return degrees * (PI / 180.0);
}

//Function to calculate distances between coordinates (heuristic for A*)
int haversineDistance(double lat1, double lon1, double lat2, double lon2) {
    // Convert all coordinates from degrees to radians
    double lat1_rad = toRadians(lat1);
    double lat2_rad = toRadians(lat2);
    double dlat = toRadians(lat2 - lat1);
    double dlon = toRadians(lon2 - lon1);

    // Apply the Haversine formula core calculation
    double a = std::sin(dlat / 2.0) * std::sin(dlat / 2.0) +
               std::cos(lat1_rad) * std::cos(lat2_rad) *
               std::sin(dlon / 2.0) * std::sin(dlon / 2.0);

    // Calculate the central angle (c)
    double c = 2.0 * std::atan2(std::sqrt(a), std::sqrt(1.0 - a));

    // Calculate the final distance (d = R * c)
    double distance = EARTH_RADIUS_KM * c;
	
	// We want the distance in meters
    return distance * 1000;
}

int EuclideanDistance(std::string node1, std::string node2) {
	double lat1 = nodes[node1].second, lat2 = nodes[node2].second;
	double long1 = nodes[node1].first, long2 = nodes[node2].first;
	return haversineDistance(lat1, long1, lat2, long2);
}

// Function to parse the xml file that has city data
void parseGraphML(const std::string& filename) {
	// Parsing the xml file pugixml
    pugi::xml_document doc;
    pugi::xml_parse_result result = doc.load_file(filename.c_str());
	
	// Handling errors
    if (!result) {
        std::cerr << "XML parsing error: " << result.description() << std::endl;
        return;
    }

    pugi::xml_node graph = doc.child("graphml").child("graph");
    
    // Parsing Nodes
    for (pugi::xml_node node = graph.child("node"); node; node = node.next_sibling("node")) {
        std::string id = node.attribute("id").value();
        double x = -1,y = -1;
        for (pugi::xml_node data = node.child("data"); data; data = data.next_sibling("data")) {
        	std::string key = data.attribute("key").value();
            if (key == "d5") x = atof(data.child_value());
            if (key == "d4") y = atof(data.child_value());
        }
        std::pair<double,double> pt(x,y);
        nodes[id] = pt;
    }
    
    // Parsing Edges
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
        roadMap.addEdge(edge.attribute("source").value(),edge.attribute("target").value(),length,true);
    }
}

// Function to find the shortest path using Dijkstra's algorithm
void pathDijkstra(std::string start, std::string end) {
	std::vector<std::string> path;
    path = roadMap.dijkstra(start, end);

    std::ofstream file("data/clean/path.txt");
    if (!file) {
        fprintf(stderr,"File not found/created\n");
        exit(1);
    }

    for (auto node: path) {
        file << node << '\n';
    }
    file.close();
}

// Function to find the shortest path using A star algorithm
void pathAstar(std::string start, std::string end) {
	std::vector<std::string> path;
    path = roadMap.a_star(start, end, EuclideanDistance);

    std::ofstream file("data/clean/path.txt");
    if (!file) {
        fprintf(stderr,"File not found/created\n");
        exit(1);
    }

    for (auto node: path) {
        file << node << '\n';
    }
    file.close();
}

int main() {
    parseGraphML("data/roads.xml");
    pathDijkstra("976696739","1254517233");

    return 0;
}
