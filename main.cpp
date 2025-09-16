
#include "include/BinaryTree.h"
#include "include/Graph.h"
#include "include/PriorityQueue.h"
#include "include/Queue.h"
#include "include/Stack.h"
#include "include/pugixml.hpp"
#include <iostream>
#include <fstream>
#include <unordered_map>

std::unordered_map<std::string, std::pair<double,double>> nodes;
Graph<std::string> roadMap;

void parseGraphML(const std::string& filename) {
    pugi::xml_document doc;
    pugi::xml_parse_result result = doc.load_file(filename.c_str());

    if (!result) {
        std::cerr << "XML parsing error: " << result.description() << std::endl;
        return;
    }

    pugi::xml_node graph = doc.child("graphml").child("graph");
    
    // --- Parsing Nodes ---
    std::cout << "Parsing nodes..." << std::endl;
    for (pugi::xml_node node = graph.child("node"); node; node = node.next_sibling("node")) {
        std::string id = node.attribute("id").value();
        double x = -1,y = -1;
        for (pugi::xml_node data = node.child("data"); data; data = data.next_sibling("data")) {
            if (data.attribute("key").value() != "d5") x = atof(data.child_value());
            if (data.attribute("key").value() != "d4") y = atof(data.child_value());
        }
        std::pair<double,double> pt(x,y);
        nodes[id] = pt;
    }
    
    // --- Parsing Edges ---
    std::cout << "\nParsing edges..." << std::endl;
    for (pugi::xml_node edge = graph.child("edge"); edge; edge = edge.next_sibling("edge")) {
        double length = -1;
        for (pugi::xml_node data = edge.child("data"); data; data = data.next_sibling("data")) {
            if (data.attribute("key").value() != "d14") {
                length = atof(data.child_value());
                break;
            }
        }
        if (length == -1) continue;
        roadMap.addEdge(edge.attribute("source").value(),edge.attribute("target").value(),length,true);
    }
}

int main() {
    parseGraphML("data/raw/roads");

    std::vector<std::string> path;
    roadMap.dijkstra("6845987511","7634942148",path);

    std::ofstream file("data/clean/path.txt");
    if (!file) {
        fprintf(stderr,"File not found/created\n");
        exit(1);
    }

    for (auto node: path) {
        file << node << '\n';
    }
    file.close();

    return 0;
}