#pragma once

#include <iostream>
#include <unordered_map>
#include <vector>
#include <list>
#include <limits>
#include <functional>
#include <algorithm>
#include "Queue.h"
#include "PriorityQueue.h"
#include "Stack.h"

template <typename T>
class Graph {
private:
    // --- Member Variables ---
    std::unordered_map<T, std::list<std::pair<T, int>>> adjList;

    // --- Helper Structs for Algorithms ---
    struct DijkstraNode {
        T vertex;
        int distance;
        // This comparator is correct, assuming your PriorityQueue is a min-queue
        bool operator<(const DijkstraNode& other) const { return distance < other.distance; }
    };

    struct AStarNode {
        T vertex;
        int f_cost; // g_cost + h_cost
        bool operator<(const AStarNode& other) const { return f_cost < other.f_cost; }
    };

    // --- Private Helper Functions ---
    void _dfs(T node, std::unordered_map<T, bool>& visited) {
        visited[node] = true;
        std::cout << node << " ";
        for (auto const& neighbor_pair : adjList[node]) {
            T neighbor = neighbor_pair.first;
            if (!visited[neighbor]) {
                _dfs(neighbor, visited);
            }
        }
    }

    bool _hasCycle(T node, std::unordered_map<T, bool>& visited, std::unordered_map<T, bool>& recursionStack) {
        visited[node] = true;
        recursionStack[node] = true;
        for (auto const& neighbor_pair : adjList[node]) {
            T neighbor = neighbor_pair.first;
            if (!visited[neighbor]) {
                if (_hasCycle(neighbor, visited, recursionStack)) {
                    return true;
                }
            } else if (recursionStack[neighbor]) {
                return true;
            }
        }
        recursionStack[node] = false;
        return false;
    }

    // Private helper to reconstruct the path once the goal is found
    std::vector<T> _reconstructPath(std::unordered_map<T, T>& came_from, T current) {
        std::vector<T> total_path;
        total_path.push_back(current);
        while (came_from.count(current)) {
            current = came_from[current];
            total_path.push_back(current);
        }
        std::reverse(total_path.begin(), total_path.end());
        return total_path;
    }

public:
    // Adds an edge between two vertices.
    void addEdge(T u, T v, int weight, bool isDirected = true) {
        adjList[u].push_back({v, weight});
        if (!isDirected) {
            adjList[v].push_back({u, weight});
        }
    }
    
    // Prints the adjacency list representation of the graph.
    void printGraph() {
    std::cout << "--- Adjacency List ---" << std::endl;
    for (const auto& pair : adjList) {
        T vertex = pair.first;
        const std::list<std::pair<T, int>>& neighbors = pair.second;
        std::cout << vertex << " -> ";
        for (const auto& edge : neighbors) {
            std::cout << "{" << edge.first << ", " << edge.second << "} ";
        }
        std::cout << std::endl;
    }
    std::cout << "----------------------" << std::endl;
    }

    // Breadth first search
    void bfs(T startNode) {
        std::cout << "BFS Traversal (custom Queue): ";
        std::unordered_map<T, bool> visited;
        Queue<T> q;
        q.enqueue(startNode);
        visited[startNode] = true;
        while (!q.isEmpty()) {
            T currentNode = q.dequeue();
            std::cout << currentNode << " ";
            for (auto const& neighbor_pair : adjList[currentNode]) {
                T neighbor = neighbor_pair.first;
                if (!visited[neighbor]) {
                    visited[neighbor] = true;
                    q.enqueue(neighbor);
                }
            }
        }
        std::cout << std::endl;
    }

    // Depth first search
    void dfs(T startNode) {
        std::cout << "DFS Traversal (recursive): ";
        std::unordered_map<T, bool> visited;
        _dfs(startNode, visited);
        std::cout << std::endl;
    }

    // Dijkstra algorithm to find the shortest path between 2 nodes
    void dijkstra(T startNode, T endNode) {
        std::unordered_map<T, int> distances;
        std::unordered_map<T, T> predecessors;
        for (auto const& pair : adjList) {
            if (distances.find(pair.first) == distances.end()) {
                 distances[pair.first] = std::numeric_limits<int>::max();
            }
            for (auto const& edge : pair.second) {
                if (distances.find(edge.first) == distances.end()) {
                    distances[edge.first] = std::numeric_limits<int>::max();
                }
            }
        }
        if (distances.find(startNode) == distances.end()) {
            distances[startNode] = std::numeric_limits<int>::max();
        }
        if (distances.find(endNode) == distances.end()) {
            distances[endNode] = std::numeric_limits<int>::max();
        }
        distances[startNode] = 0;
        PriorityQueue<DijkstraNode> pq;
        pq.push({startNode, 0});
        while(!pq.isEmpty()) {
            DijkstraNode topNode = pq.top();
            pq.pop();
            T u = topNode.vertex;
            int dist = topNode.distance;
            if (u == endNode) {
                break;
            }
            if (dist > distances[u]) {
                continue;
            }
            for (auto const& neighbor_pair : adjList[u]) {
                T v = neighbor_pair.first;
                int weight = neighbor_pair.second;
                if (distances[u] != std::numeric_limits<int>::max() && distances[u] + weight < distances[v]) {
                    distances[v] = distances[u] + weight;
                    predecessors[v] = u;
                    pq.push({v, distances[v]});
                }
            }
        }
        std::cout << "--- Dijkstra's Shortest Path (" << startNode << " to " << endNode << ") ---" << std::endl;
        if (distances[endNode] == std::numeric_limits<int>::max()) {
            std::cout << "No path found from " << startNode << " to " << endNode << "." << std::endl;
        } else {
            std::vector<T> path;
            T current = endNode;
            while (predecessors.count(current)) {
                path.push_back(current);
                current = predecessors[current];
            }
            path.push_back(startNode);
            std::reverse(path.begin(), path.end());
            std::cout << "Shortest distance: " << distances[endNode] << std::endl;
            std::cout << "Path: ";
            for (size_t i = 0; i < path.size(); ++i) {
                std::cout << path[i] << (i == path.size() - 1 ? "" : " -> ");
            }
            std::cout << std::endl;
        }
        std::cout << "------------------------------------------" << std::endl;
    }

    // Checks if the graph has a cycle
    bool hasCycle() {
        std::unordered_map<T, bool> visited;
        std::unordered_map<T, bool> recursionStack;
        for (auto const& pair : adjList) {
            T node = pair.first;
            if (!visited[node]) {
                if (_hasCycle(node, visited, recursionStack)) {
                    return true;
                }
            }
        }
        return false;
    }

    // A* algorithm to find the shortest path between 2 nodes
    std::vector<T> a_star(T startNode, T goalNode, std::function<int(T, T)> heuristic) {
        PriorityQueue<AStarNode> open_set;
        std::unordered_map<T, T> came_from;
        std::unordered_map<T, int> g_cost;
        for (auto const& pair : adjList) {
            if (g_cost.find(pair.first) == g_cost.end()) {
                 g_cost[pair.first] = std::numeric_limits<int>::max();
            }
            for (auto const& edge : pair.second) {
                if (g_cost.find(edge.first) == g_cost.end()) {
                    g_cost[edge.first] = std::numeric_limits<int>::max();
                }
            }
        }
        if (g_cost.find(startNode) == g_cost.end()) {
            g_cost[startNode] = std::numeric_limits<int>::max();
        }
        if (g_cost.find(goalNode) == g_cost.end()) {
            g_cost[goalNode] = std::numeric_limits<int>::max();
        }
        g_cost[startNode] = 0;
        open_set.push({startNode, heuristic(startNode, goalNode)});
        while (!open_set.isEmpty()) {
            T current = open_set.top().vertex;
            open_set.pop();
            if (current == goalNode) {
                return _reconstructPath(came_from, current);
            }
            for (auto const& neighbor_pair : adjList[current]) {
                T neighbor = neighbor_pair.first;
                int weight = neighbor_pair.second;
                if (g_cost[current] == std::numeric_limits<int>::max()) {
                    continue;
                }
                int tentative_g_cost = g_cost[current] + weight;
                if (tentative_g_cost < g_cost[neighbor]) {
                    came_from[neighbor] = current;
                    g_cost[neighbor] = tentative_g_cost;
                    int f_cost = g_cost[neighbor] + heuristic(neighbor, goalNode);
                    open_set.push({neighbor, f_cost});
                }
            }
        }
        return std::vector<T>();
    }
};
