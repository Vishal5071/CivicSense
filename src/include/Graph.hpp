#pragma once

#include <iostream>
#include <unordered_map>
#include <vector>
#include <list>
#include <limits>
#include <functional>
#include <algorithm>
#include "Queue.hpp"
#include "PriorityQueue.hpp"
#include "Stack.hpp"

template <typename T>
class Graph {
private:
    // Member Variables
    std::unordered_map<T, std::list<std::pair<T, int>>> adjList;

    // Helper Structs for Algorithms
    struct DijkstraNode {
        T vertex;
        int distance;
        bool operator<(const DijkstraNode& other) const { return distance < other.distance; }
    };

    struct AStarNode {
        T vertex;
        int f_cost;
        bool operator<(const AStarNode& other) const { return f_cost < other.f_cost; }
    };

    // Private Helper Functions
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
	std::vector<T> _reconstructPath(const std::unordered_map<T, T>& came_from, T current) {
		std::vector<T> total_path;
		
		total_path.push_back(current);
		size_t max_iterations = came_from.size() + 2; 
		size_t iteration_count = 0;

		while (came_from.count(current)) {
		    if (++iteration_count > max_iterations) {
		        std::cerr << "Error: Cycle detected during path reconstruction. Breaking loop." << std::endl;
		        return {};
		    }

		    current = came_from.at(current);
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
		std::cout << "Adjacency List :" << std::endl;
		for (const auto& pair : adjList) {
		    T vertex = pair.first;
		    const std::list<std::pair<T, int>>& neighbors = pair.second;
		    std::cout << vertex << " -> ";
		    for (const auto& edge : neighbors) {
		        std::cout << "{" << edge.first << ", " << edge.second << "} ";
		    }
		    std::cout << std::endl;
		}
		std::cout << "End of graph" << std::endl;
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

    // Dijkstras algorithm finds the shortest distance between two nodes
	std::vector<T> dijkstra(T startNode, T endNode) {
        std::unordered_map<T, int> distances;
        std::unordered_map<T, T> predecessors;
        PriorityQueue<DijkstraNode> pq;

        distances[startNode] = 0;
        pq.push({startNode, 0});

        auto get_distance = [&](T node) -> int {
            if (distances.count(node)) {
                return distances.at(node);
            }
            return std::numeric_limits<int>::max();
        };

        while (!pq.isEmpty()) {
            DijkstraNode topNode = pq.top();
            pq.pop();
            T u = topNode.vertex;
            int dist = topNode.distance;

            if (dist > distances[u]) {
                continue;
            }
            if (u == endNode) {
                break; 
            }

            for (auto const& neighbor_pair : adjList[u]) {
                T v = neighbor_pair.first;
                int weight = neighbor_pair.second;
                int new_distance = distances[u] + weight;

                int current_v_distance = get_distance(v);

                if (new_distance < current_v_distance) {
                    distances[v] = new_distance;
                    predecessors[v] = u;
                    pq.push({v, distances[v]});
                }
            }
        }

        if (get_distance(endNode) == std::numeric_limits<int>::max()) {
            return {};
        } else {
            std::vector<T> path = _reconstructPath(predecessors, endNode); 
            
            if (path.empty() && endNode != startNode) {
                return {};
            }
            return path;
        }
    }

    // A* algorithm finds the shortest distance between 2 nodes
    std::vector<T> a_star(T startNode, T goalNode, std::function<int(T, T)> heuristic) {
        PriorityQueue<AStarNode> open_set;
        std::unordered_map<T, T> came_from;
        std::unordered_map<T, int> g_cost;

        g_cost[startNode] = 0;
        
        int start_f_cost = heuristic(startNode, goalNode);
        open_set.push({startNode, start_f_cost});

        auto get_g_cost = [&](T node) -> int {
            if (g_cost.count(node)) {
                return g_cost.at(node);
            }
            return std::numeric_limits<int>::max();
        };

        while (!open_set.isEmpty()) {
            T current = open_set.top().vertex;
            open_set.pop();

            if (current == goalNode) {
                return _reconstructPath(came_from, current);
            }

            for (auto const& neighbor_pair : adjList[current]) {
                T neighbor = neighbor_pair.first;
                int weight = neighbor_pair.second;

                int tentative_g_cost = g_cost[current] + weight;
                int neighbor_g_cost = get_g_cost(neighbor);

                if (tentative_g_cost < neighbor_g_cost) {
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
