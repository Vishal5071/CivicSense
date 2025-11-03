#pragma once

#include <iostream>
#include <unordered_map>
#include <vector>
#include <set>
#include <list>
#include <limits>
#include <functional>
#include <algorithm>
#include <tuple>
#include "Queue.hpp"
#include "PriorityQueue.hpp"
#include "Stack.hpp"

template <typename T>
class Graph {
private:
    // Adjacency list representation
    std::unordered_map<T, std::list<std::pair<T, int>>> adjList;

    // Internal helper structures
    struct DijkstraNode {
        T vertex;
        int distance;
        bool operator>(const DijkstraNode& other) const { return distance > other.distance; }
        bool operator<(const DijkstraNode& other) const { return distance < other.distance; }
    };

    struct AStarNode {
        T vertex;
        int f_cost;
        bool operator>(const AStarNode& other) const { return f_cost > other.f_cost; }
        bool operator<(const AStarNode& other) const { return f_cost < other.f_cost; }
    };

    // Recursive DFS helper
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

    // Detect cycles helper
    bool _hasCycle(T node, std::unordered_map<T, bool>& visited, std::unordered_map<T, bool>& recursionStack) {
        visited[node] = true;
        recursionStack[node] = true;
        for (auto const& neighbor_pair : adjList[node]) {
            T neighbor = neighbor_pair.first;
            if (!visited[neighbor]) {
                if (_hasCycle(neighbor, visited, recursionStack)) return true;
            } else if (recursionStack[neighbor]) return true;
        }
        recursionStack[node] = false;
        return false;
    }

    // Path reconstruction helper
    std::vector<T> _reconstructPath(const std::unordered_map<T, T>& came_from, T current) {
        std::vector<T> path;
        path.push_back(current);
        size_t max_iterations = came_from.size() + 2;
        size_t iter = 0;

        while (came_from.count(current)) {
            if (++iter > max_iterations) {
                std::cerr << "Error: Path reconstruction loop detected.\n";
                return {};
            }
            current = came_from.at(current);
            path.push_back(current);
        }

        std::reverse(path.begin(), path.end());
        return path;
    }

public:
    std::set<T> nodes;

    // Add edge to graph
    void addEdge(T u, T v, int weight, bool isDirected = true) {
        nodes.insert(u);
        nodes.insert(v);
        adjList[u].push_back({v, weight});
        if (!isDirected) adjList[v].push_back({u, weight});
    }

    // Print adjacency list
    void printGraph() {
        std::cout << "Adjacency List:\n";
        for (const auto& [u, neighbors] : adjList) {
            std::cout << u << " -> ";
            for (const auto& [v, w] : neighbors)
                std::cout << "{" << v << ", " << w << "} ";
            std::cout << "\n";
        }
        std::cout << "End of graph.\n";
    }

    // Breadth-First Search
    void bfs(T startNode) {
        std::cout << "BFS Traversal: ";
        std::unordered_map<T, bool> visited;
        Queue<T> q;
        q.enqueue(startNode);
        visited[startNode] = true;

        while (!q.isEmpty()) {
            T current = q.dequeue();
            std::cout << current << " ";
            for (auto const& [neighbor, _] : adjList[current]) {
                if (!visited[neighbor]) {
                    visited[neighbor] = true;
                    q.enqueue(neighbor);
                }
            }
        }
        std::cout << "\n";
    }

    // Depth-First Search
    void dfs(T startNode) {
        std::cout << "DFS Traversal: ";
        std::unordered_map<T, bool> visited;
        _dfs(startNode, visited);
        std::cout << "\n";
    }

    // Dijkstra’s algorithm
    std::vector<T> dijkstra(T startNode, T endNode) {
        std::unordered_map<T, int> dist;
        std::unordered_map<T, T> prev;
        PriorityQueue<DijkstraNode> pq;

        for (const auto& n : nodes) dist[n] = std::numeric_limits<int>::max();
        dist[startNode] = 0;
        pq.push({startNode, 0});

        while (!pq.isEmpty()) {
            auto [u, d] = pq.top();
            pq.pop();

            if (d > dist[u]) continue;
            if (u == endNode) break;

            for (auto const& [v, w] : adjList[u]) {
                int newDist = dist[u] + w;
                if (newDist < dist[v]) {
                    dist[v] = newDist;
                    prev[v] = u;
                    pq.push({v, newDist});
                }
            }
        }

        if (dist[endNode] == std::numeric_limits<int>::max()) return {};
        return _reconstructPath(prev, endNode);
    }

    // A* pathfinding
    std::vector<T> a_star(T startNode, T goalNode, std::function<int(T, T)> heuristic) {
        PriorityQueue<AStarNode> open;
        std::unordered_map<T, T> came_from;
        std::unordered_map<T, int> g_cost;

        g_cost[startNode] = 0;
        open.push({startNode, heuristic(startNode, goalNode)});

        while (!open.isEmpty()) {
            T current = open.top().vertex;
            open.pop();

            if (current == goalNode) return _reconstructPath(came_from, current);

            for (auto const& [neighbor, weight] : adjList[current]) {
                int tentative_g = g_cost[current] + weight;
                if (!g_cost.count(neighbor) || tentative_g < g_cost[neighbor]) {
                    came_from[neighbor] = current;
                    g_cost[neighbor] = tentative_g;
                    int f_cost = tentative_g + heuristic(neighbor, goalNode);
                    open.push({neighbor, f_cost});
                }
            }
        }
        return {};
    }

    // Return shortest path distance (Dijkstra)
    int pathDistance(T startNode, T endNode) {
        std::unordered_map<T, int> dist;
        PriorityQueue<DijkstraNode> pq;

        for (const auto& n : nodes) dist[n] = std::numeric_limits<int>::max();
        dist[startNode] = 0;
        pq.push({startNode, 0});

        while (!pq.isEmpty()) {
            auto [u, d] = pq.top();
            pq.pop();

            if (d > dist[u]) continue;
            if (u == endNode) break;

            for (auto const& [v, w] : adjList[u]) {
                int newDist = dist[u] + w;
                if (newDist < dist[v]) {
                    dist[v] = newDist;
                    pq.push({v, newDist});
                }
            }
        }
        return dist[endNode];
    }

    // Remove edge
    void removeEdge(T u, T v, bool isDirected = true) {
        if (adjList.count(u)) {
            adjList[u].remove_if([&](const std::pair<T, int>& edge) {
                return edge.first == v;
            });
        }
        if (!isDirected && adjList.count(v)) {
            adjList[v].remove_if([&](const std::pair<T, int>& edge) {
                return edge.first == u;
            });
        }
    }

    // Return all edges (unique)
    std::vector<std::tuple<T, T, int>> getAllEdges() const {
        std::vector<std::tuple<T, T, int>> edges;
        std::set<std::pair<T, T>> seen;
        for (const auto& [u, neighbors] : adjList) {
            for (const auto& [v, w] : neighbors) {
                if (!seen.count({v, u})) {
                    edges.emplace_back(u, v, w);
                    seen.insert({u, v});
                }
            }
        }
        return edges;
    }

    // Return all nodes
    std::vector<T> getAllNodes() const {
        return std::vector<T>(nodes.begin(), nodes.end());
    }

    // Check if an edge exists
    bool hasEdge(T u, T v) const {
        if (!adjList.count(u)) return false;
        for (const auto& [neighbor, _] : adjList.at(u))
            if (neighbor == v) return true;
        return false;
    }

    // Counts how many nodes can be reached from a given start node
    int reachabilityCount(T startNode, int maxDistance) {
        if (!nodes.count(startNode)) return 0;

        std::unordered_map<T, int> dist;
        for (const auto& n : nodes)
            dist[n] = std::numeric_limits<int>::max();

        dist[startNode] = 0;
        PriorityQueue<DijkstraNode> pq;
        pq.push({startNode, 0});

        while (!pq.isEmpty()) {
            auto [u, d] = pq.top();
            pq.pop();

            if (d > maxDistance) continue;

            for (auto const& [v, w] : adjList[u]) {
                int newDist = d + w;
                if (newDist < dist[v] && newDist <= maxDistance) {
                    dist[v] = newDist;
                    pq.push({v, newDist});
                }
            }
        }

        int count = 0;
        for (auto const& [node, d] : dist) {
            if (d <= maxDistance && node != startNode)
                count++;
        }

        return count;
    }
};
