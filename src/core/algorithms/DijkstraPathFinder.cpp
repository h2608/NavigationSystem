#include "core/algorithms/DijkstraPathFinder.h"
#include <queue>
#include <limits>
#include <algorithm>

namespace nav {

PathResult DijkstraPathFinder::findPath(const Graph& graph, Node::Id start, Node::Id end) {
    PathResult result;

    // Verify start and end nodes exist
    if (graph.getNode(start) == nullptr || graph.getNode(end) == nullptr) {
        result.found = false;
        return result;
    }

    // If start == end, return trivial path
    if (start == end) {
        result.pathNodes.push_back(start);
        result.totalCost = 0.0;
        result.found = true;
        return result;
    }

    // Distance map (lazy initialization - no need to set all to infinity)
    std::unordered_map<Node::Id, double> dist;
    dist.reserve(graph.getNodeCount());
    dist[start] = 0.0;

    // Predecessor map: stores which edge led to this node
    std::unordered_map<Node::Id, Edge::Id> predecessor;

    // Priority queue: (distance, nodeId)
    // Note: std::priority_queue is a max heap, so we negate distances
    using PQEntry = std::pair<double, Node::Id>;
    std::priority_queue<PQEntry, std::vector<PQEntry>, std::greater<PQEntry>> pq;

    pq.push({0.0, start});

    // Visited set to avoid reprocessing
    std::unordered_map<Node::Id, bool> visited;

    while (!pq.empty()) {
        auto [currentDist, currentNode] = pq.top();
        pq.pop();

        // Skip if already visited
        if (visited[currentNode]) {
            continue;
        }
        visited[currentNode] = true;

        // Early termination: if we reached the end node, we're done
        if (currentNode == end) {
            break;
        }

        // Explore neighbors
        const auto& adjacentEdges = graph.getAdjacentEdges(currentNode);
        for (Edge::Id edgeId : adjacentEdges) {
            const Edge* edge = graph.getEdge(edgeId);
            if (!edge) continue;

            // Determine the neighbor node (other endpoint of the edge)
            Node::Id neighbor = (edge->getSource() == currentNode) ? edge->getTarget() : edge->getSource();

            // Skip if already visited
            if (visited[neighbor]) {
                continue;
            }

            // Calculate new distance
            double newDist = dist[currentNode] + edge->getLength();

            // If this is a better path, update (missing entry = infinity)
            auto it = dist.find(neighbor);
            double currentBest = (it != dist.end()) ? it->second : std::numeric_limits<double>::infinity();
            if (newDist < currentBest) {
                dist[neighbor] = newDist;
                predecessor[neighbor] = edgeId;
                pq.push({newDist, neighbor});
            }
        }
    }

    // Check if path was found
    auto endIt = dist.find(end);
    if (endIt == dist.end() || endIt->second == std::numeric_limits<double>::infinity()) {
        result.found = false;
        return result;
    }

    // Reconstruct path
    result.found = true;
    result.totalCost = endIt->second;
    reconstructPath(graph, start, end, predecessor, result);

    return result;
}

void DijkstraPathFinder::reconstructPath(const Graph& graph,
                                         Node::Id start,
                                         Node::Id end,
                                         const std::unordered_map<Node::Id, Edge::Id>& predecessor,
                                         PathResult& result) const {
    // Reconstruct path by following predecessor edges backwards
    std::vector<Node::Id> reversePath;
    std::vector<Edge::Id> reverseEdges;

    Node::Id current = end;
    reversePath.push_back(current);

    while (current != start) {
        auto it = predecessor.find(current);
        if (it == predecessor.end()) {
            // Path reconstruction failed (shouldn't happen if path exists)
            result.found = false;
            result.pathNodes.clear();
            result.pathEdges.clear();
            return;
        }

        Edge::Id edgeId = it->second;
        const Edge* edge = graph.getEdge(edgeId);
        if (!edge) {
            result.found = false;
            result.pathNodes.clear();
            result.pathEdges.clear();
            return;
        }

        reverseEdges.push_back(edgeId);

        // Move to the other endpoint of the edge
        current = (edge->getSource() == current) ? edge->getTarget() : edge->getSource();
        reversePath.push_back(current);
    }

    // Reverse to get path from start to end
    result.pathNodes.assign(reversePath.rbegin(), reversePath.rend());
    result.pathEdges.assign(reverseEdges.rbegin(), reverseEdges.rend());
}

} // namespace nav
