#include "core/graph/Graph.h"
#include <queue>
#include <unordered_set>

namespace nav {

const std::vector<Edge::Id> Graph::emptyEdgeList_;

Graph::Graph() : nextNodeId_(0), nextEdgeId_(0) {}

Node::Id Graph::addNode(const Point2D& position) {
    return addNodeWithId(nextNodeId_++, position);
}

Node::Id Graph::addNodeWithId(Node::Id id, const Point2D& position) {
    nodes_.emplace(id, Node(id, position));
    adjacencyList_[id];  // 默认构造空向量
    if (id >= nextNodeId_) {
        nextNodeId_ = id + 1;
    }
    return id;
}

const Node* Graph::getNode(Node::Id id) const {
    auto it = nodes_.find(id);
    return (it != nodes_.end()) ? &it->second : nullptr;
}

Node* Graph::getNode(Node::Id id) {
    auto it = nodes_.find(id);
    return (it != nodes_.end()) ? &it->second : nullptr;
}

Edge::Id Graph::addEdge(Node::Id source, Node::Id target) {
    return addEdge(source, target, RoadClass::Local);
}

Edge::Id Graph::addEdge(Node::Id source, Node::Id target, RoadClass roadClass) {
    const Node* s = getNode(source);
    const Node* t = getNode(target);
    if (!s || !t) return Edge::INVALID_ID;
    double length = s->getPosition().distanceTo(t->getPosition());
    return addEdgeWithId(nextEdgeId_++, source, target, length, 10.0, roadClass);
}

Edge::Id Graph::addEdgeWithId(Edge::Id id, Node::Id source, Node::Id target,
                               double length, double capacity,
                               RoadClass roadClass) {
    if (nodes_.find(source) == nodes_.end() || nodes_.find(target) == nodes_.end()) {
        return Edge::INVALID_ID;
    }
    Edge edge(id, source, target, length);
    edge.setCapacity(capacity);
    edge.setRoadClass(roadClass);
    edge.setImportanceScore(defaultImportanceForRoadClass(roadClass));
    edge.setDisplayTier(defaultDisplayTierForRoadClass(roadClass));
    edges_.emplace(id, edge);
    adjacencyList_[source].push_back(id);
    adjacencyList_[target].push_back(id);
    if (id >= nextEdgeId_) {
        nextEdgeId_ = id + 1;
    }
    return id;
}

const Edge* Graph::getEdge(Edge::Id id) const {
    auto it = edges_.find(id);
    return (it != edges_.end()) ? &it->second : nullptr;
}

Edge* Graph::getEdge(Edge::Id id) {
    auto it = edges_.find(id);
    return (it != edges_.end()) ? &it->second : nullptr;
}

const std::vector<Edge::Id>& Graph::getAdjacentEdges(Node::Id nodeId) const {
    auto it = adjacencyList_.find(nodeId);
    return (it != adjacencyList_.end()) ? it->second : emptyEdgeList_;
}

std::vector<Node::Id> Graph::getNeighbors(Node::Id nodeId) const {
    std::vector<Node::Id> neighbors;
    const auto& adjacentEdges = getAdjacentEdges(nodeId);

    for (Edge::Id edgeId : adjacentEdges) {
        const Edge* edge = getEdge(edgeId);
        if (edge) {
            // 添加边的另一个端点
            Node::Id neighbor = edge->getTraversableTarget(nodeId);
            if (neighbor != Node::INVALID_ID) {
                neighbors.push_back(neighbor);
            }
        }
    }

    return neighbors;
}

bool Graph::isConnected() const {
    if (nodes_.empty()) {
        return true;
    }

    // 从第一个节点执行 DFS
    std::unordered_map<Node::Id, bool> visited;
    for (const auto& pair : nodes_) {
        visited[pair.first] = false;
    }

    // 从第一个节点开始 DFS
    Node::Id startNode = nodes_.begin()->first;
    dfsVisit(startNode, visited);

    // 检查是否所有节点都被访问
    for (const auto& pair : visited) {
        if (!pair.second) {
            return false;
        }
    }

    return true;
}

void Graph::dfsVisit(Node::Id nodeId, std::unordered_map<Node::Id, bool>& visited) const {
    visited[nodeId] = true;

    const auto& adjacentEdges = getAdjacentEdges(nodeId);
    for (Edge::Id edgeId : adjacentEdges) {
        const Edge* edge = getEdge(edgeId);
        if (edge) {
            Node::Id neighbor = edge->getTraversableTarget(nodeId);
            if (neighbor != Node::INVALID_ID && !visited[neighbor]) {
                dfsVisit(neighbor, visited);
            }
        }
    }
}

void Graph::clear() {
    nodes_.clear();
    edges_.clear();
    adjacencyList_.clear();
    nextNodeId_ = 0;
    nextEdgeId_ = 0;
}

void Graph::reserve(size_t nodeCount, size_t edgeCount) {
    nodes_.reserve(nodeCount);
    edges_.reserve(edgeCount);
    adjacencyList_.reserve(nodeCount);
}

} // namespace nav
