#include "core/algorithms/DynamicPathFinder.h"
#include "core/traffic/TrafficModel.h"
#include <queue>
#include <limits>
#include <algorithm>

namespace nav {

PathResult DynamicPathFinder::findPath(const Graph& graph, Node::Id start, Node::Id end) {
    PathResult result;

    // 验证起点和终点节点是否存在
    if (graph.getNode(start) == nullptr || graph.getNode(end) == nullptr) {
        result.found = false;
        return result;
    }

    // 如果起点 == 终点，返回平凡路径
    if (start == end) {
        result.pathNodes.push_back(start);
        result.totalCost = 0.0;
        result.found = true;
        return result;
    }

    // 距离映射（延迟初始化 - 无需将所有设置为无穷大）
    std::unordered_map<Node::Id, double> dist;
    dist.reserve(graph.getNodeCount());
    dist[start] = 0.0;

    // 前驱映射：存储哪条边导致到达此节点
    std::unordered_map<Node::Id, Edge::Id> predecessor;

    // 优先队列：(距离, 节点ID)
    using PQEntry = std::pair<double, Node::Id>;
    std::priority_queue<PQEntry, std::vector<PQEntry>, std::greater<PQEntry>> pq;

    pq.push({0.0, start});

    // 访问集合以避免重复处理
    std::unordered_map<Node::Id, bool> visited;

    while (!pq.empty()) {
        auto [currentDist, currentNode] = pq.top();
        pq.pop();

        // 如果已访问则跳过
        if (visited[currentNode]) {
            continue;
        }
        visited[currentNode] = true;

        // 提前终止：如果到达终点节点，则完成
        if (currentNode == end) {
            break;
        }

        // 探索邻居
        const auto& adjacentEdges = graph.getAdjacentEdges(currentNode);
        for (Edge::Id edgeId : adjacentEdges) {
            const Edge* edge = graph.getEdge(edgeId);
            if (!edge) continue;

            // 确定邻居节点（边的另一个端点）
            Node::Id neighbor = edge->getTraversableTarget(currentNode);
            if (neighbor == Node::INVALID_ID) {
                continue;
            }

            // 如果已访问则跳过
            if (visited[neighbor]) {
                continue;
            }

            // 使用交通模型计算行驶时间（考虑道路等级速度因子）
            double travelTime = TrafficModel::calculateTravelTime(
                edge->getLength(),
                edge->getCapacity(),
                static_cast<double>(edge->getCarCount())
            ) / roadClassSpeedFactor(edge->getRoadClass());

            double newDist = dist[currentNode] + travelTime;

            // 如果这是更好的路径，则更新（缺失条目 = 无穷大）
            auto it = dist.find(neighbor);
            double currentBest = (it != dist.end()) ? it->second : std::numeric_limits<double>::infinity();
            if (newDist < currentBest) {
                dist[neighbor] = newDist;
                predecessor[neighbor] = edgeId;
                pq.push({newDist, neighbor});
            }
        }
    }

    // 检查是否找到路径
    auto endIt = dist.find(end);
    if (endIt == dist.end() || endIt->second == std::numeric_limits<double>::infinity()) {
        result.found = false;
        return result;
    }

    // 重建路径
    result.found = true;
    result.totalCost = endIt->second;
    reconstructPath(graph, start, end, predecessor, result);

    return result;
}

void DynamicPathFinder::reconstructPath(const Graph& graph,
                                        Node::Id start,
                                        Node::Id end,
                                        const std::unordered_map<Node::Id, Edge::Id>& predecessor,
                                        PathResult& result) const {
    // 通过向后跟随前驱边重建路径
    std::vector<Node::Id> reversePath;
    std::vector<Edge::Id> reverseEdges;

    Node::Id current = end;
    reversePath.push_back(current);

    while (current != start) {
        auto it = predecessor.find(current);
        if (it == predecessor.end()) {
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

        // 移动到边的另一个端点
        current = edge->getOppositeNode(current);
        if (current == Node::INVALID_ID) {
            result.found = false;
            result.pathNodes.clear();
            result.pathEdges.clear();
            return;
        }
        reversePath.push_back(current);
    }

    // 反转以获得从起点到终点的路径
    result.pathNodes.assign(reversePath.rbegin(), reversePath.rend());
    result.pathEdges.assign(reverseEdges.rbegin(), reverseEdges.rend());
}

} // namespace nav
