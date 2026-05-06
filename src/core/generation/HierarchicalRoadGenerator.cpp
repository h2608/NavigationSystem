#include "core/generation/HierarchicalRoadGenerator.h"
#include <cmath>
#include <iostream>
#include <algorithm>
#include <unordered_map>
#include <unordered_set>

namespace nav {

// ============================================================================
// 用于连通性检查的并查集
// ============================================================================
namespace {

class UnionFind {
public:
    explicit UnionFind(size_t n) : parent_(n), rank_(n, 0) {
        for (size_t i = 0; i < n; ++i) {
            parent_[i] = i;
        }
    }

    size_t find(size_t x) {
        if (parent_[x] != x) {
            parent_[x] = find(parent_[x]);  // 路径压缩
        }
        return parent_[x];
    }

    bool unite(size_t x, size_t y) {
        size_t rootX = find(x);
        size_t rootY = find(y);
        if (rootX == rootY) {
            return false;
        }
        if (rank_[rootX] < rank_[rootY]) {
            parent_[rootX] = rootY;
        } else if (rank_[rootX] > rank_[rootY]) {
            parent_[rootY] = rootX;
        } else {
            parent_[rootY] = rootX;
            rank_[rootX]++;
        }
        return true;
    }

private:
    std::vector<size_t> parent_;
    std::vector<size_t> rank_;
};

// 用于 MST 计算的临时边结构
struct TempEdge {
    Node::Id source;
    Node::Id target;
    double weight;
};

double clamp01(double value) {
    return std::max(0.0, std::min(1.0, value));
}

void configureGeneratedEdge(Edge* edge, RoadClass roadClass, double capacity) {
    if (!edge) return;

    edge->setRoadClass(roadClass);
    edge->setCapacity(capacity);

    double minCapacity = 4.0;
    double maxCapacity = 6.0;
    double baseImportance = 0.18;
    double range = 0.22;

    switch (roadClass) {
        case RoadClass::Arterial:
            minCapacity = 18.0;
            maxCapacity = 22.0;
            baseImportance = 0.82;
            range = 0.18;
            break;
        case RoadClass::Secondary:
            minCapacity = 10.0;
            maxCapacity = 14.0;
            baseImportance = 0.52;
            range = 0.22;
            break;
        case RoadClass::Local:
        default:
            break;
    }

    const double normalized = clamp01((capacity - minCapacity) / (maxCapacity - minCapacity));
    const double importance = clamp01(baseImportance + normalized * range);

    edge->setImportanceScore(importance);
    edge->setDisplayTier(displayTierForImportance(importance));
}

} // anonymous namespace

HierarchicalRoadGenerator::HierarchicalRoadGenerator()
    : rng_(std::random_device{}())
{}

HierarchicalRoadGenerator::HierarchicalRoadGenerator(unsigned int seed)
    : rng_(seed)
{}

void HierarchicalRoadGenerator::generate(Graph& graph, int numNodes,
                                          double width, double height) {
    graph.clear();
    if (numNodes <= 0) return;

    // 预分配
    graph.reserve(static_cast<size_t>(numNodes),
                  static_cast<size_t>(numNodes * 3));

    std::vector<Node::Id> arterialNodes;
    std::vector<Node::Id> secondaryNodes;

    // 按比例分配节点预算
    int arterialTarget = std::max(6, static_cast<int>(numNodes * 0.05));
    int secondaryTarget = std::max(10, static_cast<int>(numNodes * 0.15));

    // 阶段 A：生成主干道
    generateArterials(graph, width, height, arterialTarget, arterialNodes);

    // 阶段 B：生成次干道
    generateSecondaries(graph, width, height, secondaryTarget, arterialNodes, secondaryNodes);

    // 阶段 C：生成支路（剩余节点）
    int usedNodes = static_cast<int>(arterialNodes.size() + secondaryNodes.size());
    int remainingNodes = std::max(0, numNodes - usedNodes);
    generateLocals(graph, remainingNodes, width, height,
                   arterialNodes, secondaryNodes);

    // 确保全图连通
    ensureConnectivity(graph);

    std::cout << "Hierarchical generation complete: "
              << graph.getNodeCount() << " nodes, "
              << graph.getEdgeCount() << " edges" << std::endl;
}

// ============================================================================
// 阶段 A：主干道 — 水平和垂直走廊
// ============================================================================
void HierarchicalRoadGenerator::generateArterials(Graph& graph,
    double width, double height, int targetNodes,
    std::vector<Node::Id>& arterialNodes) {

    // 根据目标节点数计算走廊布局
    int nodesPerCorridor = std::max(3, static_cast<int>(std::sqrt(static_cast<double>(targetNodes))));
    int totalCorridors = std::max(2, targetNodes / nodesPerCorridor);
    int numHorizontal = std::max(1, totalCorridors / 2);
    int numVertical = std::max(1, totalCorridors - numHorizontal);

    double nodeSpacingH = width / nodesPerCorridor;
    double nodeSpacingV = height / nodesPerCorridor;

    std::uniform_real_distribution<double> jitter(-nodeSpacingH * 0.03, nodeSpacingH * 0.03);
    std::uniform_real_distribution<double> capacityNoise(-2.0, 2.0);

    std::vector<Node::Id> allArterialNodes;

    // 水平主干道走廊
    std::vector<std::vector<Node::Id>> hCorridors;
    for (int i = 0; i < numHorizontal; ++i) {
        double y = (i + 1) * height / (numHorizontal + 1);
        std::vector<Node::Id> corridor;

        for (int j = 0; j < nodesPerCorridor; ++j) {
            double x = (j + 0.5) * width / nodesPerCorridor + jitter(rng_);
            x = std::max(0.0, std::min(x, width));
            double yj = y + jitter(rng_);
            yj = std::max(0.0, std::min(yj, height));

            Node::Id nid = graph.addNode(Point2D(x, yj));
            corridor.push_back(nid);
            allArterialNodes.push_back(nid);
        }

        // 连接走廊中相邻节点
        for (size_t k = 1; k < corridor.size(); ++k) {
            Edge::Id eid = graph.addEdge(corridor[k-1], corridor[k], RoadClass::Arterial);
            Edge* e = graph.getEdge(eid);
            configureGeneratedEdge(e, RoadClass::Arterial, 20.0 + capacityNoise(rng_));
        }

        hCorridors.push_back(std::move(corridor));
    }

    // 垂直主干道走廊
    std::vector<std::vector<Node::Id>> vCorridors;
    for (int i = 0; i < numVertical; ++i) {
        double x = (i + 1) * width / (numVertical + 1);
        std::vector<Node::Id> corridor;

        for (int j = 0; j < nodesPerCorridor; ++j) {
            double y = (j + 0.5) * height / nodesPerCorridor + jitter(rng_);
            y = std::max(0.0, std::min(y, height));
            double xj = x + jitter(rng_);
            xj = std::max(0.0, std::min(xj, width));

            // 检查是否接近已有的水平走廊节点，如果是则复用
            Node::Id nearest = findNearestNode(graph, Point2D(xj, y), allArterialNodes);
            double mergeThreshold = nodeSpacingV * 0.5;
            if (nearest != Node::INVALID_ID) {
                const Node* nn = graph.getNode(nearest);
                if (nn && nn->getPosition().distanceTo(Point2D(xj, y)) < mergeThreshold) {
                    corridor.push_back(nearest);
                    continue;
                }
            }

            Node::Id nid = graph.addNode(Point2D(xj, y));
            corridor.push_back(nid);
            allArterialNodes.push_back(nid);
        }

        // 连接走廊中相邻节点
        for (size_t k = 1; k < corridor.size(); ++k) {
            Edge::Id eid = graph.addEdge(corridor[k-1], corridor[k], RoadClass::Arterial);
            Edge* e = graph.getEdge(eid);
            configureGeneratedEdge(e, RoadClass::Arterial, 20.0 + capacityNoise(rng_));
        }

        vCorridors.push_back(std::move(corridor));
    }

    // 在水平与垂直走廊交叉处连接
    for (const auto& hCorridor : hCorridors) {
        for (const auto& vCorridor : vCorridors) {
            // 找到两条走廊中最近的一对节点
            double bestDist = std::numeric_limits<double>::max();
            Node::Id bestH = Node::INVALID_ID, bestV = Node::INVALID_ID;
            for (Node::Id hNode : hCorridor) {
                for (Node::Id vNode : vCorridor) {
                    const Node* hn = graph.getNode(hNode);
                    const Node* vn = graph.getNode(vNode);
                    if (!hn || !vn) continue;
                    double dist = hn->getPosition().distanceTo(vn->getPosition());
                    if (dist < bestDist) {
                        bestDist = dist;
                        bestH = hNode;
                        bestV = vNode;
                    }
                }
            }
            // 如果最近的一对足够近（但不是同一节点），添加连接
            if (bestH != bestV && bestH != Node::INVALID_ID &&
                bestDist < nodeSpacingH * 1.5) {
                Edge::Id eid = graph.addEdge(bestH, bestV, RoadClass::Arterial);
                Edge* e = graph.getEdge(eid);
                configureGeneratedEdge(e, RoadClass::Arterial, 20.0 + capacityNoise(rng_));
            }
        }
    }

    arterialNodes = std::move(allArterialNodes);

    std::cout << "Arterials: " << arterialNodes.size() << " nodes" << std::endl;
}

// ============================================================================
// 阶段 B：次干道 — 在主干道块内细分
// ============================================================================
void HierarchicalRoadGenerator::generateSecondaries(Graph& graph,
    double width, double height, int targetNodes,
    const std::vector<Node::Id>& arterialNodes,
    std::vector<Node::Id>& secondaryNodes) {

    // 根据目标节点数计算走廊布局
    int nodesPerCorridor = std::max(3, static_cast<int>(std::sqrt(static_cast<double>(targetNodes))));
    int totalCorridors = std::max(2, targetNodes / nodesPerCorridor);
    int numHorizontal = std::max(1, totalCorridors / 2);
    int numVertical = std::max(1, totalCorridors - numHorizontal);

    double nodeSpacingH = width / nodesPerCorridor;
    double nodeSpacingV = height / nodesPerCorridor;

    std::uniform_real_distribution<double> jitter(-nodeSpacingH * 0.05, nodeSpacingH * 0.05);
    std::uniform_real_distribution<double> capacityNoise(-2.0, 2.0);

    std::vector<Node::Id> connectTargets(arterialNodes);

    // 水平次干道
    for (int i = 0; i < numHorizontal; ++i) {
        double y = (i + 0.5) * height / numHorizontal + jitter(rng_);
        y = std::max(0.0, std::min(y, height));

        std::vector<Node::Id> corridor;
        for (int j = 0; j < nodesPerCorridor; ++j) {
            double x = (j + 0.5) * width / nodesPerCorridor + jitter(rng_);
            x = std::max(0.0, std::min(x, width));

            Node::Id nearest = findNearestNode(graph, Point2D(x, y), connectTargets);
            double mergeThreshold = nodeSpacingH * 0.4;
            if (nearest != Node::INVALID_ID) {
                const Node* nn = graph.getNode(nearest);
                if (nn && nn->getPosition().distanceTo(Point2D(x, y)) < mergeThreshold) {
                    corridor.push_back(nearest);
                    continue;
                }
            }

            Node::Id nid = graph.addNode(Point2D(x, y));
            corridor.push_back(nid);
            secondaryNodes.push_back(nid);
            connectTargets.push_back(nid);
        }

        // 连接走廊
        for (size_t k = 1; k < corridor.size(); ++k) {
            if (corridor[k-1] != corridor[k]) {
                Edge::Id eid = graph.addEdge(corridor[k-1], corridor[k], RoadClass::Secondary);
                Edge* e = graph.getEdge(eid);
                configureGeneratedEdge(e, RoadClass::Secondary, 12.0 + capacityNoise(rng_));
            }
        }
    }

    // 垂直次干道
    for (int i = 0; i < numVertical; ++i) {
        double x = (i + 0.5) * width / numVertical + jitter(rng_);
        x = std::max(0.0, std::min(x, width));

        std::vector<Node::Id> corridor;
        for (int j = 0; j < nodesPerCorridor; ++j) {
            double y = (j + 0.5) * height / nodesPerCorridor + jitter(rng_);
            y = std::max(0.0, std::min(y, height));

            Node::Id nearest = findNearestNode(graph, Point2D(x, y), connectTargets);
            double mergeThreshold = nodeSpacingV * 0.4;
            if (nearest != Node::INVALID_ID) {
                const Node* nn = graph.getNode(nearest);
                if (nn && nn->getPosition().distanceTo(Point2D(x, y)) < mergeThreshold) {
                    corridor.push_back(nearest);
                    continue;
                }
            }

            Node::Id nid = graph.addNode(Point2D(x, y));
            corridor.push_back(nid);
            secondaryNodes.push_back(nid);
            connectTargets.push_back(nid);
        }

        // 连接走廊
        for (size_t k = 1; k < corridor.size(); ++k) {
            if (corridor[k-1] != corridor[k]) {
                Edge::Id eid = graph.addEdge(corridor[k-1], corridor[k], RoadClass::Secondary);
                Edge* e = graph.getEdge(eid);
                configureGeneratedEdge(e, RoadClass::Secondary, 12.0 + capacityNoise(rng_));
            }
        }
    }

    std::cout << "Secondaries: " << secondaryNodes.size() << " nodes" << std::endl;
}

// ============================================================================
// 阶段 C：支路 — 扰动网格 + Kruskal MST 填充
// ============================================================================
void HierarchicalRoadGenerator::generateLocals(Graph& graph,
    int remainingNodes, double width, double height,
    const std::vector<Node::Id>& arterialNodes,
    const std::vector<Node::Id>& secondaryNodes) {

    if (remainingNodes <= 0) return;

    // 计算支路网格维度
    int cols = static_cast<int>(std::ceil(std::sqrt(
        remainingNodes * width / height)));
    int rows = static_cast<int>(std::ceil(
        static_cast<double>(remainingNodes) / cols));

    while (rows * cols > remainingNodes * 2) {
        if (cols > rows) cols--;
        else rows--;
    }

    double cellWidth = width / cols;
    double cellHeight = height / rows;

    std::uniform_real_distribution<double> perturbDist(-0.35, 0.35);
    std::uniform_real_distribution<double> weightDist(0.0, 1.0);
    std::uniform_real_distribution<double> capacityNoise(-1.0, 1.0);

    // 存储节点网格
    std::vector<std::vector<Node::Id>> nodeGrid(
        rows, std::vector<Node::Id>(cols, Node::INVALID_ID));

    // 合并所有高级别节点用于连接
    std::vector<Node::Id> higherNodes;
    higherNodes.reserve(arterialNodes.size() + secondaryNodes.size());
    higherNodes.insert(higherNodes.end(), arterialNodes.begin(), arterialNodes.end());
    higherNodes.insert(higherNodes.end(), secondaryNodes.begin(), secondaryNodes.end());

    // 生成支路节点
    std::vector<Node::Id> localNodes;
    int nodesCreated = 0;
    for (int i = 0; i < rows && nodesCreated < remainingNodes; ++i) {
        for (int j = 0; j < cols && nodesCreated < remainingNodes; ++j) {
            double baseX = (j + 0.5) * cellWidth;
            double baseY = (i + 0.5) * cellHeight;

            double offsetX = perturbDist(rng_) * cellWidth;
            double offsetY = perturbDist(rng_) * cellHeight;

            double x = std::max(j * cellWidth + cellWidth * 0.05,
                std::min(baseX + offsetX, (j + 1) * cellWidth - cellWidth * 0.05));
            double y = std::max(i * cellHeight + cellHeight * 0.05,
                std::min(baseY + offsetY, (i + 1) * cellHeight - cellHeight * 0.05));

            // 检查是否太靠近高级别节点
            Node::Id nearest = findNearestNode(graph, Point2D(x, y), higherNodes);
            double minDist = std::min(cellWidth, cellHeight) * 0.3;
            if (nearest != Node::INVALID_ID) {
                const Node* nn = graph.getNode(nearest);
                if (nn && nn->getPosition().distanceTo(Point2D(x, y)) < minDist) {
                    // 复用高级别节点
                    nodeGrid[i][j] = nearest;
                    nodesCreated++;
                    continue;
                }
            }

            Node::Id nid = graph.addNode(Point2D(x, y));
            nodeGrid[i][j] = nid;
            localNodes.push_back(nid);
            nodesCreated++;
        }
    }

    // 收集所有潜在边
    std::vector<TempEdge> allEdges;
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            Node::Id curr = nodeGrid[i][j];
            if (curr == Node::INVALID_ID) continue;

            // 右邻居
            if (j + 1 < cols && nodeGrid[i][j+1] != Node::INVALID_ID &&
                nodeGrid[i][j+1] != curr) {
                allEdges.push_back({curr, nodeGrid[i][j+1], weightDist(rng_)});
            }
            // 下邻居
            if (i + 1 < rows && nodeGrid[i+1][j] != Node::INVALID_ID &&
                nodeGrid[i+1][j] != curr) {
                allEdges.push_back({curr, nodeGrid[i+1][j], weightDist(rng_)});
            }
            // 对角线（棋盘模式）
            if ((i + j) % 2 == 0) {
                if (i + 1 < rows && j + 1 < cols &&
                    nodeGrid[i+1][j+1] != Node::INVALID_ID &&
                    nodeGrid[i+1][j+1] != curr) {
                    if (weightDist(rng_) < 0.4) {
                        allEdges.push_back({curr, nodeGrid[i+1][j+1], weightDist(rng_)});
                    }
                }
                if (i + 1 < rows && j - 1 >= 0 &&
                    nodeGrid[i+1][j-1] != Node::INVALID_ID &&
                    nodeGrid[i+1][j-1] != curr) {
                    if (weightDist(rng_) < 0.4) {
                        allEdges.push_back({curr, nodeGrid[i+1][j-1], weightDist(rng_)});
                    }
                }
            }
        }
    }

    // Kruskal MST + 保留 60% 非 MST 边
    std::sort(allEdges.begin(), allEdges.end(),
        [](const TempEdge& a, const TempEdge& b) { return a.weight < b.weight; });

    // 构建 ID 映射用于 UnionFind（支路使用图中的实际节点 ID）
    std::unordered_map<Node::Id, size_t> idToIndex;
    std::vector<Node::Id> indexToId;
    for (const auto& pair : graph.getNodes()) {
        size_t idx = idToIndex.size();
        idToIndex[pair.first] = idx;
        indexToId.push_back(pair.first);
    }

    UnionFind uf(idToIndex.size());
    std::vector<TempEdge> mstEdges, nonMstEdges;

    for (const auto& edge : allEdges) {
        auto itS = idToIndex.find(edge.source);
        auto itT = idToIndex.find(edge.target);
        if (itS == idToIndex.end() || itT == idToIndex.end()) continue;

        if (uf.unite(itS->second, itT->second)) {
            mstEdges.push_back(edge);
        } else {
            nonMstEdges.push_back(edge);
        }
    }

    // 保留 60% 非 MST 边
    size_t nonMstToKeep = static_cast<size_t>(nonMstEdges.size() * 0.6);
    std::shuffle(nonMstEdges.begin(), nonMstEdges.end(), rng_);
    nonMstEdges.resize(nonMstToKeep);

    // 添加边到图
    for (const auto& edge : mstEdges) {
        Edge::Id eid = graph.addEdge(edge.source, edge.target, RoadClass::Local);
        Edge* e = graph.getEdge(eid);
        configureGeneratedEdge(e, RoadClass::Local, 5.0 + capacityNoise(rng_));
    }
    for (const auto& edge : nonMstEdges) {
        Edge::Id eid = graph.addEdge(edge.source, edge.target, RoadClass::Local);
        Edge* e = graph.getEdge(eid);
        configureGeneratedEdge(e, RoadClass::Local, 5.0 + capacityNoise(rng_));
    }

    // 将部分支路节点连接到最近的高级别节点
    std::uniform_real_distribution<double> connectChance(0.0, 1.0);
    for (Node::Id localNode : localNodes) {
        if (connectChance(rng_) > 0.3) continue;  // 30% 的支路节点连接到高级别
        Node::Id nearest = findNearestNode(graph, graph.getNode(localNode)->getPosition(),
                                            higherNodes);
        if (nearest != Node::INVALID_ID && nearest != localNode) {
            const Node* ln = graph.getNode(localNode);
            const Node* hn = graph.getNode(nearest);
            if (ln && hn) {
                double dist = ln->getPosition().distanceTo(hn->getPosition());
                if (dist < cellWidth * 2.0) {
                    Edge::Id eid = graph.addEdge(localNode, nearest, RoadClass::Local);
                    Edge* e = graph.getEdge(eid);
                    configureGeneratedEdge(e, RoadClass::Local, 5.0 + capacityNoise(rng_));
                }
            }
        }
    }

    std::cout << "Locals: " << localNodes.size() << " new nodes, "
              << nodesCreated << " total placed" << std::endl;
}

// ============================================================================
// 确保全图连通 — 桥接断开的分量
// ============================================================================
void HierarchicalRoadGenerator::ensureConnectivity(Graph& graph) {
    if (graph.getNodeCount() == 0) return;

    // 构建 ID 映射
    std::unordered_map<Node::Id, size_t> idToIndex;
    std::vector<Node::Id> indexToId;
    for (const auto& pair : graph.getNodes()) {
        size_t idx = idToIndex.size();
        idToIndex[pair.first] = idx;
        indexToId.push_back(pair.first);
    }

    UnionFind uf(idToIndex.size());

    // 将现有边加入并查集
    for (const auto& pair : graph.getEdges()) {
        const Edge& edge = pair.second;
        auto itS = idToIndex.find(edge.getSource());
        auto itT = idToIndex.find(edge.getTarget());
        if (itS != idToIndex.end() && itT != idToIndex.end()) {
            uf.unite(itS->second, itT->second);
        }
    }

    // 找到所有连通分量
    std::unordered_map<size_t, std::vector<size_t>> components;
    for (size_t i = 0; i < indexToId.size(); ++i) {
        components[uf.find(i)].push_back(i);
    }

    if (components.size() <= 1) {
        std::cout << "Graph is already connected." << std::endl;
        return;
    }

    std::cout << "Connecting " << components.size() << " components..." << std::endl;

    // 收集各分量的代表节点
    std::vector<size_t> componentRoots;
    for (const auto& comp : components) {
        componentRoots.push_back(comp.second[0]);
    }

    // 逐对连接分量（链式连接）
    std::uniform_real_distribution<double> capacityNoise(-2.0, 2.0);
    for (size_t i = 1; i < componentRoots.size(); ++i) {
        // 在两个分量中找到最近的一对节点
        const auto& compA = components[uf.find(componentRoots[i-1])];
        const auto& compB = components[uf.find(componentRoots[i])];

        double bestDist = std::numeric_limits<double>::max();
        Node::Id bestA = Node::INVALID_ID, bestB = Node::INVALID_ID;

        // 为效率考虑，只采样部分节点
        size_t sampleA = std::min(compA.size(), static_cast<size_t>(50));
        size_t sampleB = std::min(compB.size(), static_cast<size_t>(50));

        for (size_t a = 0; a < sampleA; ++a) {
            Node::Id nodeA = indexToId[compA[a]];
            const Node* nA = graph.getNode(nodeA);
            if (!nA) continue;

            for (size_t b = 0; b < sampleB; ++b) {
                Node::Id nodeB = indexToId[compB[b]];
                const Node* nB = graph.getNode(nodeB);
                if (!nB) continue;

                double dist = nA->getPosition().distanceTo(nB->getPosition());
                if (dist < bestDist) {
                    bestDist = dist;
                    bestA = nodeA;
                    bestB = nodeB;
                }
            }
        }

        if (bestA != Node::INVALID_ID && bestB != Node::INVALID_ID) {
            Edge::Id eid = graph.addEdge(bestA, bestB, RoadClass::Secondary);
            Edge* e = graph.getEdge(eid);
            configureGeneratedEdge(e, RoadClass::Secondary, 12.0 + capacityNoise(rng_));
            // 更新并查集
            uf.unite(idToIndex[bestA], idToIndex[bestB]);
        }
    }
}

// ============================================================================
// 辅助函数：在候选节点中找到距 pos 最近的节点
// ============================================================================
Node::Id HierarchicalRoadGenerator::findNearestNode(const Graph& graph,
    const Point2D& pos,
    const std::vector<Node::Id>& candidates) const {

    if (candidates.empty()) return Node::INVALID_ID;

    double bestDist = std::numeric_limits<double>::max();
    Node::Id bestId = Node::INVALID_ID;

    for (Node::Id id : candidates) {
        const Node* node = graph.getNode(id);
        if (!node) continue;
        double dist = node->getPosition().distanceTo(pos);
        if (dist < bestDist) {
            bestDist = dist;
            bestId = id;
        }
    }

    return bestId;
}

} // namespace nav
