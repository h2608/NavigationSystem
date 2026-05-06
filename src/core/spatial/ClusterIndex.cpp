#include "core/spatial/ClusterIndex.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_map>
#include <vector>

namespace nav {

namespace {

struct NodeSample {
    Node::Id id = Node::INVALID_ID;
    Point2D position;
    double score = 1.0;
    int densityCol = 0;
    int densityRow = 0;
};

struct AssignedCluster {
    double weightedX = 0.0;
    double weightedY = 0.0;
    double totalWeight = 0.0;
    double bestScore = -std::numeric_limits<double>::infinity();
    Point2D anchor;
    Node::Id anchorId = Node::INVALID_ID;
    size_t count = 0;
};

double clampDouble(double value, double minValue, double maxValue) {
    return std::max(minValue, std::min(value, maxValue));
}

int clampInt(int value, int minValue, int maxValue) {
    return std::max(minValue, std::min(value, maxValue));
}

double squaredDistance(const Point2D& a, const Point2D& b) {
    const double dx = a.x - b.x;
    const double dy = a.y - b.y;
    return dx * dx + dy * dy;
}

double mapArea(double width, double height) {
    return std::max(1.0, width) * std::max(1.0, height);
}

uint64_t densityKey(int row, int col, int numCols) {
    return static_cast<uint64_t>(row) * static_cast<uint64_t>(numCols) +
           static_cast<uint64_t>(col);
}

std::vector<NodeSample> collectNodeSamples(const Graph& graph,
                                           double mapWidth,
                                           double mapHeight) {
    std::vector<NodeSample> samples;
    samples.reserve(graph.getNodeCount());
    if (graph.getNodeCount() == 0) {
        return samples;
    }

    const double typicalSpacing = std::sqrt(mapArea(mapWidth, mapHeight) /
                                            static_cast<double>(graph.getNodeCount()));
    const double densityCellSize = std::max(1.0, typicalSpacing * 3.0);
    const int numCols = std::max(1, static_cast<int>(std::ceil(mapWidth / densityCellSize)));
    const int numRows = std::max(1, static_cast<int>(std::ceil(mapHeight / densityCellSize)));

    std::unordered_map<uint64_t, size_t> densityCells;
    densityCells.reserve(graph.getNodeCount());

    for (const auto& pair : graph.getNodes()) {
        const Node& node = pair.second;
        const Point2D& pos = node.getPosition();
        NodeSample sample;
        sample.id = node.getId();
        sample.position = pos;
        sample.densityCol = clampInt(static_cast<int>(pos.x / densityCellSize), 0, numCols - 1);
        sample.densityRow = clampInt(static_cast<int>(pos.y / densityCellSize), 0, numRows - 1);

        const auto& adjacentEdges = graph.getAdjacentEdges(node.getId());
        double importanceSum = 0.0;
        double maxImportance = 0.0;
        for (Edge::Id edgeId : adjacentEdges) {
            if (const Edge* edge = graph.getEdge(edgeId)) {
                importanceSum += edge->getImportanceScore();
                maxImportance = std::max(maxImportance, edge->getImportanceScore());
            }
        }

        const double degree = static_cast<double>(adjacentEdges.size());
        const double avgImportance = degree > 0.0 ? importanceSum / degree : 0.0;
        sample.score = 1.0 + degree * 0.16 + avgImportance * 1.25 + maxImportance * 1.15;

        densityCells[densityKey(sample.densityRow, sample.densityCol, numCols)]++;
        samples.push_back(sample);
    }

    for (NodeSample& sample : samples) {
        size_t localDensity = 0;
        for (int row = sample.densityRow - 1; row <= sample.densityRow + 1; ++row) {
            if (row < 0 || row >= numRows) continue;
            for (int col = sample.densityCol - 1; col <= sample.densityCol + 1; ++col) {
                if (col < 0 || col >= numCols) continue;
                auto it = densityCells.find(densityKey(row, col, numCols));
                if (it != densityCells.end()) {
                    localDensity += it->second;
                }
            }
        }
        sample.score += std::log1p(static_cast<double>(localDensity)) * 0.35;
    }

    std::sort(samples.begin(), samples.end(), [](const NodeSample& a, const NodeSample& b) {
        return a.id < b.id;
    });
    return samples;
}

std::vector<size_t> selectCenters(const std::vector<NodeSample>& samples,
                                  double mapWidth,
                                  double mapHeight,
                                  size_t targetCount) {
    const size_t clampedTarget = std::min(targetCount, samples.size());
    std::vector<size_t> centers;
    centers.reserve(clampedTarget);
    if (samples.empty() || clampedTarget == 0) {
        return centers;
    }

    std::vector<size_t> order(samples.size());
    for (size_t i = 0; i < samples.size(); ++i) {
        order[i] = i;
    }
    std::sort(order.begin(), order.end(), [&samples](size_t a, size_t b) {
        if (samples[a].score != samples[b].score) {
            return samples[a].score > samples[b].score;
        }
        return samples[a].id < samples[b].id;
    });

    std::vector<bool> selected(samples.size(), false);
    const double baseSpacing = std::sqrt(mapArea(mapWidth, mapHeight) /
                                         static_cast<double>(clampedTarget));

    for (int pass = 0; pass < 4 && centers.size() < clampedTarget; ++pass) {
        const double spacing = baseSpacing * 0.68 * std::pow(0.72, pass);
        const double minDistanceSq = spacing * spacing;
        for (size_t sampleIndex : order) {
            if (selected[sampleIndex]) continue;

            bool farEnough = true;
            for (size_t centerIndex : centers) {
                if (squaredDistance(samples[sampleIndex].position,
                                    samples[centerIndex].position) < minDistanceSq) {
                    farEnough = false;
                    break;
                }
            }

            if (!farEnough) continue;
            centers.push_back(sampleIndex);
            selected[sampleIndex] = true;
            if (centers.size() >= clampedTarget) {
                break;
            }
        }
    }

    while (centers.size() < clampedTarget) {
        size_t bestIndex = samples.size();
        double bestValue = -1.0;
        for (size_t i = 0; i < samples.size(); ++i) {
            if (selected[i]) continue;

            double nearestDistanceSq = std::numeric_limits<double>::infinity();
            for (size_t centerIndex : centers) {
                nearestDistanceSq = std::min(nearestDistanceSq,
                                             squaredDistance(samples[i].position,
                                                             samples[centerIndex].position));
            }

            const double coverage = centers.empty() ? 1.0 : std::sqrt(nearestDistanceSq);
            const double value = coverage * (0.75 + samples[i].score * 0.25);
            if (value > bestValue ||
                (value == bestValue && bestIndex < samples.size() && samples[i].id < samples[bestIndex].id)) {
                bestValue = value;
                bestIndex = i;
            }
        }

        if (bestIndex >= samples.size()) {
            break;
        }
        centers.push_back(bestIndex);
        selected[bestIndex] = true;
    }

    return centers;
}

} // namespace

void ClusterIndex::buildSingleLevel(const Graph& graph, double mapWidth,
                                     double gridCellSize, std::vector<Cluster>& out) {
    out.clear();
    if (gridCellSize <= 0.0 || graph.getNodeCount() == 0) return;

    int numCols = std::max(1, static_cast<int>(std::ceil(mapWidth / gridCellSize)));

    struct CellData {
        double sumX = 0.0;
        double sumY = 0.0;
        size_t count = 0;
    };
    std::unordered_map<uint64_t, CellData> cells;

    for (const auto& pair : graph.getNodes()) {
        const Node& node = pair.second;
        const Point2D& pos = node.getPosition();

        int col = static_cast<int>(pos.x / gridCellSize);
        int row = static_cast<int>(pos.y / gridCellSize);
        uint64_t key = static_cast<uint64_t>(row) * numCols + col;

        auto& cell = cells[key];
        cell.sumX += pos.x;
        cell.sumY += pos.y;
        cell.count++;
    }

    out.reserve(cells.size());
    for (const auto& pair : cells) {
        const CellData& cell = pair.second;
        if (cell.count > 0) {
            Cluster cluster;
            cluster.position = Point2D(cell.sumX / cell.count,
                                        cell.sumY / cell.count);
            cluster.memberCount = cell.count;
            out.push_back(cluster);
        }
    }
}

void ClusterIndex::buildAdaptiveLevel(const Graph& graph, double mapWidth,
                                      double mapHeight, size_t targetCount,
                                      std::vector<Cluster>& out) {
    out.clear();

    const std::vector<NodeSample> samples = collectNodeSamples(graph, mapWidth, mapHeight);
    const std::vector<size_t> centers = selectCenters(samples, mapWidth, mapHeight, targetCount);
    if (samples.empty() || centers.empty()) {
        return;
    }

    std::vector<AssignedCluster> assigned(centers.size());
    for (const NodeSample& sample : samples) {
        size_t nearestCenter = 0;
        double nearestDistanceSq = std::numeric_limits<double>::infinity();
        for (size_t i = 0; i < centers.size(); ++i) {
            const double distanceSq = squaredDistance(sample.position,
                                                      samples[centers[i]].position);
            if (distanceSq < nearestDistanceSq) {
                nearestDistanceSq = distanceSq;
                nearestCenter = i;
            }
        }

        const double weight = 1.0 + sample.score;
        AssignedCluster& cluster = assigned[nearestCenter];
        cluster.weightedX += sample.position.x * weight;
        cluster.weightedY += sample.position.y * weight;
        cluster.totalWeight += weight;
        cluster.count++;
        if (sample.score > cluster.bestScore ||
            (sample.score == cluster.bestScore && sample.id < cluster.anchorId)) {
            cluster.bestScore = sample.score;
            cluster.anchor = sample.position;
            cluster.anchorId = sample.id;
        }
    }

    out.reserve(assigned.size());
    const double anchorBlend = centers.size() <= 70 ? 0.35 : 0.24;
    for (const AssignedCluster& assignedCluster : assigned) {
        if (assignedCluster.count == 0 || assignedCluster.totalWeight <= 0.0) {
            continue;
        }

        const double centroidX = assignedCluster.weightedX / assignedCluster.totalWeight;
        const double centroidY = assignedCluster.weightedY / assignedCluster.totalWeight;
        const double x = centroidX * (1.0 - anchorBlend) + assignedCluster.anchor.x * anchorBlend;
        const double y = centroidY * (1.0 - anchorBlend) + assignedCluster.anchor.y * anchorBlend;

        Cluster cluster;
        cluster.position = Point2D(clampDouble(x, 0.0, mapWidth),
                                   clampDouble(y, 0.0, mapHeight));
        cluster.memberCount = assignedCluster.count;
        out.push_back(cluster);
    }

    std::sort(out.begin(), out.end(), [](const Cluster& a, const Cluster& b) {
        if (a.memberCount != b.memberCount) {
            return a.memberCount > b.memberCount;
        }
        if (a.position.y != b.position.y) {
            return a.position.y < b.position.y;
        }
        return a.position.x < b.position.x;
    });
}

void ClusterIndex::build(const Graph& graph, double mapWidth,
                          double mapHeight, double gridCellSize) {
    gridCellSize_ = gridCellSize;
    buildSingleLevel(graph, mapWidth, gridCellSize, clusters_);
}

void ClusterIndex::buildMultiResolution(const Graph& graph, double mapWidth,
                                         double mapHeight,
                                         const std::vector<double>& cellSizes) {
    levels_.clear();
    levels_.reserve(cellSizes.size());

    for (double cellSize : cellSizes) {
        ClusterLevel level;
        level.cellSize = cellSize;
        buildSingleLevel(graph, mapWidth, cellSize, level.clusters);
        levels_.push_back(std::move(level));
    }
}

void ClusterIndex::buildAdaptiveMultiResolution(const Graph& graph, double mapWidth,
                                                double mapHeight,
                                                const std::vector<size_t>& targetCounts) {
    levels_.clear();
    levels_.reserve(targetCounts.size());

    for (size_t targetCount : targetCounts) {
        ClusterLevel level;
        level.cellSize = 0.0;
        buildAdaptiveLevel(graph, mapWidth, mapHeight, targetCount, level.clusters);
        levels_.push_back(std::move(level));
    }
}

} // namespace nav
