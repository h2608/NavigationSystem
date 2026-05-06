#ifndef CLUSTERINDEX_H
#define CLUSTERINDEX_H

#include "core/graph/Graph.h"
#include <vector>

namespace nav {

// 多分辨率节点聚类索引
class ClusterIndex {
public:
    struct Cluster {
        Point2D position;          // 聚类中心（成员质心）
        size_t memberCount;        // 包含的原始节点数
    };

    struct ClusterLevel {
        double cellSize;
        std::vector<Cluster> clusters;
    };

    ClusterIndex() = default;

    // 从图构建单分辨率聚类索引
    void build(const Graph& graph, double mapWidth,
               double mapHeight, double gridCellSize);

    // 从图构建多分辨率聚类索引
    void buildMultiResolution(const Graph& graph, double mapWidth,
                               double mapHeight,
                               const std::vector<double>& cellSizes);

    void buildAdaptiveMultiResolution(const Graph& graph, double mapWidth,
                                      double mapHeight,
                                      const std::vector<size_t>& targetCounts);

    const std::vector<Cluster>& getClusters() const {
        return clusters_;
    }

    const std::vector<ClusterLevel>& getLevels() const {
        return levels_;
    }

    double getGridCellSize() const { return gridCellSize_; }

private:
    void buildSingleLevel(const Graph& graph, double mapWidth,
                           double gridCellSize, std::vector<Cluster>& out);

    void buildAdaptiveLevel(const Graph& graph, double mapWidth,
                            double mapHeight, size_t targetCount,
                            std::vector<Cluster>& out);

    std::vector<Cluster> clusters_;
    std::vector<ClusterLevel> levels_;
    double gridCellSize_ = 0.0;
};

} // namespace nav

#endif // CLUSTERINDEX_H
