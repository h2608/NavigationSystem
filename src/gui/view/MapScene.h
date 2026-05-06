#ifndef MAPSCENE_H
#define MAPSCENE_H

#include <QElapsedTimer>
#include <QGraphicsEllipseItem>
#include <QGraphicsPathItem>
#include <QGraphicsScene>
#include <QRectF>
#include <QTimer>
#include <cstdint>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "core/algorithms/PathFinder.h"
#include "core/graph/Edge.h"
#include "core/graph/Graph.h"
#include "core/graph/Node.h"
#include "gui/items/ClusterItem.h"
#include "gui/view/DisplayFilter.h"

namespace nav {

class NodeItem;
class EdgeItem;

class MapScene : public QGraphicsScene {
    Q_OBJECT

public:
    using NodeItemMap = std::unordered_map<Node::Id, NodeItem*>;
    using EdgeItemMap = std::unordered_map<Edge::Id, EdgeItem*>;

    explicit MapScene(QObject* parent = nullptr);

    void loadMap(const Graph& graph);
    void clearMap();

    NodeItem* getNodeItem(Node::Id id) const;
    EdgeItem* getEdgeItem(Edge::Id id) const;
    void updateEdgeCongestion(Edge::Id id, int congestionStatus);

    void setGraph(Graph* graph) { graph_ = graph; }
    Graph* getGraph() const { return graph_; }
    void setPathFinder(PathFinder* pathfinder) { pathfinder_ = pathfinder; }
    void setHeatmapVisible(bool visible) { heatmapVisible_ = visible; }
    bool isHeatmapVisible() const { return heatmapVisible_; }

    void clearPathSelection();
    const NodeItemMap& getNodeItems() const { return nodeItems_; }
    const EdgeItemMap& getEdgeItems() const { return edgeItems_; }
    void resetAllEdgeStyles();

    void buildClusters(const Graph& graph, double mapWidth, double mapHeight);
    void setActiveClusterLevel(int level);
    bool hasDisplayBand() const { return lastAppliedBandIndex_ >= 0; }
    bool needsDisplayBandRefresh() const { return lastAppliedBandIndex_ < 0 || displayBandDirty_; }
    bool hasPendingDisplayWork() const;
    void cancelPendingDisplayBandWork();
    void cancelDisplayTransition();
    void requestDisplayBandTransition(int bandIndex, const ZoomBand& band,
                                      const QRectF& prioritySceneRect = QRectF());
    void transitionToDisplayBand(int bandIndex, const ZoomBand& band);
    void updateViewZoom(double zoom);

    void highlightNodes(const std::vector<Node::Id>& nodeIds, const QColor& color);
    void highlightEdges(const std::vector<Edge::Id>& edgeIds, const QColor& color);
    void clearSpatialHighlights();
    PathResult findPathById(Node::Id startId, Node::Id endId);

    void showQueryPoint(double x, double y);
    void showTrafficPoint(double x, double y);
    void showTrafficEdges(const std::vector<Edge::Id>& edgeIds, const Graph& graph);
    void clearTrafficHighlights();
    void updateTrafficHighlights(const Graph& graph);
    const std::vector<Edge::Id>& getTrafficHighlightedEdges() const { return trafficHighlightedEdges_; }
    const std::unordered_set<Edge::Id>& getHighlightedEdges() const { return highlightedEdges_; }

signals:
    void pathFound(const PathResult& result);
    void statusMessage(const QString& message);
    void errorOccurred(const QString& title, const QString& message);
    void displayUpdateFinished();

protected:
    void mousePressEvent(QGraphicsSceneMouseEvent* event) override;

private:
    enum class DisplayWorkPhase {
        Idle,
        Edges,
        Nodes,
        Clusters
    };

    void advanceDisplayTransition();
    void queuePendingDisplayBandChunk();
    void processPendingDisplayBandChunk();
    bool processEdgeDisplayWork(QElapsedTimer& budgetTimer, int& processed);
    bool processNodeDisplayWork(QElapsedTimer& budgetTimer, int& processed);
    bool processClusterDisplayWork(QElapsedTimer& budgetTimer, int& processed);
    bool shouldAnimateEdgeTransition(const Edge& edge, EdgeItem* item, bool forceVisible) const;
    void startDisplayTransitionAnimationOrFinish();
    void finishDisplayUpdate();
    void highlightNode(Node::Id id, const QColor& color);
    void showPath(const PathResult& result);
    void clearPath();
    void placeMarker(QGraphicsEllipseItem*& marker, double x, double y,
                     const QColor& fill, const QColor& border);
    void refreshPathStyle();
    int pathZoomBucketFor(double zoom) const;
    qreal edgeOpacityForBand(const Edge& edge, const ZoomBand& band, bool forceVisible) const;
    double edgeWidthScaleForBand(const Edge& edge, const ZoomBand& band, bool forceVisible) const;

    NodeItemMap nodeItems_;
    EdgeItemMap edgeItems_;

    Graph* graph_ = nullptr;
    PathFinder* pathfinder_ = nullptr;
    bool heatmapVisible_ = false;
    Node::Id startNode_ = Node::INVALID_ID;
    Node::Id endNode_ = Node::INVALID_ID;
    int clickState_ = 0;

    QGraphicsPathItem* pathItem_ = nullptr;
    std::vector<Node::Id> highlightedNodes_;
    std::unordered_set<Edge::Id> highlightedEdges_;
    QGraphicsEllipseItem* queryPointMarker_ = nullptr;
    std::vector<Edge::Id> trafficHighlightedEdges_;
    QGraphicsEllipseItem* trafficPointMarker_ = nullptr;

    std::vector<std::vector<ClusterItem*>> clusterLevels_;
    std::vector<EdgeItem*> transitioningEdgeItems_;
    std::vector<NodeItem*> transitioningNodeItems_;
    std::vector<ClusterItem*> transitioningClusterItems_;
    std::unordered_set<Edge::Id> pendingTrafficEdgeSet_;
    DisplayWorkPhase displayWorkPhase_ = DisplayWorkPhase::Idle;
    ZoomBand pendingDisplayBand_{};
    QRectF pendingPrioritySceneRect_;
    EdgeItemMap::iterator pendingEdgeIt_;
    NodeItemMap::iterator pendingNodeIt_;
    size_t pendingClusterLevelIndex_ = 0;
    size_t pendingClusterItemIndex_ = 0;
    uint64_t displayWorkGeneration_ = 0;
    bool pendingDisplayWork_ = false;
    bool displayWorkQueued_ = false;
    int activeClusterLevel_ = -1;
    int lastAppliedBandIndex_ = -1;
    int pathZoomBucket_ = -1;
    bool displayBandDirty_ = false;
    bool showLabels_ = false;
    double currentViewZoom_ = 1.0;

    QTimer displayTransitionTimer_;
    QElapsedTimer displayTransitionClock_;
    int displayTransitionDurationMs_ = 150;
};

} // namespace nav

#endif // MAPSCENE_H
