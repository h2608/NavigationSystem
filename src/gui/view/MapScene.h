#ifndef MAPSCENE_H
#define MAPSCENE_H

#include <QElapsedTimer>
#include <QGraphicsEllipseItem>
#include <QGraphicsPathItem>
#include <QGraphicsScene>
#include <QTimer>
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
    const std::unordered_map<Node::Id, NodeItem*>& getNodeItems() const { return nodeItems_; }
    const std::unordered_map<Edge::Id, EdgeItem*>& getEdgeItems() const { return edgeItems_; }
    void resetAllEdgeStyles();

    void buildClusters(const Graph& graph, double mapWidth, double mapHeight);
    void setActiveClusterLevel(int level);
    void transitionToDisplayBand(const ZoomBand& band, double zoom);

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

protected:
    void mousePressEvent(QGraphicsSceneMouseEvent* event) override;

private:
    void advanceDisplayTransition();
    void highlightNode(Node::Id id, const QColor& color);
    void showPath(const PathResult& result);
    void clearPath();
    void placeMarker(QGraphicsEllipseItem*& marker, double x, double y,
                     const QColor& fill, const QColor& border);
    void refreshPathStyle();
    qreal edgeOpacityForBand(const Edge& edge, const ZoomBand& band, bool forceVisible) const;
    double edgeWidthScaleForBand(const Edge& edge, const ZoomBand& band, bool forceVisible) const;

    std::unordered_map<Node::Id, NodeItem*> nodeItems_;
    std::unordered_map<Edge::Id, EdgeItem*> edgeItems_;

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
    int activeClusterLevel_ = -1;
    bool showLabels_ = false;
    double currentViewZoom_ = 1.0;

    QTimer displayTransitionTimer_;
    QElapsedTimer displayTransitionClock_;
    int displayTransitionDurationMs_ = 150;
};

} // namespace nav

#endif // MAPSCENE_H
