#ifndef MAPSCENE_H
#define MAPSCENE_H

#include <QGraphicsScene>
#include <QGraphicsPathItem>
#include <QGraphicsEllipseItem>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include "core/graph/Graph.h"
#include "core/graph/Node.h"
#include "core/graph/Edge.h"
#include "core/algorithms/PathFinder.h"

namespace nav {

class NodeItem;
class EdgeItem;

class MapScene : public QGraphicsScene {
    Q_OBJECT

public:
    explicit MapScene(QObject* parent = nullptr);

    // Load a graph into the scene
    void loadMap(const Graph& graph);

    // Clear all map items
    void clearMap();

    // Get items by ID for updates
    NodeItem* getNodeItem(Node::Id id) const;
    EdgeItem* getEdgeItem(Edge::Id id) const;

    // Update edge style based on congestion
    void updateEdgeCongestion(Edge::Id id, int congestionStatus);

    // Set the graph reference for pathfinding
    void setGraph(Graph* graph) { graph_ = graph; }
    Graph* getGraph() const { return graph_; }

    // Set the pathfinder to use
    void setPathFinder(PathFinder* pathfinder) { pathfinder_ = pathfinder; }

    // Heatmap visibility (kept in sync by MainWindow)
    void setHeatmapVisible(bool visible) { heatmapVisible_ = visible; }
    bool isHeatmapVisible() const { return heatmapVisible_; }

    // Clear current path selection
    void clearPathSelection();

    // Get all node items for LOD updates
    const std::unordered_map<Node::Id, NodeItem*>& getNodeItems() const { return nodeItems_; }

    // Highlight a list of nodes (for spatial query results)
    void highlightNodes(const std::vector<Node::Id>& nodeIds, const QColor& color);

    // Highlight a list of edges (for spatial query results)
    void highlightEdges(const std::vector<Edge::Id>& edgeIds, const QColor& color);

    // Clear spatial query highlights
    void clearSpatialHighlights();

    // Find path by explicit node IDs (for control panel)
    PathResult findPathById(Node::Id startId, Node::Id endId);

    // Show query point marker
    void showQueryPoint(double x, double y);
    void showTrafficPoint(double x, double y);

    // Traffic visualization for localized view (F4)
    void showTrafficEdges(const std::vector<Edge::Id>& edgeIds, const Graph& graph);
    void clearTrafficHighlights();
    void updateTrafficHighlights(const Graph& graph);
    const std::vector<Edge::Id>& getTrafficHighlightedEdges() const { return trafficHighlightedEdges_; }

signals:
    void pathFound(const PathResult& result);
    void statusMessage(const QString& message);

protected:
    void mousePressEvent(QGraphicsSceneMouseEvent* event) override;

private:
    void highlightNode(Node::Id id, const QColor& color);
    void showPath(const PathResult& result);
    void clearPath();
    void placeMarker(QGraphicsEllipseItem*& marker, double x, double y,
                     const QColor& fill, const QColor& border);

    std::unordered_map<Node::Id, NodeItem*> nodeItems_;
    std::unordered_map<Edge::Id, EdgeItem*> edgeItems_;

    // Pathfinding state
    Graph* graph_ = nullptr;
    PathFinder* pathfinder_ = nullptr;
    bool heatmapVisible_ = false;
    Node::Id startNode_ = Node::INVALID_ID;
    Node::Id endNode_ = Node::INVALID_ID;
    int clickState_ = 0;  // 0: waiting for start, 1: waiting for end, 2: path shown

    // Path visualization
    QGraphicsPathItem* pathItem_ = nullptr;
    QColor originalStartColor_;
    QColor originalEndColor_;

    // Spatial query visualization
    std::vector<Node::Id> highlightedNodes_;
    std::unordered_set<Edge::Id> highlightedEdges_;
    QGraphicsEllipseItem* queryPointMarker_ = nullptr;

    // Traffic localized view (F4)
    std::vector<Edge::Id> trafficHighlightedEdges_;
    QGraphicsEllipseItem* trafficPointMarker_ = nullptr;
};

} // namespace nav

#endif // MAPSCENE_H
