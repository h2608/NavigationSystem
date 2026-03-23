#include "gui/view/MapScene.h"
#include "gui/items/NodeItem.h"
#include "gui/items/EdgeItem.h"
#include "core/traffic/TrafficModel.h"
#include <QGraphicsSceneMouseEvent>
#include <QPainterPath>
#include <QPen>
#include <iostream>

namespace nav {

namespace {

int edgeCongestionStatus(const Edge& edge) {
    return TrafficModel::getCongestionStatus(
        edge.getCapacity(),
        static_cast<double>(edge.getCarCount())
    );
}

int edgeDisplayStatus(const Edge& edge, bool heatmapVisible) {
    return heatmapVisible ? edgeCongestionStatus(edge) : 0;
}

} // namespace

MapScene::MapScene(QObject* parent)
    : QGraphicsScene(parent)
{
    // Set a light gray background
    setBackgroundBrush(QBrush(QColor(245, 245, 245)));
}

void MapScene::loadMap(const Graph& graph) {
    // Clear existing items
    clearMap();

    // Disable indexing during bulk insertion for performance
    setItemIndexMethod(NoIndex);

    // Add edges first (so they appear below nodes)
    for (const auto& edgePair : graph.getEdges()) {
        const Edge& edge = edgePair.second;

        const Node* sourceNode = graph.getNode(edge.getSource());
        const Node* targetNode = graph.getNode(edge.getTarget());

        if (sourceNode && targetNode) {
            EdgeItem* item = new EdgeItem(
                edge.getId(),
                sourceNode->getPosition(),
                targetNode->getPosition()
            );
            addItem(item);
            edgeItems_[edge.getId()] = item;
        }
    }

    // Add nodes
    for (const auto& nodePair : graph.getNodes()) {
        const Node& node = nodePair.second;

        NodeItem* item = new NodeItem(node.getId(), node.getPosition());
        addItem(item);
        nodeItems_[node.getId()] = item;
    }

    // Re-enable BSP tree indexing for efficient queries
    setItemIndexMethod(BspTreeIndex);

    // Set scene rect to fit all items with some padding
    QRectF bounds = itemsBoundingRect();
    double padding = 100.0;
    setSceneRect(bounds.adjusted(-padding, -padding, padding, padding));
}

void MapScene::clearMap() {
    clearPath();
    clear();
    nodeItems_.clear();
    edgeItems_.clear();
    highlightedNodes_.clear();
    highlightedEdges_.clear();
    trafficHighlightedEdges_.clear();
    startNode_ = Node::INVALID_ID;
    endNode_ = Node::INVALID_ID;
    clickState_ = 0;
    pathItem_ = nullptr;
    queryPointMarker_ = nullptr;
    trafficPointMarker_ = nullptr;
}

NodeItem* MapScene::getNodeItem(Node::Id id) const {
    auto it = nodeItems_.find(id);
    return (it != nodeItems_.end()) ? it->second : nullptr;
}

EdgeItem* MapScene::getEdgeItem(Edge::Id id) const {
    auto it = edgeItems_.find(id);
    return (it != edgeItems_.end()) ? it->second : nullptr;
}

void MapScene::updateEdgeCongestion(Edge::Id id, int congestionStatus) {
    if (highlightedEdges_.count(id)) {
        return;
    }

    EdgeItem* item = getEdgeItem(id);
    if (item) {
        item->updateStyle(congestionStatus);
    }
}

void MapScene::clearPathSelection() {
    clearPath();

    // Reset node colors and highlight state
    if (startNode_ != Node::INVALID_ID) {
        NodeItem* item = getNodeItem(startNode_);
        if (item) {
            item->setBrush(QBrush(QColor(60, 60, 60)));  // Original dark gray
            item->setPen(Qt::NoPen);
            item->setHighlighted(false);  // Reset to small size
        }
    }
    if (endNode_ != Node::INVALID_ID) {
        NodeItem* item = getNodeItem(endNode_);
        if (item) {
            item->setBrush(QBrush(QColor(60, 60, 60)));  // Original dark gray
            item->setPen(Qt::NoPen);
            item->setHighlighted(false);  // Reset to small size
        }
    }

    startNode_ = Node::INVALID_ID;
    endNode_ = Node::INVALID_ID;
    clickState_ = 0;
}

void MapScene::mousePressEvent(QGraphicsSceneMouseEvent* event) {
    if (event->button() != Qt::LeftButton) {
        QGraphicsScene::mousePressEvent(event);
        return;
    }

    // Find clicked node
    QList<QGraphicsItem*> clickedItems = items(event->scenePos());
    NodeItem* clickedNode = nullptr;

    for (QGraphicsItem* item : clickedItems) {
        clickedNode = dynamic_cast<NodeItem*>(item);
        if (clickedNode) break;
    }

    if (!clickedNode) {
        QGraphicsScene::mousePressEvent(event);
        return;
    }

    Node::Id clickedId = clickedNode->getNodeId();

    switch (clickState_) {
        case 0:  // First click: set start node
            clearPathSelection();
            startNode_ = clickedId;
            highlightNode(startNode_, QColor(33, 150, 243));  // Blue
            clickState_ = 1;
            emit statusMessage(QString("Start: Node %1. Click another node for destination.").arg(startNode_));
            break;

        case 1:  // Second click: set end node and find path
            if (clickedId == startNode_) {
                // Clicked same node, ignore
                break;
            }
            endNode_ = clickedId;
            highlightNode(endNode_, QColor(156, 39, 176));  // Purple

            // Find path
            if (graph_ && pathfinder_) {
                PathResult result = pathfinder_->findPath(*graph_, startNode_, endNode_);
                if (result.found) {
                    showPath(result);
                    emit pathFound(result);
                    emit statusMessage(QString("Path found: %1 nodes, cost: %2")
                                           .arg(result.pathNodes.size())
                                           .arg(result.totalCost, 0, 'f', 2));
                } else {
                    emit statusMessage("No path found!");
                }
            } else {
                emit statusMessage("Pathfinder not configured!");
            }
            clickState_ = 2;
            break;

        case 2:  // Third click: clear and start over
            clearPathSelection();
            startNode_ = clickedId;
            highlightNode(startNode_, QColor(33, 150, 243));  // Blue
            clickState_ = 1;
            emit statusMessage(QString("Start: Node %1. Click another node for destination.").arg(startNode_));
            break;
    }

    QGraphicsScene::mousePressEvent(event);
}

void MapScene::highlightNode(Node::Id id, const QColor& color) {
    NodeItem* item = getNodeItem(id);
    if (item) {
        item->setBrush(QBrush(color));
        item->setPen(QPen(color.darker(130), 2));  // Add border
        item->setHighlighted(true);  // Make visually larger (16px diameter)
        item->setZValue(12.0);  // Bring to front
    }
}

void MapScene::showPath(const PathResult& result) {
    clearPath();

    if (!graph_ || result.pathNodes.size() < 2) return;

    // Create path
    QPainterPath path;

    const Node* firstNode = graph_->getNode(result.pathNodes[0]);
    if (firstNode) {
        path.moveTo(firstNode->getPosition().x, firstNode->getPosition().y);
    }

    for (size_t i = 1; i < result.pathNodes.size(); ++i) {
        const Node* node = graph_->getNode(result.pathNodes[i]);
        if (node) {
            path.lineTo(node->getPosition().x, node->getPosition().y);
        }
    }

    // Create path item
    pathItem_ = new QGraphicsPathItem(path);
    QPen pen(QColor(33, 150, 243, 220));  // Semi-transparent blue
    pen.setWidthF(6.0);  // Thicker path for visibility
    pen.setCapStyle(Qt::RoundCap);
    pen.setJoinStyle(Qt::RoundJoin);
    pathItem_->setPen(pen);
    pathItem_->setZValue(5.0);  // Above edges, below highlighted nodes

    addItem(pathItem_);
}

void MapScene::clearPath() {
    if (pathItem_) {
        removeItem(pathItem_);
        delete pathItem_;
        pathItem_ = nullptr;
    }
}

void MapScene::highlightNodes(const std::vector<Node::Id>& nodeIds, const QColor& color) {
    // Clear previous highlights first
    clearSpatialHighlights();

    highlightedNodes_ = nodeIds;

    for (Node::Id id : nodeIds) {
        NodeItem* item = getNodeItem(id);
        if (item) {
            item->setBrush(QBrush(color));
            item->setPen(QPen(color.darker(120), 2));  // Add border for visibility
            item->setHighlighted(true);  // Make visually larger (16px diameter)
            item->setZValue(8.0);  // Above normal nodes
        }
    }
}

void MapScene::highlightEdges(const std::vector<Edge::Id>& edgeIds, const QColor& color) {
    highlightedEdges_.insert(edgeIds.begin(), edgeIds.end());

    QPen pen(color);
    pen.setWidthF(3.0);
    pen.setCapStyle(Qt::RoundCap);

    for (Edge::Id id : edgeIds) {
        EdgeItem* item = getEdgeItem(id);
        if (item) {
            item->setPen(pen);
            item->setZValue(2.0);  // Above normal edges
        }
    }
}

void MapScene::clearSpatialHighlights() {
    // Reset previously highlighted nodes to default
    for (Node::Id id : highlightedNodes_) {
        NodeItem* item = getNodeItem(id);
        if (item) {
            item->setBrush(QBrush(QColor(60, 60, 60)));  // Original dark gray
            item->setPen(Qt::NoPen);  // Remove border
            item->setHighlighted(false);  // Reset to small size
            item->setZValue(10.0);  // Original Z
        }
    }
    highlightedNodes_.clear();

    // Restore previously highlighted edges to correct state
    for (Edge::Id id : highlightedEdges_) {
        EdgeItem* item = getEdgeItem(id);
        if (item) {
            int status = 0;  // Default green when heatmap is off
            if (heatmapVisible_ && graph_) {
                const Edge* edge = graph_->getEdge(id);
                if (edge) {
                    status = edgeDisplayStatus(*edge, heatmapVisible_);
                }
            }
            item->updateStyle(status);
            item->setZValue(0.0);
        }
    }
    highlightedEdges_.clear();

    // Remove query point marker
    if (queryPointMarker_) {
        removeItem(queryPointMarker_);
        delete queryPointMarker_;
        queryPointMarker_ = nullptr;
    }
}

PathResult MapScene::findPathById(Node::Id startId, Node::Id endId) {
    PathResult result;

    if (!graph_ || !pathfinder_) {
        result.found = false;
        emit statusMessage("Pathfinder not configured!");
        return result;
    }

    // Clear previous path selection
    clearPathSelection();

    // Set start and end nodes
    startNode_ = startId;
    endNode_ = endId;

    // Highlight start and end nodes
    highlightNode(startNode_, QColor(33, 150, 243));  // Blue
    highlightNode(endNode_, QColor(156, 39, 176));    // Purple

    // Find path
    result = pathfinder_->findPath(*graph_, startNode_, endNode_);

    if (result.found) {
        showPath(result);
        emit pathFound(result);
        emit statusMessage(QString("Path found: %1 nodes, cost: %2")
                               .arg(result.pathNodes.size())
                               .arg(result.totalCost, 0, 'f', 2));
    } else {
        emit statusMessage(QString("No path found from Node %1 to Node %2!")
                               .arg(startId).arg(endId));
    }

    clickState_ = 2;  // Path shown state
    return result;
}

void MapScene::showQueryPoint(double x, double y) {
    placeMarker(queryPointMarker_, x, y, QColor(255, 0, 0, 150), QColor(200, 0, 0));
}

void MapScene::showTrafficPoint(double x, double y) {
    placeMarker(trafficPointMarker_, x, y, QColor(255, 87, 34, 150), QColor(230, 74, 25));
}

void MapScene::placeMarker(QGraphicsEllipseItem*& marker, double x, double y,
                            const QColor& fill, const QColor& border) {
    if (marker) {
        removeItem(marker);
        delete marker;
    }
    marker = new QGraphicsEllipseItem(x - 8, y - 8, 16, 16);
    marker->setBrush(QBrush(fill));
    marker->setPen(QPen(border, 2));
    marker->setZValue(15.0);
    addItem(marker);
}

void MapScene::showTrafficEdges(const std::vector<Edge::Id>& edgeIds, const Graph& graph) {
    clearTrafficHighlights();
    trafficHighlightedEdges_ = edgeIds;

    for (Edge::Id id : edgeIds) {
        EdgeItem* item = getEdgeItem(id);
        const Edge* edge = graph.getEdge(id);
        if (item && edge) {
            int status = edgeDisplayStatus(*edge, heatmapVisible_);
            item->updateStyle(status);
            item->setZValue(2.0);  // Above normal edges
        }
    }
}

void MapScene::clearTrafficHighlights() {
    // Restore edges to correct state based on heatmap visibility
    for (Edge::Id id : trafficHighlightedEdges_) {
        EdgeItem* item = getEdgeItem(id);
        if (item) {
            int status = 0;  // Default green when heatmap is off
            if (heatmapVisible_ && graph_) {
                const Edge* edge = graph_->getEdge(id);
                if (edge) {
                    status = edgeDisplayStatus(*edge, heatmapVisible_);
                }
            }
            item->updateStyle(status);
            item->setZValue(0.0);
        }
    }
    trafficHighlightedEdges_.clear();

    if (trafficPointMarker_) {
        removeItem(trafficPointMarker_);
        delete trafficPointMarker_;
        trafficPointMarker_ = nullptr;
    }
}

void MapScene::updateTrafficHighlights(const Graph& graph) {
    for (Edge::Id id : trafficHighlightedEdges_) {
        EdgeItem* item = getEdgeItem(id);
        const Edge* edge = graph.getEdge(id);
        if (item && edge) {
            int status = edgeDisplayStatus(*edge, heatmapVisible_);
            item->updateStyle(status);
        }
    }
}

} // namespace nav
