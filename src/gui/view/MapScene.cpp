#include "gui/view/MapScene.h"

#include <QGraphicsSceneMouseEvent>
#include <QList>
#include <QPainterPath>
#include <QPen>
#include <QEasingCurve>

#include <algorithm>
#include <cmath>
#include <limits>

#include "core/spatial/ClusterIndex.h"
#include "core/traffic/TrafficModel.h"
#include "gui/items/EdgeItem.h"
#include "gui/items/NodeItem.h"

namespace nav {

namespace {

constexpr int kDisplayWorkBudgetMs = 4;
constexpr int kMaxEdgesPerChunk = 700;
constexpr int kMaxNodesPerChunk = 900;
constexpr int kMaxClustersPerChunk = 250;
constexpr double kAnimatedEdgeImportance = 0.72;
constexpr double kPriorityAnimatedEdgeImportance = 0.50;

int edgeCongestionStatus(const Edge& edge) {
    return TrafficModel::getCongestionStatus(
        edge.getCapacity(),
        static_cast<double>(edge.getCarCount())
    );
}

int edgeDisplayStatus(const Edge& edge, bool heatmapVisible) {
    return heatmapVisible ? edgeCongestionStatus(edge) : 0;
}

qreal smoothstep(qreal edge0, qreal edge1, qreal x) {
    if (edge0 == edge1) {
        return x < edge0 ? 0.0 : 1.0;
    }
    const qreal t = std::clamp((x - edge0) / (edge1 - edge0), 0.0, 1.0);
    return t * t * (3.0 - 2.0 * t);
}

std::vector<size_t> adaptiveClusterTargets(size_t nodeCount) {
    if (nodeCount == 0) {
        return {};
    }

    const double root = std::sqrt(static_cast<double>(nodeCount));
    size_t overview = static_cast<size_t>(std::lround(root * 0.45));
    size_t district = static_cast<size_t>(std::lround(root * 1.25));

    overview = std::clamp(overview, static_cast<size_t>(8), static_cast<size_t>(70));
    district = std::clamp(district, static_cast<size_t>(24), static_cast<size_t>(220));

    overview = std::min(overview, nodeCount);
    district = std::min(std::max(district, overview), nodeCount);
    return {overview, district};
}

} // namespace

MapScene::MapScene(QObject* parent)
    : QGraphicsScene(parent)
{
    setBackgroundBrush(QBrush(QColor(249, 249, 246)));

    displayTransitionTimer_.setInterval(16);
    connect(&displayTransitionTimer_, &QTimer::timeout, this, &MapScene::advanceDisplayTransition);
}

void MapScene::loadMap(const Graph& graph) {
    clearMap();
    setItemIndexMethod(NoIndex);

    for (const auto& edgePair : graph.getEdges()) {
        const Edge& edge = edgePair.second;
        const Node* sourceNode = graph.getNode(edge.getSource());
        const Node* targetNode = graph.getNode(edge.getTarget());
        if (!sourceNode || !targetNode) continue;

        EdgeItem* item = new EdgeItem(
            edge.getId(),
            sourceNode->getPosition(),
            targetNode->getPosition(),
            edge.getRoadClass(),
            edge.getDisplayTier(),
            edge.getImportanceScore()
        );
        addItem(item);
        edgeItems_[edge.getId()] = item;
    }

    for (const auto& nodePair : graph.getNodes()) {
        const Node& node = nodePair.second;
        NodeItem* item = new NodeItem(node.getId(), node.getPosition());
        addItem(item);
        nodeItems_[node.getId()] = item;
    }

    setItemIndexMethod(BspTreeIndex);

    const QRectF bounds = itemsBoundingRect();
    const double padding = 100.0;
    setSceneRect(bounds.adjusted(-padding, -padding, padding, padding));
}

void MapScene::clearMap() {
    cancelDisplayTransition();
    clearPath();
    clear();

    nodeItems_.clear();
    edgeItems_.clear();
    highlightedNodes_.clear();
    highlightedEdges_.clear();
    trafficHighlightedEdges_.clear();
    clusterLevels_.clear();
    activeClusterLevel_ = -1;
    startNode_ = Node::INVALID_ID;
    endNode_ = Node::INVALID_ID;
    clickState_ = 0;
    pathItem_ = nullptr;
    queryPointMarker_ = nullptr;
    trafficPointMarker_ = nullptr;
    transitioningEdgeItems_.clear();
    transitioningNodeItems_.clear();
    transitioningClusterItems_.clear();
    pendingTrafficEdgeSet_.clear();
    displayWorkPhase_ = DisplayWorkPhase::Idle;
    pendingDisplayWork_ = false;
    displayWorkQueued_ = false;
    showLabels_ = false;
    currentViewZoom_ = 1.0;
    activeClusterLevel_ = -1;
    lastAppliedBandIndex_ = -1;
    pathZoomBucket_ = -1;
    displayBandDirty_ = false;
}

void MapScene::resetAllEdgeStyles() {
    for (auto& pair : edgeItems_) {
        pair.second->setSpatialHighlight(false);
        pair.second->setTrafficFocus(false);
        pair.second->applyBaseStyle();
    }
}

void MapScene::buildClusters(const Graph& graph, double mapWidth, double mapHeight) {
    for (auto& level : clusterLevels_) {
        for (ClusterItem* item : level) {
            removeItem(item);
            delete item;
        }
    }
    clusterLevels_.clear();
    activeClusterLevel_ = -1;

    ClusterIndex index;
    index.buildAdaptiveMultiResolution(
        graph,
        mapWidth,
        mapHeight,
        adaptiveClusterTargets(graph.getNodeCount())
    );

    for (const auto& level : index.getLevels()) {
        std::vector<ClusterItem*> items;
        for (const auto& cluster : level.clusters) {
            ClusterItem* item = new ClusterItem(cluster.position, cluster.memberCount);
            item->beginVisualTransition(0.0);
            item->applyVisualProgress(1.0);
            addItem(item);
            items.push_back(item);
        }
        clusterLevels_.push_back(std::move(items));
    }
}

void MapScene::setActiveClusterLevel(int level) {
    activeClusterLevel_ = level;
    for (int i = 0; i < static_cast<int>(clusterLevels_.size()); ++i) {
        const qreal opacity = (i == level) ? 1.0 : 0.0;
        for (ClusterItem* item : clusterLevels_[i]) {
            item->beginVisualTransition(opacity);
            item->applyVisualProgress(1.0);
        }
    }
}

bool MapScene::hasPendingDisplayWork() const {
    return pendingDisplayWork_ ||
           displayWorkQueued_ ||
           displayTransitionTimer_.isActive() ||
           displayTransitionClock_.isValid();
}

void MapScene::cancelPendingDisplayBandWork() {
    pendingDisplayWork_ = false;
    displayWorkQueued_ = false;
    displayWorkPhase_ = DisplayWorkPhase::Idle;
    pendingTrafficEdgeSet_.clear();
    ++displayWorkGeneration_;
}

void MapScene::cancelDisplayTransition() {
    const bool hadWork =
        pendingDisplayWork_ ||
        displayWorkQueued_ ||
        displayTransitionTimer_.isActive() ||
        displayTransitionClock_.isValid() ||
        !transitioningEdgeItems_.empty() ||
        !transitioningNodeItems_.empty() ||
        !transitioningClusterItems_.empty();

    if (!hadWork) {
        return;
    }

    cancelPendingDisplayBandWork();
    displayTransitionTimer_.stop();
    displayTransitionClock_.invalidate();
    transitioningEdgeItems_.clear();
    transitioningNodeItems_.clear();
    transitioningClusterItems_.clear();
    displayBandDirty_ = lastAppliedBandIndex_ >= 0;
}

void MapScene::requestDisplayBandTransition(int bandIndex, const ZoomBand& band,
                                            const QRectF& prioritySceneRect) {
    if (lastAppliedBandIndex_ == bandIndex && !displayBandDirty_ && !pendingDisplayWork_) {
        return;
    }

    cancelPendingDisplayBandWork();
    displayTransitionTimer_.stop();
    displayTransitionClock_.invalidate();
    transitioningEdgeItems_.clear();
    transitioningNodeItems_.clear();
    transitioningClusterItems_.clear();

    lastAppliedBandIndex_ = bandIndex;
    displayBandDirty_ = false;
    showLabels_ = band.showLabels;
    activeClusterLevel_ = band.clusterLevel;
    pendingDisplayBand_ = band;
    pendingPrioritySceneRect_ = prioritySceneRect;
    pendingTrafficEdgeSet_ = std::unordered_set<Edge::Id>(
        trafficHighlightedEdges_.begin(),
        trafficHighlightedEdges_.end()
    );

    pendingEdgeIt_ = edgeItems_.begin();
    pendingNodeIt_ = nodeItems_.begin();
    pendingClusterLevelIndex_ = 0;
    pendingClusterItemIndex_ = 0;
    displayWorkPhase_ = DisplayWorkPhase::Edges;
    pendingDisplayWork_ = true;
    queuePendingDisplayBandChunk();
}

void MapScene::transitionToDisplayBand(int bandIndex, const ZoomBand& band) {
    requestDisplayBandTransition(bandIndex, band);
}

void MapScene::queuePendingDisplayBandChunk() {
    if (!pendingDisplayWork_ || displayWorkQueued_) {
        return;
    }

    const uint64_t generation = displayWorkGeneration_;
    displayWorkQueued_ = true;
    QTimer::singleShot(0, this, [this, generation]() {
        displayWorkQueued_ = false;
        if (generation != displayWorkGeneration_ || !pendingDisplayWork_) {
            return;
        }
        processPendingDisplayBandChunk();
    });
}

void MapScene::processPendingDisplayBandChunk() {
    if (!pendingDisplayWork_) {
        return;
    }

    QElapsedTimer budgetTimer;
    budgetTimer.start();
    int processed = 0;

    while (pendingDisplayWork_) {
        if (displayWorkPhase_ == DisplayWorkPhase::Edges) {
            if (!processEdgeDisplayWork(budgetTimer, processed)) {
                queuePendingDisplayBandChunk();
                return;
            }
            displayWorkPhase_ = DisplayWorkPhase::Nodes;
        }

        if (displayWorkPhase_ == DisplayWorkPhase::Nodes) {
            if (!processNodeDisplayWork(budgetTimer, processed)) {
                queuePendingDisplayBandChunk();
                return;
            }
            displayWorkPhase_ = DisplayWorkPhase::Clusters;
        }

        if (displayWorkPhase_ == DisplayWorkPhase::Clusters) {
            if (!processClusterDisplayWork(budgetTimer, processed)) {
                queuePendingDisplayBandChunk();
                return;
            }
            pendingDisplayWork_ = false;
            displayWorkPhase_ = DisplayWorkPhase::Idle;
            pendingTrafficEdgeSet_.clear();
            startDisplayTransitionAnimationOrFinish();
            return;
        }
    }
}

bool MapScene::processEdgeDisplayWork(QElapsedTimer& budgetTimer, int& processed) {
    while (pendingEdgeIt_ != edgeItems_.end()) {
        const Edge::Id edgeId = pendingEdgeIt_->first;
        EdgeItem* item = pendingEdgeIt_->second;
        ++pendingEdgeIt_;
        ++processed;

        if (item && graph_) {
            if (const Edge* edge = graph_->getEdge(edgeId)) {
                const bool forceVisible =
                    highlightedEdges_.count(edgeId) > 0 ||
                    pendingTrafficEdgeSet_.count(edgeId) > 0;
                const qreal targetOpacity = edgeOpacityForBand(*edge, pendingDisplayBand_, forceVisible);
                const double targetWidthScale = edgeWidthScaleForBand(*edge, pendingDisplayBand_, forceVisible);

                if (item->beginVisualTransition(targetOpacity, targetWidthScale)) {
                    if (shouldAnimateEdgeTransition(*edge, item, forceVisible)) {
                        transitioningEdgeItems_.push_back(item);
                    } else {
                        item->applyVisualProgress(1.0);
                    }
                }
            }
        }

        if (processed >= kMaxEdgesPerChunk || budgetTimer.elapsed() >= kDisplayWorkBudgetMs) {
            return pendingEdgeIt_ == edgeItems_.end();
        }
    }
    return true;
}

bool MapScene::processNodeDisplayWork(QElapsedTimer& budgetTimer, int& processed) {
    while (pendingNodeIt_ != nodeItems_.end()) {
        NodeItem* item = pendingNodeIt_->second;
        ++pendingNodeIt_;
        ++processed;

        if (item) {
            if (item->beginVisualTransition(item->isInteractive() ? 1.0 : pendingDisplayBand_.nodeOpacity)) {
                if (item->isInteractive()) {
                    transitioningNodeItems_.push_back(item);
                } else {
                    item->applyVisualProgress(1.0);
                }
            }
        }

        if (processed >= kMaxNodesPerChunk || budgetTimer.elapsed() >= kDisplayWorkBudgetMs) {
            return pendingNodeIt_ == nodeItems_.end();
        }
    }
    return true;
}

bool MapScene::processClusterDisplayWork(QElapsedTimer& budgetTimer, int& processed) {
    while (pendingClusterLevelIndex_ < clusterLevels_.size()) {
        auto& level = clusterLevels_[pendingClusterLevelIndex_];
        while (pendingClusterItemIndex_ < level.size()) {
            ClusterItem* item = level[pendingClusterItemIndex_];
            ++pendingClusterItemIndex_;
            ++processed;

            const qreal targetOpacity =
                (static_cast<int>(pendingClusterLevelIndex_) == pendingDisplayBand_.clusterLevel)
                    ? pendingDisplayBand_.clusterOpacity
                    : 0.0;
            if (item && item->beginVisualTransition(targetOpacity)) {
                transitioningClusterItems_.push_back(item);
            }

            if (processed >= kMaxClustersPerChunk || budgetTimer.elapsed() >= kDisplayWorkBudgetMs) {
                return pendingClusterLevelIndex_ >= clusterLevels_.size();
            }
        }

        pendingClusterItemIndex_ = 0;
        ++pendingClusterLevelIndex_;
    }
    return true;
}

bool MapScene::shouldAnimateEdgeTransition(const Edge& edge, EdgeItem* item, bool forceVisible) const {
    if (forceVisible) {
        return true;
    }

    if (!pendingPrioritySceneRect_.isEmpty() &&
        pendingPrioritySceneRect_.intersects(item->sceneBoundingRect())) {
        return edge.getImportanceScore() >= kPriorityAnimatedEdgeImportance;
    }

    return edge.getImportanceScore() >= kAnimatedEdgeImportance;
}

void MapScene::startDisplayTransitionAnimationOrFinish() {
    if (transitioningEdgeItems_.empty() &&
        transitioningNodeItems_.empty() &&
        transitioningClusterItems_.empty()) {
        displayTransitionTimer_.stop();
        displayTransitionClock_.invalidate();
        finishDisplayUpdate();
        return;
    }

    displayTransitionClock_.restart();
    if (!displayTransitionTimer_.isActive()) {
        displayTransitionTimer_.start();
    }
}

void MapScene::finishDisplayUpdate() {
    emit displayUpdateFinished();
}

void MapScene::updateViewZoom(double zoom) {
    currentViewZoom_ = zoom;

    const int nextBucket = pathZoomBucketFor(zoom);
    if (nextBucket == pathZoomBucket_) {
        return;
    }

    pathZoomBucket_ = nextBucket;
    refreshPathStyle();
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

    if (startNode_ != Node::INVALID_ID) {
        if (NodeItem* item = getNodeItem(startNode_)) {
            item->setBrush(QBrush(QColor(66, 72, 78)));
            item->setPen(Qt::NoPen);
            item->setHighlighted(false);
        }
    }

    if (endNode_ != Node::INVALID_ID) {
        if (NodeItem* item = getNodeItem(endNode_)) {
            item->setBrush(QBrush(QColor(66, 72, 78)));
            item->setPen(Qt::NoPen);
            item->setHighlighted(false);
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

    const QList<QGraphicsItem*> clickedItems = items(event->scenePos());
    NodeItem* clickedNode = nullptr;
    for (QGraphicsItem* item : clickedItems) {
        clickedNode = dynamic_cast<NodeItem*>(item);
        if (clickedNode) break;
    }

    if (!clickedNode) {
        QGraphicsScene::mousePressEvent(event);
        return;
    }

    const Node::Id clickedId = clickedNode->getNodeId();

    switch (clickState_) {
        case 0:
            clearPathSelection();
            startNode_ = clickedId;
            highlightNode(startNode_, QColor(33, 150, 243));
            clickState_ = 1;
            emit statusMessage(QString("Start node selected: %1").arg(startNode_));
            break;
        case 1:
            if (clickedId == startNode_) {
                break;
            }
            endNode_ = clickedId;
            highlightNode(endNode_, QColor(156, 39, 176));

            if (graph_ && pathfinder_) {
                PathResult result = pathfinder_->findPath(*graph_, startNode_, endNode_);
                if (result.found) {
                    showPath(result);
                    emit pathFound(result);
                    emit statusMessage(QString("Path found with %1 nodes, cost %2")
                        .arg(result.pathNodes.size())
                        .arg(result.totalCost, 0, 'f', 2));
                } else {
                    emit errorOccurred("Path", "No route found between the selected nodes.");
                }
            } else {
                emit errorOccurred("Path", "Graph or path finder is not ready.");
            }
            clickState_ = 2;
            break;
        case 2:
            clearPathSelection();
            startNode_ = clickedId;
            highlightNode(startNode_, QColor(33, 150, 243));
            clickState_ = 1;
            emit statusMessage(QString("Start node selected: %1").arg(startNode_));
            break;
    }

    QGraphicsScene::mousePressEvent(event);
}

void MapScene::highlightNode(Node::Id id, const QColor& color) {
    NodeItem* item = getNodeItem(id);
    if (!item) return;

    item->setBrush(QBrush(color));
    item->setPen(QPen(color.darker(130), 2));
    item->setHighlighted(true);
    item->setZValue(12.0);
}

void MapScene::showPath(const PathResult& result) {
    clearPath();

    if (!graph_ || result.pathNodes.size() < 2) return;

    QPainterPath path;
    if (const Node* firstNode = graph_->getNode(result.pathNodes.front())) {
        path.moveTo(firstNode->getPosition().x, firstNode->getPosition().y);
    }

    for (size_t i = 1; i < result.pathNodes.size(); ++i) {
        const Node* node = graph_->getNode(result.pathNodes[i]);
        if (!node) continue;
        path.lineTo(node->getPosition().x, node->getPosition().y);
    }

    pathItem_ = new QGraphicsPathItem(path);
    pathItem_->setZValue(5.0);
    addItem(pathItem_);
    refreshPathStyle();
}

void MapScene::clearPath() {
    if (!pathItem_) return;

    removeItem(pathItem_);
    delete pathItem_;
    pathItem_ = nullptr;
}

void MapScene::highlightNodes(const std::vector<Node::Id>& nodeIds, const QColor& color) {
    clearSpatialHighlights();
    highlightedNodes_ = nodeIds;

    for (Node::Id id : nodeIds) {
        NodeItem* item = getNodeItem(id);
        if (!item) continue;
        item->setBrush(QBrush(color));
        item->setPen(QPen(color.darker(120), 2));
        item->setHighlighted(true);
        item->setZValue(8.0);
    }
}

void MapScene::highlightEdges(const std::vector<Edge::Id>& edgeIds, const QColor& color) {
    highlightedEdges_.insert(edgeIds.begin(), edgeIds.end());

    for (Edge::Id id : edgeIds) {
        EdgeItem* item = getEdgeItem(id);
        if (!item) continue;
        item->setSpatialHighlight(true, color);
    }
}

void MapScene::clearSpatialHighlights() {
    for (Node::Id id : highlightedNodes_) {
        NodeItem* item = getNodeItem(id);
        if (!item) continue;
        item->setBrush(QBrush(QColor(66, 72, 78)));
        item->setPen(Qt::NoPen);
        item->setHighlighted(false);
        item->setZValue(10.0);
    }
    highlightedNodes_.clear();

    for (Edge::Id id : highlightedEdges_) {
        EdgeItem* item = getEdgeItem(id);
        if (!item) continue;
        item->setSpatialHighlight(false);

        int status = 0;
        if (heatmapVisible_ && graph_) {
            if (const Edge* edge = graph_->getEdge(id)) {
                status = edgeDisplayStatus(*edge, heatmapVisible_);
            }
        }
        item->updateStyle(status);
    }
    highlightedEdges_.clear();

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
        emit errorOccurred("Path", "Graph or path finder is not ready.");
        return result;
    }

    clearPathSelection();
    startNode_ = startId;
    endNode_ = endId;

    highlightNode(startNode_, QColor(33, 150, 243));
    highlightNode(endNode_, QColor(156, 39, 176));

    result = pathfinder_->findPath(*graph_, startNode_, endNode_);
    if (result.found) {
        showPath(result);
        emit pathFound(result);
        emit statusMessage(QString("Path found with %1 nodes, cost %2")
            .arg(result.pathNodes.size())
            .arg(result.totalCost, 0, 'f', 2));
    } else {
        emit errorOccurred("Path", QString("No route found between node %1 and node %2.")
            .arg(startId).arg(endId));
    }

    clickState_ = 2;
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
    marker->setFlag(QGraphicsItem::ItemIgnoresTransformations, true);
    marker->setZValue(15.0);
    addItem(marker);
}

void MapScene::showTrafficEdges(const std::vector<Edge::Id>& edgeIds, const Graph& graph) {
    clearTrafficHighlights();
    trafficHighlightedEdges_ = edgeIds;

    for (Edge::Id id : edgeIds) {
        EdgeItem* item = getEdgeItem(id);
        const Edge* edge = graph.getEdge(id);
        if (!item || !edge) continue;

        item->setTrafficFocus(true);
        item->updateStyle(edgeDisplayStatus(*edge, heatmapVisible_));
    }
}

void MapScene::clearTrafficHighlights() {
    for (Edge::Id id : trafficHighlightedEdges_) {
        EdgeItem* item = getEdgeItem(id);
        if (!item) continue;

        item->setTrafficFocus(false);
        int status = 0;
        if (heatmapVisible_ && graph_) {
            if (const Edge* edge = graph_->getEdge(id)) {
                status = edgeDisplayStatus(*edge, heatmapVisible_);
            }
        }
        item->updateStyle(status);
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
        if (!item || !edge) continue;

        item->updateStyle(edgeDisplayStatus(*edge, heatmapVisible_));
    }
}

void MapScene::advanceDisplayTransition() {
    if (!displayTransitionClock_.isValid()) {
        displayTransitionTimer_.stop();
        return;
    }

    const qreal progress = std::clamp(
        static_cast<qreal>(displayTransitionClock_.elapsed()) /
        static_cast<qreal>(displayTransitionDurationMs_),
        0.0,
        1.0
    );
    const qreal eased = QEasingCurve(QEasingCurve::OutCubic).valueForProgress(progress);

    for (EdgeItem* item : transitioningEdgeItems_) {
        if (item) {
            item->applyVisualProgress(eased);
        }
    }
    for (NodeItem* item : transitioningNodeItems_) {
        if (item) {
            item->applyVisualProgress(eased);
        }
    }
    for (ClusterItem* item : transitioningClusterItems_) {
        if (item) {
            item->applyVisualProgress(eased);
        }
    }

    if (progress >= 1.0) {
        displayTransitionTimer_.stop();
        displayTransitionClock_.invalidate();
        transitioningEdgeItems_.clear();
        transitioningNodeItems_.clear();
        transitioningClusterItems_.clear();
        finishDisplayUpdate();
    }
}

void MapScene::refreshPathStyle() {
    if (!pathItem_) return;

    const double normalized = std::clamp(
        (std::log2(std::max(currentViewZoom_, 0.05)) + 3.0) / 5.0,
        0.0,
        1.0
    );

    QPen pen(QColor(33, 150, 243, 225));
    pen.setWidthF(4.4 + normalized * 1.8);
    pen.setCapStyle(Qt::RoundCap);
    pen.setJoinStyle(Qt::RoundJoin);
    pathItem_->setPen(pen);
}

int MapScene::pathZoomBucketFor(double zoom) const {
    const double normalized = std::clamp(
        (std::log2(std::max(zoom, 0.05)) + 3.0) / 5.0,
        0.0,
        1.0
    );
    return static_cast<int>(std::lround(normalized * 8.0));
}

qreal MapScene::edgeOpacityForBand(const Edge& edge, const ZoomBand& band, bool forceVisible) const {
    if (forceVisible) return 1.0;
    if (band.minEdgeImportance <= 0.0) return static_cast<qreal>(band.edgeOpacity);

    const qreal softness = 0.08;
    const qreal importance = static_cast<qreal>(edge.getImportanceScore());
    const qreal minImportance = static_cast<qreal>(band.minEdgeImportance);
    return smoothstep(minImportance - softness, minImportance + softness, importance) *
           static_cast<qreal>(band.edgeOpacity);
}

double MapScene::edgeWidthScaleForBand(const Edge& edge, const ZoomBand& band, bool forceVisible) const {
    double scale = band.widthScale * (0.92 + edge.getImportanceScore() * 0.18);
    if (forceVisible) {
        scale += 0.08;
    }
    return scale;
}

} // namespace nav
