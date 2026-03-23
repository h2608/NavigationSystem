#include "gui/MainWindow.h"
#include "gui/view/MapView.h"
#include "gui/view/MapScene.h"
#include "gui/widgets/ControlPanel.h"
#include "gui/widgets/GenerateMapDialog.h"
#include "core/spatial/BoundingBox.h"
#include <QMenuBar>
#include <QToolBar>
#include <QAction>
#include <QStatusBar>
#include <QLabel>
#include <algorithm>
#include <iostream>
#include <unordered_set>
#include <limits>

namespace nav {

MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent)
    , mapView_(nullptr)
    , mapScene_(nullptr)
    , controlPanel_(nullptr)
    , graph_(std::make_unique<Graph>())
    , generator_(std::make_unique<GridPerturbationGenerator>())
    , dijkstraPathfinder_(std::make_unique<DijkstraPathFinder>())
    , dynamicPathfinder_(std::make_unique<DynamicPathFinder>())
    , simulationTimer_(new QTimer(this))
    , simulationAction_(nullptr)
{
    setupUi();
    setupMenuBar();
    setupToolBar();
    setupControlPanel();

    // Setup simulation timer
    connect(simulationTimer_, &QTimer::timeout, this, &MainWindow::onSimulationStep);

    // Connect scene signals
    connect(mapScene_, &MapScene::pathFound, this, &MainWindow::onPathFound);
    connect(mapScene_, &MapScene::statusMessage, this, &MainWindow::onStatusMessage);

    // Set window properties
    setWindowTitle("Navigation System");
    resize(1400, 900);

    // Show status
    statusBar()->showMessage("Ready. Use 'Generate Map' to create a new map. Click nodes or use Control Panel.");
}

MainWindow::~MainWindow() {
    simulationTimer_->stop();
}

void MainWindow::setupUi() {
    // Create scene and view
    mapScene_ = new MapScene(this);
    mapView_ = new MapView(this);
    mapView_->setScene(mapScene_);

    // Set as central widget
    setCentralWidget(mapView_);
}

void MainWindow::setupMenuBar() {
    // File menu
    QMenu* fileMenu = menuBar()->addMenu("&File");

    QAction* generateAction = fileMenu->addAction("&Generate New Map");
    generateAction->setShortcut(QKeySequence("Ctrl+G"));
    connect(generateAction, &QAction::triggered, this, &MainWindow::onGenerateMap);

    fileMenu->addSeparator();

    QAction* exitAction = fileMenu->addAction("E&xit");
    exitAction->setShortcut(QKeySequence::Quit);
    connect(exitAction, &QAction::triggered, this, &QMainWindow::close);

    // View menu
    QMenu* viewMenu = menuBar()->addMenu("&View");

    QAction* zoomFitAction = viewMenu->addAction("Zoom to &Fit");
    zoomFitAction->setShortcut(QKeySequence("Ctrl+0"));
    connect(zoomFitAction, &QAction::triggered, this, &MainWindow::onZoomToFit);

    // Simulation menu
    QMenu* simMenu = menuBar()->addMenu("&Simulation");

    simulationAction_ = simMenu->addAction("&Show Traffic Heatmap");
    simulationAction_->setShortcut(QKeySequence("Ctrl+T"));
    simulationAction_->setCheckable(true);
    connect(simulationAction_, &QAction::triggered, this, &MainWindow::onToggleSimulation);
}

void MainWindow::setupToolBar() {
    QToolBar* toolbar = addToolBar("Main Toolbar");
    toolbar->setMovable(false);

    QAction* generateAction = toolbar->addAction("Generate Map");
    connect(generateAction, &QAction::triggered, this, &MainWindow::onGenerateMap);

    QAction* zoomFitAction = toolbar->addAction("Zoom to Fit");
    connect(zoomFitAction, &QAction::triggered, this, &MainWindow::onZoomToFit);

    toolbar->addSeparator();

    // Add heatmap toggle to toolbar
    QAction* simAction = toolbar->addAction("Show/Hide Heatmap");
    simAction->setCheckable(true);
    connect(simAction, &QAction::triggered, this, &MainWindow::onToggleSimulation);
    connect(simulationAction_, &QAction::toggled, simAction, &QAction::setChecked);
    connect(simAction, &QAction::toggled, simulationAction_, &QAction::setChecked);
}

void MainWindow::setupControlPanel() {
    controlPanel_ = new ControlPanel(this);
    addDockWidget(Qt::RightDockWidgetArea, controlPanel_);

    // Connect control panel signals
    connect(controlPanel_, &ControlPanel::findNearestRequested,
            this, &MainWindow::onFindNearestRequested);
    connect(controlPanel_, &ControlPanel::computePathRequested,
            this, &MainWindow::onComputePathRequested);
    connect(controlPanel_, &ControlPanel::showTrafficNearRequested,
            this, &MainWindow::onShowTrafficNearRequested);
    connect(controlPanel_, &ControlPanel::clearHighlightsRequested,
            this, &MainWindow::onClearHighlightsRequested);
}

void MainWindow::loadGraph(const Graph& graph) {
    mapScene_->loadMap(graph);
    mapScene_->setGraph(graph_.get());
    updatePathfinder();
    rebuildQuadTree();
    mapView_->zoomToFit();

    // Update control panel ranges
    if (graph.getNodeCount() > 0) {
        controlPanel_->setNodeIdRange(0, static_cast<int>(graph.getNodeCount()) - 1);
        controlPanel_->setCoordinateRange(0.0, mapWidth_, 0.0, mapHeight_);
    }

    statusBar()->showMessage(QString("Loaded map: %1 nodes, %2 edges. Click nodes or use Control Panel.")
                                 .arg(graph.getNodeCount())
                                 .arg(graph.getEdgeCount()));
}

void MainWindow::generateNewMap(int numNodes, double width, double height) {
    // Stop simulation timer temporarily
    simulationTimer_->stop();

    mapWidth_ = width;
    mapHeight_ = height;

    statusBar()->showMessage("Generating map...");

    // Clear and regenerate
    graph_ = std::make_unique<Graph>();
    generator_->generate(*graph_, numNodes, width, height);

    std::cout << "Generated map: " << graph_->getNodeCount() << " nodes, "
              << graph_->getEdgeCount() << " edges" << std::endl;

    // Create new simulator for the new graph (uses Dijkstra for route computation)
    simulator_ = std::make_unique<TrafficSimulator>(*graph_, dijkstraPathfinder_.get());

    // Scale the default traffic load to the generated graph size so the heatmap
    // can surface non-green states on the default 10k-node map.
    static constexpr int kEdgesPerSpawn = 500;
    static constexpr int kMinSpawnRate  = 20;
    static constexpr int kMaxSpawnRate  = 40;
    static constexpr int kEdgeToCarDiv  = 4;
    static constexpr int kMinMaxCars    = 3000;
    static constexpr int kMaxMaxCars    = 5000;

    const int edgeCount = static_cast<int>(graph_->getEdgeCount());
    simulator_->setSpawnRate(std::clamp(edgeCount / kEdgesPerSpawn, kMinSpawnRate, kMaxSpawnRate));
    simulator_->setMaxCars(std::clamp(edgeCount / kEdgeToCarDiv, kMinMaxCars, kMaxMaxCars));

    // Set simulation running BEFORE loadGraph so updatePathfinder() picks DynamicPathFinder
    simulationRunning_ = true;

    // Load into scene
    loadGraph(*graph_);

    // Start traffic simulation in background (always running)
    simulationTimer_->start(100);  // 100ms interval
    std::cout << "Traffic simulation started (background)" << std::endl;

    // Reset heatmap visibility to off by default
    heatmapVisible_ = false;
    mapScene_->setHeatmapVisible(false);
    simulationAction_->setChecked(false);
}

void MainWindow::rebuildQuadTree() {
    if (!graph_) return;

    BoundingBox bounds(0.0, 0.0, mapWidth_, mapHeight_);
    quadTree_ = std::make_unique<QuadTree>(bounds);

    for (const auto& nodePair : graph_->getNodes()) {
        const Node& node = nodePair.second;
        quadTree_->insert(node.getId(), node.getPosition());
    }

    std::cout << "QuadTree rebuilt with " << graph_->getNodeCount() << " nodes" << std::endl;
}

void MainWindow::updatePathfinder() {
    // Use dynamic pathfinder when simulation is running, otherwise use Dijkstra
    if (simulationRunning_) {
        mapScene_->setPathFinder(dynamicPathfinder_.get());
    } else {
        mapScene_->setPathFinder(dijkstraPathfinder_.get());
    }
}

void MainWindow::onGenerateMap() {
    GenerateMapDialog dialog(this);
    if (dialog.exec() == QDialog::Accepted) {
        int nodeCount = dialog.getNodeCount();
        double width = dialog.getMapWidth();
        double height = dialog.getMapHeight();
        generateNewMap(nodeCount, width, height);
    }
}

void MainWindow::onZoomToFit() {
    mapView_->zoomToFit();
}

void MainWindow::onToggleSimulation() {
    // Toggle heatmap visibility (simulation always runs in background)
    heatmapVisible_ = !heatmapVisible_;
    mapScene_->setHeatmapVisible(heatmapVisible_);

    if (heatmapVisible_) {
        simulationAction_->setText("&Hide Traffic Heatmap");
        statusBar()->showMessage("Traffic heatmap visible. Colors show congestion levels.");
        std::cout << "Traffic heatmap enabled" << std::endl;

        // Immediately update all edge colors to show current traffic state
        if (simulator_ && graph_) {
            for (const auto& edgePair : graph_->getEdges()) {
                const Edge& edge = edgePair.second;
                int status = TrafficModel::getCongestionStatus(
                    edge.getCapacity(),
                    static_cast<double>(edge.getCarCount())
                );
                mapScene_->updateEdgeCongestion(edge.getId(), status);
            }
        }
    } else {
        simulationAction_->setText("&Show Traffic Heatmap");
        statusBar()->showMessage("Traffic heatmap hidden. Edges show default green.");
        std::cout << "Traffic heatmap disabled" << std::endl;

        // Reset all edges to default green color
        if (graph_) {
            for (const auto& edgePair : graph_->getEdges()) {
                mapScene_->updateEdgeCongestion(edgePair.first, 0);  // 0 = green
            }
        }
    }

    simulationAction_->setChecked(heatmapVisible_);
}

void MainWindow::onSimulationStep() {
    if (simulator_ && graph_) {
        // Always run the simulation step (traffic data updates in background)
        simulator_->step();

        // Only update visuals if heatmap is visible
        if (heatmapVisible_) {
            for (Edge::Id edgeId : simulator_->getChangedEdges()) {
                const Edge* edge = graph_->getEdge(edgeId);
                if (edge) {
                    int status = TrafficModel::getCongestionStatus(
                        edge->getCapacity(),
                        static_cast<double>(edge->getCarCount())
                    );
                    mapScene_->updateEdgeCongestion(edgeId, status);
                }
            }
        }

        // Update localized traffic view if active
        if (!mapScene_->getTrafficHighlightedEdges().empty()) {
            mapScene_->updateTrafficHighlights(*graph_);
        }
    }
}

void MainWindow::onPathFound(const PathResult& result) {
    std::cout << "Path found: " << result.pathNodes.size() << " nodes, cost: "
              << result.totalCost << std::endl;
    controlPanel_->showPathResult(static_cast<int>(result.pathNodes.size()),
                                   result.totalCost, result.found);
}

void MainWindow::onStatusMessage(const QString& message) {
    statusBar()->showMessage(message);
}

void MainWindow::onFindNearestRequested(double x, double y, int k) {
    if (!quadTree_ || !graph_) {
        controlPanel_->showSpatialQueryResult(0, 0);
        statusBar()->showMessage("QuadTree not initialized!");
        return;
    }

    std::cout << "Finding " << k << " nearest nodes to (" << x << ", " << y << ")" << std::endl;

    // Show query point on map
    mapScene_->showQueryPoint(x, y);

    // Find k nearest nodes
    Point2D queryPoint(x, y);
    std::vector<Node::Id> nearestNodes = quadTree_->findKNearest(queryPoint, static_cast<size_t>(k));

    // Highlight the found nodes (Cyan color)
    mapScene_->highlightNodes(nearestNodes, QColor(0, 188, 212));  // Cyan

    // Collect all edges associated with the found nodes
    std::unordered_set<Edge::Id> associatedEdgeSet;
    for (Node::Id nodeId : nearestNodes) {
        const auto& edges = graph_->getAdjacentEdges(nodeId);
        for (Edge::Id edgeId : edges) {
            const Edge* edge = graph_->getEdge(edgeId);
            if (edge) {
                associatedEdgeSet.insert(edgeId);
            }
        }
    }

    // Highlight associated edges
    std::vector<Edge::Id> associatedEdges(associatedEdgeSet.begin(), associatedEdgeSet.end());
    mapScene_->highlightEdges(associatedEdges, QColor(0, 188, 212));  // Cyan

    int edgeCount = static_cast<int>(associatedEdges.size());

    std::cout << "Found " << nearestNodes.size() << " nodes with " << edgeCount << " connected edges" << std::endl;

    // Auto-focus on the query point
    mapView_->focusOnPoint(x, y, 2.0);

    controlPanel_->showSpatialQueryResult(static_cast<int>(nearestNodes.size()), edgeCount);
    statusBar()->showMessage(QString("Spatial Query: Found %1 nearest nodes with %2 connected edges")
                                 .arg(nearestNodes.size())
                                 .arg(edgeCount));
}

void MainWindow::onComputePathRequested(uint32_t startId, uint32_t endId, RoutingCriteria criteria) {
    if (!graph_) {
        controlPanel_->showPathResult(0, 0.0, false);
        statusBar()->showMessage("Graph not initialized!");
        return;
    }

    std::cout << "Computing path from Node " << startId << " to Node " << endId;
    std::cout << " using " << (criteria == RoutingCriteria::ShortestDistance ? "Shortest Distance (F2)" : "Fastest Time (F4)") << std::endl;

    // Clear spatial highlights before showing path
    mapScene_->clearSpatialHighlights();

    // Select pathfinder based on routing criteria (decoupled from simulation state)
    PathFinder* selectedPathfinder = nullptr;
    if (criteria == RoutingCriteria::ShortestDistance) {
        selectedPathfinder = dijkstraPathfinder_.get();
    } else {
        selectedPathfinder = dynamicPathfinder_.get();
    }

    // Temporarily set the pathfinder for this query
    mapScene_->setPathFinder(selectedPathfinder);

    // Use MapScene's findPathById which handles visualization
    PathResult result = mapScene_->findPathById(startId, endId);

    // Auto-focus on the path bounds
    if (result.found && result.pathNodes.size() >= 2) {
        // Calculate bounding rect of the path
        double minX = std::numeric_limits<double>::max();
        double minY = std::numeric_limits<double>::max();
        double maxX = std::numeric_limits<double>::lowest();
        double maxY = std::numeric_limits<double>::lowest();

        for (Node::Id nodeId : result.pathNodes) {
            const Node* node = graph_->getNode(nodeId);
            if (node) {
                const Point2D& pos = node->getPosition();
                minX = std::min(minX, pos.x);
                minY = std::min(minY, pos.y);
                maxX = std::max(maxX, pos.x);
                maxY = std::max(maxY, pos.y);
            }
        }

        QRectF pathBounds(minX, minY, maxX - minX, maxY - minY);
        mapView_->focusOnBounds(pathBounds, 100.0);
    }

    controlPanel_->showPathResult(static_cast<int>(result.pathNodes.size()),
                                   result.totalCost, result.found);
}

void MainWindow::onShowTrafficNearRequested(double x, double y, double radius) {
    if (!quadTree_ || !graph_) {
        controlPanel_->showTrafficResult(0);
        statusBar()->showMessage("QuadTree not initialized!");
        return;
    }

    std::cout << "Showing traffic near (" << x << ", " << y << ") radius " << radius << std::endl;

    // Clear previous traffic highlights
    mapScene_->clearTrafficHighlights();

    // Find all nodes within radius using QuadTree range query
    BoundingBox queryBox(x - radius, y - radius, x + radius, y + radius);
    std::vector<Node::Id> nearbyNodes = quadTree_->queryRange(queryBox);

    // Collect all edges adjacent to nearby nodes (roads near the coordinate)
    std::unordered_set<Edge::Id> nearbyEdgeSet;
    for (Node::Id nodeId : nearbyNodes) {
        const auto& edges = graph_->getAdjacentEdges(nodeId);
        for (Edge::Id edgeId : edges) {
            const Edge* edge = graph_->getEdge(edgeId);
            if (edge) {
                nearbyEdgeSet.insert(edgeId);
            }
        }
    }

    std::vector<Edge::Id> nearbyEdges(nearbyEdgeSet.begin(), nearbyEdgeSet.end());

    // Show traffic colors on those edges
    mapScene_->showTrafficEdges(nearbyEdges, *graph_);

    // Show traffic query point marker
    mapScene_->showTrafficPoint(x, y);

    // Auto-focus on the area
    QRectF viewBounds(x - radius, y - radius, radius * 2, radius * 2);
    mapView_->focusOnBounds(viewBounds, 50.0);

    controlPanel_->showTrafficResult(static_cast<int>(nearbyEdges.size()));
    statusBar()->showMessage(QString("Traffic View: Showing %1 roads near (%2, %3)")
                                 .arg(nearbyEdges.size())
                                 .arg(x, 0, 'f', 0)
                                 .arg(y, 0, 'f', 0));
}

void MainWindow::onClearHighlightsRequested() {
    mapScene_->clearSpatialHighlights();
    mapScene_->clearTrafficHighlights();
    mapScene_->clearPathSelection();
    statusBar()->showMessage("All highlights cleared.");
}

} // namespace nav
