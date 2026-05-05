#include "gui/MainWindow.h"
#include "gui/view/MapView.h"
#include "gui/view/MapScene.h"
#include "gui/widgets/ControlPanel.h"
#include "gui/widgets/GenerateMapDialog.h"
#include "core/spatial/BoundingBox.h"
#include "core/io/GraphIO.h"
#include <QMenuBar>
#include <QToolBar>
#include <QAction>
#include <QStatusBar>
#include <QLabel>
#include <QFileDialog>
#include <QMessageBox>
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
    , generator_(std::make_unique<HierarchicalRoadGenerator>())
    , dijkstraPathfinder_(std::make_unique<DijkstraPathFinder>())
    , dynamicPathfinder_(std::make_unique<DynamicPathFinder>())
    , simulationTimer_(new QTimer(this))
    , simulationAction_(nullptr)
{
    setupUi();
    setupMenuBar();
    setupToolBar();
    setupControlPanel();

    // 设置仿真定时器
    connect(simulationTimer_, &QTimer::timeout, this, &MainWindow::onSimulationStep);

    // 连接场景信号
    connect(mapScene_, &MapScene::pathFound, this, &MainWindow::onPathFound);
    connect(mapScene_, &MapScene::statusMessage, this, &MainWindow::onStatusMessage);
    connect(mapScene_, &MapScene::errorOccurred, this, [this](const QString& title, const QString& msg) {
        QMessageBox::warning(this, title, msg);
        statusBar()->showMessage(msg);
    });

    // 设置窗口属性
    setWindowTitle("导航系统");
    resize(1400, 900);

    // 显示状态
    statusBar()->showMessage("就绪。使用\"生成地图\"创建新地图，点击节点或使用控制面板。");
}

MainWindow::~MainWindow() {
    simulationTimer_->stop();
}

void MainWindow::setupUi() {
    // 创建场景和视图
    mapScene_ = new MapScene(this);
    mapView_ = new MapView(this);
    mapView_->setScene(mapScene_);

    // 设置为中心部件
    setCentralWidget(mapView_);
}

void MainWindow::setupMenuBar() {
    // 文件菜单
    QMenu* fileMenu = menuBar()->addMenu("文件(&F)");

    QAction* generateAction = fileMenu->addAction("生成新地图(&G)");
    generateAction->setShortcut(QKeySequence("Ctrl+G"));
    connect(generateAction, &QAction::triggered, this, &MainWindow::onGenerateMap);

    QAction* saveAction = fileMenu->addAction("保存地图(&S)");
    saveAction->setShortcut(QKeySequence("Ctrl+S"));
    connect(saveAction, &QAction::triggered, this, &MainWindow::onSaveMap);

    QAction* loadAction = fileMenu->addAction("加载地图(&L)");
    loadAction->setShortcut(QKeySequence("Ctrl+O"));
    connect(loadAction, &QAction::triggered, this, &MainWindow::onLoadMap);

    fileMenu->addSeparator();

    QAction* exitAction = fileMenu->addAction("退出(&X)");
    exitAction->setShortcut(QKeySequence::Quit);
    connect(exitAction, &QAction::triggered, this, &QMainWindow::close);

    // 视图菜单
    QMenu* viewMenu = menuBar()->addMenu("视图(&V)");

    QAction* zoomFitAction = viewMenu->addAction("适应窗口(&F)");
    zoomFitAction->setShortcut(QKeySequence("Ctrl+0"));
    connect(zoomFitAction, &QAction::triggered, this, &MainWindow::onZoomToFit);

    // 仿真菜单
    QMenu* simMenu = menuBar()->addMenu("仿真(&S)");

    simulationAction_ = simMenu->addAction("显示交通热力图(&T)");
    simulationAction_->setShortcut(QKeySequence("Ctrl+T"));
    simulationAction_->setCheckable(true);
    connect(simulationAction_, &QAction::triggered, this, &MainWindow::onToggleSimulation);
}

void MainWindow::setupToolBar() {
    QToolBar* toolbar = addToolBar("主工具栏");
    toolbar->setMovable(false);

    QAction* generateAction = toolbar->addAction("生成地图");
    connect(generateAction, &QAction::triggered, this, &MainWindow::onGenerateMap);

    QAction* saveToolAction = toolbar->addAction("保存地图");
    connect(saveToolAction, &QAction::triggered, this, &MainWindow::onSaveMap);

    QAction* loadToolAction = toolbar->addAction("加载地图");
    connect(loadToolAction, &QAction::triggered, this, &MainWindow::onLoadMap);

    QAction* zoomFitAction = toolbar->addAction("适应窗口");
    connect(zoomFitAction, &QAction::triggered, this, &MainWindow::onZoomToFit);

    toolbar->addSeparator();

    // 添加热力图切换到工具栏
    QAction* simAction = toolbar->addAction("显示/隐藏热力图");
    simAction->setCheckable(true);
    connect(simAction, &QAction::triggered, this, &MainWindow::onToggleSimulation);
    connect(simulationAction_, &QAction::toggled, simAction, &QAction::setChecked);
    connect(simAction, &QAction::toggled, simulationAction_, &QAction::setChecked);
}

void MainWindow::setupControlPanel() {
    controlPanel_ = new ControlPanel(this);
    addDockWidget(Qt::RightDockWidgetArea, controlPanel_);

    // 连接控制面板信号
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
    mapScene_->buildClusters(graph, mapWidth_, mapHeight_);
    mapScene_->setGraph(graph_.get());
    updatePathfinder();
    rebuildQuadTree();
    mapView_->zoomToFit();

    // 更新控制面板范围
    if (graph.getNodeCount() > 0) {
        controlPanel_->setNodeIdRange(0, static_cast<int>(graph.getNodeCount()) - 1);
        controlPanel_->setCoordinateRange(0.0, mapWidth_, 0.0, mapHeight_);
    }

    statusBar()->showMessage(QString("已加载地图: %1 个节点, %2 条边。点击节点或使用控制面板。")
                                 .arg(graph.getNodeCount())
                                 .arg(graph.getEdgeCount()));
}

void MainWindow::generateNewMap(int numNodes, double width, double height) {
    // 暂时停止仿真定时器
    simulationTimer_->stop();

    mapWidth_ = width;
    mapHeight_ = height;

    statusBar()->showMessage("正在生成地图...");

    // 清除并重新生成
    graph_ = std::make_unique<Graph>();
    generator_->generate(*graph_, numNodes, width, height);

    std::cout << "Generated map: " << graph_->getNodeCount() << " nodes, "
              << graph_->getEdgeCount() << " edges" << std::endl;

    startSimulation();
}

void MainWindow::startSimulation() {
    simulator_ = std::make_unique<TrafficSimulator>(*graph_, dijkstraPathfinder_.get());

    // 根据图大小缩放交通负载
    static constexpr int kEdgesPerSpawn = 500;
    static constexpr int kMinSpawnRate  = 5;
    static constexpr int kMaxSpawnRate  = 30;
    static constexpr int kEdgeToCarDiv  = 4;
    static constexpr int kMinMaxCars    = 500;
    static constexpr int kMaxMaxCars    = 5000;

    const int edgeCount = static_cast<int>(graph_->getEdgeCount());
    simulator_->setSpawnRate(std::clamp(edgeCount / kEdgesPerSpawn, kMinSpawnRate, kMaxSpawnRate));
    simulator_->setMaxCars(std::clamp(edgeCount / kEdgeToCarDiv, kMinMaxCars, kMaxMaxCars));

    // 在 loadGraph 之前设置仿真运行，以便 updatePathfinder() 选择 DynamicPathFinder
    simulationRunning_ = true;

    loadGraph(*graph_);

    simulationTimer_->start(100);
    std::cout << "Traffic simulation started (background)" << std::endl;

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
    // 仿真运行时使用动态路径查找器，否则使用 Dijkstra
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
    // 切换热力图可见性（仿真始终在后台运行）
    heatmapVisible_ = !heatmapVisible_;
    mapScene_->setHeatmapVisible(heatmapVisible_);

    if (heatmapVisible_) {
        simulationAction_->setText("隐藏交通热力图(&T)");
        statusBar()->showMessage("交通热力图已显示。颜色表示拥堵程度。");
        std::cout << "Traffic heatmap enabled" << std::endl;

        // 立即更新所有边颜色以显示当前交通状态
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
        simulationAction_->setText("显示交通热力图(&T)");
        statusBar()->showMessage("交通热力图已隐藏。道路显示默认绿色。");
        std::cout << "Traffic heatmap disabled" << std::endl;

        // 将所有边重置为默认绿色
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
        // 始终运行仿真步骤（交通数据在后台更新）
        simulator_->step();

        // 仅在热力图可见时更新视觉效果
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

        // 如果活动则更新局部交通视图
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
        QMessageBox::warning(this, "查询失败", "四叉树未初始化！请先生成或加载地图。");
        statusBar()->showMessage("四叉树未初始化！");
        return;
    }

    std::cout << "Finding " << k << " nearest nodes to (" << x << ", " << y << ")" << std::endl;

    // 在地图上显示查询点
    mapScene_->showQueryPoint(x, y);

    // 查找 k 个最近节点
    Point2D queryPoint(x, y);
    std::vector<Node::Id> nearestNodes = quadTree_->findKNearest(queryPoint, static_cast<size_t>(k));

    // 高亮显示找到的节点（青色）
    mapScene_->highlightNodes(nearestNodes, QColor(0, 188, 212));  // 青色

    // 收集与找到的节点关联的所有边
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

    // 高亮显示关联的边
    std::vector<Edge::Id> associatedEdges(associatedEdgeSet.begin(), associatedEdgeSet.end());
    mapScene_->highlightEdges(associatedEdges, QColor(0, 188, 212));  // 青色

    int edgeCount = static_cast<int>(associatedEdges.size());

    std::cout << "Found " << nearestNodes.size() << " nodes with " << edgeCount << " connected edges" << std::endl;

    // 自动聚焦到查询点
    mapView_->focusOnPoint(x, y, 2.0);

    controlPanel_->showSpatialQueryResult(static_cast<int>(nearestNodes.size()), edgeCount);
    statusBar()->showMessage(QString("空间查询: 找到 %1 个最近节点，关联 %2 条边")
                                 .arg(nearestNodes.size())
                                 .arg(edgeCount));
}

void MainWindow::onComputePathRequested(uint32_t startId, uint32_t endId, RoutingCriteria criteria) {
    if (!graph_) {
        controlPanel_->showPathResult(0, 0.0, false);
        QMessageBox::warning(this, "计算失败", "图数据未初始化！请先生成或加载地图。");
        statusBar()->showMessage("图数据未初始化！");
        return;
    }

    // 验证节点 ID
    if (!graph_->getNode(startId)) {
        controlPanel_->showPathResult(0, 0.0, false);
        QMessageBox::warning(this, "计算失败",
            QString("起始节点 ID %1 不存在！有效范围: 0 - %2")
                .arg(startId).arg(graph_->getNodeCount() - 1));
        return;
    }
    if (!graph_->getNode(endId)) {
        controlPanel_->showPathResult(0, 0.0, false);
        QMessageBox::warning(this, "计算失败",
            QString("终止节点 ID %1 不存在！有效范围: 0 - %2")
                .arg(endId).arg(graph_->getNodeCount() - 1));
        return;
    }

    std::cout << "Computing path from Node " << startId << " to Node " << endId;
    std::cout << " using " << (criteria == RoutingCriteria::ShortestDistance ? "Shortest Distance (F2)" : "Fastest Time (F4)") << std::endl;

    // 在显示路径之前清除空间高亮
    mapScene_->clearSpatialHighlights();

    // 根据路由策略选择路径查找器（与仿真状态解耦）
    PathFinder* selectedPathfinder = nullptr;
    if (criteria == RoutingCriteria::ShortestDistance) {
        selectedPathfinder = dijkstraPathfinder_.get();
    } else {
        selectedPathfinder = dynamicPathfinder_.get();
    }

    // 临时设置此查询的路径查找器
    mapScene_->setPathFinder(selectedPathfinder);

    // 使用 MapScene 的 findPathById 处理可视化
    PathResult result = mapScene_->findPathById(startId, endId);

    // 自动聚焦到路径边界
    if (result.found && result.pathNodes.size() >= 2) {
        // 计算路径的边界矩形
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
        QMessageBox::warning(this, "查询失败", "四叉树未初始化！请先生成或加载地图。");
        statusBar()->showMessage("四叉树未初始化！");
        return;
    }

    std::cout << "Showing traffic near (" << x << ", " << y << ") radius " << radius << std::endl;

    // 清除之前的交通高亮
    mapScene_->clearTrafficHighlights();

    // 使用四叉树范围查询查找半径内的所有节点
    BoundingBox queryBox(x - radius, y - radius, x + radius, y + radius);
    std::vector<Node::Id> nearbyNodes = quadTree_->queryRange(queryBox);

    // 收集邻近节点的所有相邻边（坐标附近的道路）
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

    // 在这些边上显示交通颜色
    mapScene_->showTrafficEdges(nearbyEdges, *graph_);

    // 显示交通查询点标记
    mapScene_->showTrafficPoint(x, y);

    // 自动聚焦到该区域
    QRectF viewBounds(x - radius, y - radius, radius * 2, radius * 2);
    mapView_->focusOnBounds(viewBounds, 50.0);

    controlPanel_->showTrafficResult(static_cast<int>(nearbyEdges.size()));
    statusBar()->showMessage(QString("交通视图: 显示 (%2, %3) 附近 %1 条道路")
                                 .arg(nearbyEdges.size())
                                 .arg(x, 0, 'f', 0)
                                 .arg(y, 0, 'f', 0));
}

void MainWindow::onClearHighlightsRequested() {
    mapScene_->clearSpatialHighlights();
    mapScene_->clearTrafficHighlights();
    mapScene_->clearPathSelection();
    statusBar()->showMessage("所有高亮已清除。");
}

void MainWindow::onSaveMap() {
    if (!graph_ || graph_->getNodeCount() == 0) {
        QMessageBox::warning(this, "保存失败", "当前没有可保存的地图！请先生成地图。");
        return;
    }

    QString filePath = QFileDialog::getSaveFileName(
        this, "保存地图", "", "JSON 地图文件 (*.json);;所有文件 (*)");

    if (filePath.isEmpty()) {
        return;
    }

    statusBar()->showMessage("正在保存地图...");

    bool success = GraphIO::save(filePath.toStdString(), *graph_, mapWidth_, mapHeight_);

    if (success) {
        statusBar()->showMessage(QString("地图已保存: %1").arg(filePath));
    } else {
        QMessageBox::warning(this, "保存失败",
            QString("无法保存地图文件！\n%1").arg(
                QString::fromStdString(GraphIO::getLastError())));
        statusBar()->showMessage("地图保存失败！");
    }
}

void MainWindow::onLoadMap() {
    QString filePath = QFileDialog::getOpenFileName(
        this, "加载地图", "", "JSON 地图文件 (*.json);;所有文件 (*)");

    if (filePath.isEmpty()) {
        return;
    }

    statusBar()->showMessage("正在加载地图...");

    MapFileData fileData;
    bool success = GraphIO::load(filePath.toStdString(), fileData);

    if (!success) {
        QMessageBox::warning(this, "加载失败",
            QString("无法加载地图文件！\n%1").arg(
                QString::fromStdString(GraphIO::getLastError())));
        statusBar()->showMessage("地图加载失败！");
        return;
    }

    // 停止现有仿真
    simulationTimer_->stop();

    mapWidth_ = fileData.width;
    mapHeight_ = fileData.height;
    graph_ = std::make_unique<Graph>(std::move(fileData.graph));

    startSimulation();

    statusBar()->showMessage(QString("地图已加载: %1 个节点, %2 条边。文件: %3")
                                 .arg(graph_->getNodeCount())
                                 .arg(graph_->getEdgeCount())
                                 .arg(filePath));
}

} // namespace nav
