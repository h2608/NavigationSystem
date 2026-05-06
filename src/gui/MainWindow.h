#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>
#include <memory>
#include "core/graph/Graph.h"
#include "core/generation/HierarchicalRoadGenerator.h"
#include "core/algorithms/DijkstraPathFinder.h"
#include "core/algorithms/DynamicPathFinder.h"
#include "core/traffic/TrafficSimulator.h"
#include "core/spatial/QuadTree.h"
#include "core/io/GraphIO.h"
#include "gui/widgets/ControlPanel.h"

namespace nav {

class MapView;
class MapScene;

class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    explicit MainWindow(QWidget* parent = nullptr);
    ~MainWindow();

    // 加载并显示图
    void loadGraph(const Graph& graph);

    // 生成新的随机地图
    void generateNewMap(int numNodes = 3000, double width = 5000.0, double height = 5000.0);

private slots:
    void onGenerateMap();
    void onSaveMap();
    void onLoadMap();
    void onZoomToFit();
    void onToggleSimulation();
    void onSimulationStep();
    void onPathFound(const PathResult& result);
    void onStatusMessage(const QString& message);
    void onViewInteractionBegan();
    void onViewInteractionEnded();

    // 控制面板槽
    void onFindNearestRequested(double x, double y, int k);
    void onComputePathRequested(uint32_t startId, uint32_t endId, RoutingCriteria criteria);
    void onShowTrafficNearRequested(double x, double y, double radius);
    void onClearHighlightsRequested();

private:
    void setupUi();
    void setupMenuBar();
    void setupToolBar();
    void setupControlPanel();
    void updatePathfinder();
    void rebuildQuadTree();
    void startSimulation();

    MapView* mapView_;
    MapScene* mapScene_;
    ControlPanel* controlPanel_;

    std::unique_ptr<Graph> graph_;
    std::unique_ptr<HierarchicalRoadGenerator> generator_;
    std::unique_ptr<TrafficSimulator> simulator_;
    std::unique_ptr<DijkstraPathFinder> dijkstraPathfinder_;
    std::unique_ptr<DynamicPathFinder> dynamicPathfinder_;
    std::unique_ptr<QuadTree> quadTree_;

    QTimer* simulationTimer_;
    QAction* simulationAction_;
    bool simulationRunning_ = false;
    bool simulationSuspendedForInteraction_ = false;
    bool heatmapVisible_ = false;  // 可视化热力图显示的开关

    double mapWidth_ = 5000.0;
    double mapHeight_ = 5000.0;
};

} // namespace nav

#endif // MAINWINDOW_H
